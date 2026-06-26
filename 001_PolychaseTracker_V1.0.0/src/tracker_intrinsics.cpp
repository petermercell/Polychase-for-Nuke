// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Peter Mercell
//
// Developed with assistance from Claude (Anthropic).

// =============================================================================
// tracker_intrinsics.cpp — background, NON-DESTRUCTIVE focal / principal-point
// solve for the PolychaseTracker plugin.
//
// WHY THIS EXISTS (and why it is not just a checkbox on Track/Refine):
//   The upstream solvers cannot hold an arbitrary pose fixed. PnP
//   (pnp/solvers.cc) always optimises the camera pose — its Parameters carry R
//   and the full CameraState, and optimize_focal_length/principal only ADD
//   intrinsic DOFs on top. The bundle refiner (refiner.cc) is the same: its
//   Step() does pose.t += dp, rebuilds R from the increment, and only then adds
//   focal/principal; it pins ONLY the first/last cameras as ground truth, never
//   an interior frame. So routing the intrinsic solve through Track or Refine
//   necessarily RE-SOLVES and overwrites the pose — destroying the track the
//   artist already hand-tracked and refined. That is the bug we are escaping.
//
//   This file instead FREEZES R,t at the kept pose (pose_translate/rotate/scale
//   curves) and fits the intrinsic alone. With R,t fixed the pinhole projection
//       u = fx*(X/Z) + cx ,   v = fy*(Y/Z) + cy
//   is LINEAR in the unknowns, so each frame is a tiny weighted least-squares
//   (1 DOF for focal: fy, with fx = aspect*fy locked, matching the refiner's
//   focal parametrisation; 2 DOF for the principal point) solved over the SAME
//   optical-flow-DB raycast correspondences Track builds. A few IRLS passes with
//   a Huber weight reject outliers, and the fy/cx/cy are clamped to the same
//   CameraIntrinsics::GetBounds the upstream solver uses.
//
//   It writes ONLY the solved_focal / solved_cx / solved_cy curves — it never
//   calls key_pose_matrix_at — so the tracked+refined pose is untouched by
//   construction. Export then keys these onto the spawned Camera2 when the
//   matching "Export Solved …" box is ticked (the box is an EXPORT gate now, not
//   a solve switch). Solve first (button), then tick to export.
//
//   The two buttons are independent: "Solve Focal" fits fy holding the principal
//   point at whatever is current (solved curve if present, else the seed centre);
//   "Solve Principal Point" fits cx,cy holding the focal at whatever is current.
//   Neither clears the other's curve, so pressing one never destroys the other.
//
// CONVENTION: identical to on_track — OpenCV / Y-down / real-pixel intrinsics,
//   the gl_to_cv_flip() on the view, the seed intrinsics from
//   tracker_intrinsics(cam->projection(), w, h), and the same fx_px*haperture/w
//   pixel->mm focal map. The per-frame view is reconstructed from the kept pose:
//       view_cv(t) = view_cv_seed * model'(t) * model0^-1
//   (the inverse of Track's model'(t) = view_cv_seed^-1 * view_cv(t) * model0).
// =============================================================================
#include "polychase_tracker.h"

// Shared OpenGL<->OpenCV convention bridge (one definition for Track/Refine/
// user-track anchoring / intrinsics); aliased to the icp_ names below.
#include "pcn_convention.h"

// Upstream (C++17-clean) headers, same set tracker_track.cpp pulls. tracker.h
// transitively provides Database, RayCast/AcceleratedMesh, SceneTransformations,
// CameraIntrinsics and the keypoint/flow types.
#include "tracker.h"
#include "database.h"
#include "ray_casting.h"

// Qt-free progress facade (no-op when headless / Qt-free), same as Track.
#include "track_progress.h"

#include <Eigen/LU>          // RowMajorMatrix4f::inverse()
#include <Eigen/Eigenvalues> // SelfAdjointEigenSolver — coplanarity test (zoom guard)

#include <algorithm>         // std::clamp, std::min, std::max
#include <cmath>
#include <map>
#include <memory>            // std::shared_ptr
#include <optional>          // std::optional<RayHit>
#include <sstream>
#include <string>
#include <vector>

using namespace DD::Image;

namespace pcn {

// -----------------------------------------------------------------------------
// Convention helpers. The GL<->CV bridge is shared (pcn_convention.h); aliased
// here under the icp_ names so call sites are unchanged and the math can never
// drift from Track / Refine. icp_cam_imatrix_at stays local (it samples a camera
// at a frame, restoring its OutputContext).
// -----------------------------------------------------------------------------

constexpr auto icp_gl_to_cv_flip      = &conv::gl_to_cv_flip;
constexpr auto icp_tracker_intrinsics = &conv::tracker_intrinsics;

// world->camera (imatrix) of a CameraOp sampled at a specific frame. Mirror of
// tracker_track.cpp::cam_imatrix_at.
static DD::Image::Matrix4 icp_cam_imatrix_at(DD::Image::CameraOp* cam, double frame)
{
    OutputContext orig = cam->outputContext();
    OutputContext c    = orig;
    c.setFrame(frame);
    cam->setOutputContext(c);
    cam->validate(true);
    const DD::Image::Matrix4 m = cam->imatrix();
    cam->setOutputContext(orig);
    cam->validate(true);
    return m;
}

// LOCAL-space embree mesh — shared definition (pcn_convention.h).
constexpr auto icp_build_local_accel_mesh = &conv::build_local_accel_mesh;


// One linearised observation at the target frame: with R,t frozen the camera-
// space ray direction (a,b) = (Xc/Zc, Yc/Zc) is constant, and the measured pixel
// is (u,v). The pinhole maps (a,b)->(fx*a+cx, fy*b+cy), linear in fx/fy/cx/cy.
struct IcpObs {
    double a, b;     // Xc/Zc, Yc/Zc at the FIXED target-frame pose
    double u, v;     // measured pixel (OpenCV / y-down / real px)
    double w;        // flow-error weight
};


// -----------------------------------------------------------------------------
// solve_intrinsics_impl — the shared worker behind both buttons.
//   do_focal     : fit fy (fx = aspect*fy), holding cx,cy at the current value.
//   do_principal : fit cx,cy, holding fx,fy at the current value.
// Exactly one of the two is true per call. Writes only the corresponding
// solved_* curve(s); the pose curves are never touched.
// -----------------------------------------------------------------------------
bool PolychaseTracker::solve_intrinsics_impl(bool do_focal, bool do_principal)
{
    const char* what = do_focal ? "focal length" : "principal point";
    std::ostringstream oss;
    oss << "[" << timestamp() << "] Solve " << what << " (background)\n";

    // ---- inputs / state ----
    Iop*      img = input_img();
    CameraOp* cam = input_cam();
    Op*       geo = input_geo_op();
    if (!img) { oss << "  [FAIL] input 0 (img) not connected.";              set_status(oss.str()); return false; }
    if (!cam) { oss << "  [FAIL] input 1 (cam) not connected.";              set_status(oss.str()); return false; }
    if (!geo) { oss << "  [FAIL] input 2 (geo) not connected.";              set_status(oss.str()); return false; }
    if (!track_only_user_ && (!db_path_ || db_path_[0] == '\0')) {
        oss << "  [FAIL] Database path not set.";                            set_status(oss.str()); return false;
    }
    if (!has_pose_keys()) {
        oss << "  [FAIL] No tracked pose. Track first — the intrinsic solve\n"
            << "  fits the lens against your EXISTING pose without changing it.";
                                                                            set_status(oss.str()); return false;
    }
    if (last_frame_ <= first_frame_) {
        oss << "  [FAIL] Last Frame must be greater than First Frame.";      set_status(oss.str()); return false;
    }

    // ---- dimensions ----
    int w = 0, h = 0;
    try {
        img->validate(true);
        const Format& fmt = img->info().format();
        w = fmt.width();
        h = fmt.height();
    } catch (const std::exception& e) {
        oss << "  [FAIL] could not validate input 0: " << e.what();          set_status(oss.str()); return false;
    }
    if (w <= 0 || h <= 0) {
        oss << "  [FAIL] input 0 reports invalid dimensions.";               set_status(oss.str()); return false;
    }

    // ---- mesh (local space) + mask ----
    GeoMesh gm;
    geo->validate(true);
    if (!extract_mesh(geo, gm)) {
        oss << "  [FAIL] could not extract a mesh from the geo input.";       set_status(oss.str()); return false;
    }
    if (gm.local_vertices.rows() < 3) {
        oss << "  [FAIL] mesh has < 3 vertices.";                            set_status(oss.str()); return false;
    }
    std::shared_ptr<AcceleratedMesh> accel =
        icp_build_local_accel_mesh(gm, build_mask_array((uint32_t)gm.triangles.rows()));

    // ---- bridge constants (identical to on_track) ----
    cam->validate(true);
    const RowMajorMatrix4f flip = icp_gl_to_cv_flip();
    const CameraIntrinsics intr = icp_tracker_intrinsics(cam->projection(), (float)w, (float)h);
    const double aspect = (intr.aspect_ratio > 1e-6f) ? (double)intr.aspect_ratio : 1.0;

    double haperture = 24.576;
    if (Knob* hk = cam->knob("haperture")) haperture = hk->get_value();

    // Clamp bounds — the same ones the upstream PnP/refiner use, so a frame that
    // wants to run away pins to the identical limit as a Track-side solve would.
    const CameraIntrinsics::Bounds bounds = intr.GetBounds(min_fov_deg_, max_fov_deg_);

    // ---- seed-frame bridge basis (kept pose -> per-frame view) ----
    const int seed_frame = (track_seed_frame_ != 0) ? track_seed_frame_ : first_frame_;
    const RowMajorMatrix4f view_cv_seed = flip * nuke_to_eigen_m4(icp_cam_imatrix_at(cam, (double)seed_frame));
    const RowMajorMatrix4f model0       = nuke_to_eigen_m4(pose_matrix_to_nuke((double)seed_frame));
    const RowMajorMatrix4f model0_inv   = model0.inverse();
    const Eigen::Matrix3f  model0_R     = model0.block<3,3>(0,0);
    const Eigen::Vector3f  model0_t     = model0.block<3,1>(0,3);

    // view_cv(t) from the kept pose: the inverse of Track's model'(t) map.
    auto view_at = [&](int frame) -> RowMajorMatrix4f {
        const RowMajorMatrix4f model_prime =
            nuke_to_eigen_m4(pose_matrix_to_nuke((double)frame));
        return view_cv_seed * model_prime * model0_inv;
    };

    // Current intrinsic at a frame (so each button holds the OTHER quantity at
    // whatever has already been solved, else the seed value).
    auto cur_focal_px = [&](int frame, double& fx_out, double& fy_out) {
        if (has_solved_focal()) {
            double mm = (double)solved_focal_;
            if (Knob* sf = knob("solved_focal")) mm = sf->get_value_at((double)frame);
            fx_out = mm * (double)w / haperture;     // inverse of fx_px*haperture/w
            fy_out = (aspect > 1e-6) ? fx_out / aspect : fx_out;
        } else {
            fx_out = (double)intr.fx;
            fy_out = (double)intr.fy;
        }
    };
    auto cur_principal_px = [&](int frame, double& cx_out, double& cy_out) {
        if (has_solved_principal()) {
            cx_out = (double)intr.cx; cy_out = (double)intr.cy;
            if (Knob* kx = knob("solved_cx")) cx_out = kx->get_value_at((double)frame);
            if (Knob* ky = knob("solved_cy")) cy_out = ky->get_value_at((double)frame);
        } else {
            cx_out = (double)intr.cx; cy_out = (double)intr.cy;
        }
    };

    // ---- pin mode: the lens is fit against the PIN MODEL (Phase 7) ----
    // "Use Only User Tracks" fits the lens to the same helper set Track/Refine use:
    // each linked pin at its EXACT mesh vertex with per-frame 2D from resolve_pin_2d
    // (key override, else linked track), or the loaded-track fallback when nothing
    // is linked. Same IRLS fit below; only the per-frame observation source differs.
    UserTracks solve_tracks;
    if (track_only_user_) {
        solve_tracks = build_solve_tracks();
        if ((int)solve_tracks.size() < 3) {
            oss << "  [FAIL] 'Use Only User Tracks' is on but only " << solve_tracks.size()
                << " linked pin / user track(s) (need >=3). Connect pins to tracks "
                   "(or Load Tracks) first.";
            set_status(oss.str()); return false;
        }
    }

    // ---- open the DB once (flow mode only); reuse scratch buffers ----
    std::unique_ptr<Database> database;
    if (!track_only_user_)
        database = std::make_unique<Database>(std::string(db_path_));
    std::vector<int32_t> flow_ids;
    Keypoints            kps;
    ImagePairFlow        flow;
    std::map<int, RowMajorMatrix4f> view_cache;   // per-frame view, lazily filled
    auto get_view = [&](int frame) -> const RowMajorMatrix4f& {
        auto it = view_cache.find(frame);
        if (it == view_cache.end()) it = view_cache.emplace(frame, view_at(frame)).first;
        return it->second;
    };

    // Pin mode has far fewer (exact) correspondences per frame than flow, so the
    // per-frame floor is lower; flow keeps the robust floor of 8.
    const size_t kMinObs    = track_only_user_ ? 4 : 8;  // below this, skip the frame
    constexpr int    kIters     = 4;     // IRLS passes
    constexpr double kHuberPx   = 3.0;   // Huber knee (pixels)

    TrackProgress prog(std::string("Polychase \xE2\x80\x94 solving ") + what);
    auto report = [&](int frame) {
        const int span = last_frame_ - first_frame_;
        const int done = frame - first_frame_;
        prog.set_percent(span > 0 ? (int)((100.0 * done) / span) : 100);
    };

    std::vector<IcpObs> obs;
    obs.reserve(2048);

    int    frames_solved = 0, frames_skipped = 0;
    double sum_rms = 0.0;

    for (int t = first_frame_; t <= last_frame_; ++t) {
        report(t);
        if (prog.cancelled()) { oss << "  [STOP] cancelled at frame " << t << ".\n"; break; }

        obs.clear();
        const RowMajorMatrix4f view_t = get_view(t);

        if (track_only_user_) {
            // Pin-model correspondences: each helper track's exact vertex (LOCAL)
            // through the per-frame pose -> CV camera space (a,b); its 2D at this
            // frame (OpenCV y-down) -> (u,v).
            for (const UserTrack& ut : solve_tracks) {
                const Eigen::Vector2f* o = nullptr;
                for (const UserTrackObservation& ob : ut.observations)
                    if (ob.frame_id == t) { o = &ob.image_point; break; }
                if (!o) continue;
                const Eigen::Vector3f& vloc = ut.object_point;   // LOCAL (exact vertex / anchor)
                const Eigen::Vector3f Xw = model0_R * vloc + model0_t;
                const Eigen::Vector4f Xh(Xw.x(), Xw.y(), Xw.z(), 1.0f);
                const Eigen::Vector4f Pc = view_t * Xh;
                if (Pc.z() <= 1e-6f) continue;
                IcpObs o2;
                o2.a = (double)Pc.x() / (double)Pc.z();
                o2.b = (double)Pc.y() / (double)Pc.z();
                o2.u = (double)o->x();   // helper-track pixel is OpenCV y-down already
                o2.v = (double)o->y();
                o2.w = 1.0;
                obs.push_back(o2);
            }
        }
        else {
        // Gather correspondences observed AT frame t (mirror tracker.cc SolveFrame:
        // raycast each neighbour's source keypoint against the mesh at the
        // neighbour's pose, pair with the target keypoint at t).
        database->FindOpticalFlowsToImage(t, flow_ids);
        for (int32_t f : flow_ids) {
            if (f == t || f < first_frame_ || f > last_frame_) continue;
            database->ReadKeypoints(f, kps);
            database->ReadImagePairFlow(f, t, flow);
            const size_t n = flow.src_kps_indices.size();
            if (flow.tgt_kps.size() != n) continue;

            const RowMajorMatrix4f& view_f = get_view(f);
            const SceneTransformations scene_f = {
                .model_matrix = model0,
                .view_matrix  = view_f,
                .intrinsics   = intr,
            };

            for (size_t i = 0; i < n; ++i) {
                const uint32_t kp_idx = flow.src_kps_indices[i];
                if (kp_idx >= kps.size()) continue;
                const Eigen::Vector2f& kp     = kps[kp_idx];
                const Eigen::Vector2f& tgt_kp = flow.tgt_kps[i];

                const std::optional<RayHit> hit = RayCast(*accel, scene_f, kp, true);
                if (!hit) continue;

                // object hit -> world (model0), then -> camera at the FIXED t pose.
                const Eigen::Vector3f Xw = model0_R * hit->pos + model0_t;
                const Eigen::Vector4f Xh(Xw.x(), Xw.y(), Xw.z(), 1.0f);
                const Eigen::Vector4f Pc = view_t * Xh;
                if (Pc.z() <= 1e-6f) continue;     // behind / on the CV camera plane

                IcpObs o;
                o.a = (double)Pc.x() / (double)Pc.z();
                o.b = (double)Pc.y() / (double)Pc.z();
                o.u = (double)tgt_kp.x();
                o.v = (double)tgt_kp.y();
                double we = 1.0;
                if (i < flow.flow_errors.size() && flow.flow_errors[i] > 1e-6f)
                    we = std::min(1.0 / (double)flow.flow_errors[i], 1e3);
                o.w = we;
                obs.push_back(o);
            }
        }
        }   // end else (flow-DB gather)

        if (obs.size() < kMinObs) { ++frames_skipped; continue; }

        // ------------------------------------------------------------------
        // IRLS weighted linear fit, R,t frozen.
        // ------------------------------------------------------------------
        if (do_focal) {
            // Hold cx,cy; fit a single fy (fx = aspect*fy). Residuals:
            //   r_u = (aspect*fy)*a + cx - u ,  r_v = fy*b + cy - v
            // Scalar normal equation:  fy = S_xy / S_xx  with
            //   S_xy = Σ w[ (aspect*a)(u-cx) + b(v-cy) ]
            //   S_xx = Σ w[ (aspect*a)^2 + b^2 ]
            double cx, cy; cur_principal_px(t, cx, cy);
            double fy_cur, fx_cur; cur_focal_px(t, fx_cur, fy_cur);
            double fy = fy_cur;

            for (int it = 0; it < kIters; ++it) {
                double Sxy = 0.0, Sxx = 0.0;
                for (const IcpObs& o : obs) {
                    const double A   = aspect * o.a;
                    const double ru  = A * fy + cx - o.u;
                    const double rv  = o.b * fy + cy - o.v;
                    const double rmag = std::sqrt(ru*ru + rv*rv);
                    const double hw  = (rmag <= kHuberPx) ? 1.0 : (kHuberPx / rmag);
                    const double wgt = o.w * hw;
                    Sxy += wgt * (A * (o.u - cx) + o.b * (o.v - cy));
                    Sxx += wgt * (A * A + o.b * o.b);
                }
                if (Sxx <= 1e-12) break;
                fy = Sxy / Sxx;
                fy = std::clamp(fy, (double)bounds.f_low, (double)bounds.f_high);
            }

            const double fx     = aspect * fy;
            const double mm     = fx * haperture / (double)w;
            key_focal_at((double)t, mm);

            double srms = 0.0;
            for (const IcpObs& o : obs) {
                const double ru = aspect*fy*o.a + cx - o.u;
                const double rv = fy*o.b + cy - o.v;
                srms += ru*ru + rv*rv;
            }
            sum_rms += std::sqrt(srms / (double)obs.size());
        } else { // do_principal
            // Hold fx,fy; fit cx,cy. Each is an independent weighted mean:
            //   cx = Σ w(u - fx*a)/Σ w ,  cy = Σ w(v - fy*b)/Σ w
            double fx, fy; cur_focal_px(t, fx, fy);
            double cx, cy; cur_principal_px(t, cx, cy);

            for (int it = 0; it < kIters; ++it) {
                double Su = 0.0, Sv = 0.0, Sw = 0.0;
                for (const IcpObs& o : obs) {
                    const double ru = fx*o.a + cx - o.u;
                    const double rv = fy*o.b + cy - o.v;
                    const double rmag = std::sqrt(ru*ru + rv*rv);
                    const double hw  = (rmag <= kHuberPx) ? 1.0 : (kHuberPx / rmag);
                    const double wgt = o.w * hw;
                    Su += wgt * (o.u - fx*o.a);
                    Sv += wgt * (o.v - fy*o.b);
                    Sw += wgt;
                }
                if (Sw <= 1e-12) break;
                cx = std::clamp(Su / Sw, (double)bounds.cx_low, (double)bounds.cx_high);
                cy = std::clamp(Sv / Sw, (double)bounds.cy_low, (double)bounds.cy_high);
            }

            key_principal_at((double)t, cx, cy);

            double srms = 0.0;
            for (const IcpObs& o : obs) {
                const double ru = fx*o.a + cx - o.u;
                const double rv = fy*o.b + cy - o.v;
                srms += ru*ru + rv*rv;
            }
            sum_rms += std::sqrt(srms / (double)obs.size());
        }
        ++frames_solved;
    }

    if (frames_solved == 0) {
        oss << "  [FAIL] no frame had enough correspondences (need \xE2\x89\xA5 "
            << kMinObs << "). Nothing was written; the pose is untouched.";
        set_status(oss.str());
        return false;
    }

    oss << "  [PASS] " << (do_focal ? "Solved Focal" : "Solved Cx/Cy")
        << " baked over " << first_frame_ << ".." << last_frame_ << ".\n"
        << "    frames solved   = " << frames_solved
        << " (skipped " << frames_skipped << ", too few features)\n"
        << "    mean reproj RMS = " << (sum_rms / std::max(1, frames_solved)) << " px\n"
        << "  Pose curves were NOT modified. Tick 'Export Solved "
        << (do_focal ? "Focal" : "Principal") << "' to bake it into the Camera2 on Export.";
    set_status(oss.str());

    // The solved_* curves are display knobs; nudge a redraw so the readout and
    // any expression-linked camera refresh immediately.
    asapUpdate();
    return true;
}


void PolychaseTracker::on_solve_focal()
{
    solve_intrinsics_impl(/*do_focal=*/true, /*do_principal=*/false);
}

void PolychaseTracker::on_solve_principal()
{
    solve_intrinsics_impl(/*do_focal=*/false, /*do_principal=*/true);
}


// =============================================================================
// on_solve_zoom — TRUE varying-focal solve by block-coordinate alternation
//                 (ZOOM_SOLVE_IMPLEMENTATION.md, Option 1).
//
// WHY (and why it is a SEPARATE button from Solve Focal):
//   Focal and dolly/depth are coupled — a zoom-in looks like a dolly-in. A pose
//   solved with FIXED focal (Track) has already absorbed any real zoom into
//   translation, so a fixed-pose focal fit alone (Solve Focal) just hands the
//   flat focal back. To pull a genuine zoom out you must let pose AND focal move
//   together. Rather than re-couple the upstream solver (which would re-solve and
//   overwrite the whole hand-tracked pose), we converge them by ALTERNATING two
//   passes that already exist and are trusted:
//
//     (1) fixed-pose focal fit  — solve_intrinsics_impl(do_focal=true): R,t held
//         at the kept pose, fit fy per frame, bake to the solved_focal curve.
//     (2) fixed-focal pose re-solve — on_refine(): re-solve the interior poses of
//         the Refine Range with the just-solved per-frame focal SEEDED FIXED
//         (on_refine's have_focal_curve path). Range ends + anchors held.
//
//   Repeat until the largest per-frame focal change between two passes drops
//   below zoom_tol_mm_, or zoom_passes_ passes have run. Each focal fit re-fits
//   focal under the latest pose; each refine re-fits pose under the latest focal.
//
// DESTRUCTIVE to pose: step (2) re-bakes the Refine Range's interior frames. That
//   is the whole point (a faithful zoom needs the pose to move), so this lives on
//   its own opt-in button and never on the default Solve Focal button. All pose
//   writes happen INSIDE on_refine via the guarded key_pose_matrix_at (which
//   refuses non-finite poses and rejects diverged ones), so the non-finite-key
//   invariant is satisfied here for free — we add NO raw pose writes.
//
// SHARED STATE: the solved_focal curve is the hand-off between the two passes;
//   we snapshot it (knob("solved_focal")->get_value_at) before each pass to
//   measure convergence. Nothing else is needed — solve_intrinsics_impl and
//   on_refine own their own inputs, status, and progress dialogs.
// =============================================================================
void PolychaseTracker::on_solve_zoom()
{
    std::ostringstream oss;
    oss << "[" << timestamp() << "] Solve Zoom (alternating focal/pose, Option 1)\n";

    // Precondition: a tracked pose to start from. The first focal fit freezes it;
    // the refine steps then move it under the solved focal. Without a pose there
    // is nothing to alternate against (solve_intrinsics_impl would fail anyway,
    // but fail early here with a clearer message).
    if (!has_pose_keys()) {
        oss << "  [FAIL] No tracked pose. Track (and ideally Refine) a constant-\n"
            << "  focal pass first; Solve Zoom then pulls the lens variation out of\n"
            << "  it by alternating a focal fit with a pose re-solve.";
        set_status(oss.str());
        return;
    }
    if (last_frame_ <= first_frame_) {
        oss << "  [FAIL] Last Frame must be greater than First Frame.";
        set_status(oss.str());
        return;
    }

    // Sample the current solved_focal curve over the solve span so we can measure
    // the per-pass change. Span matches solve_intrinsics_impl (first..last).
    auto snapshot_focal = [&](std::vector<double>& out) {
        const int n = std::max(0, last_frame_ - first_frame_ + 1);
        out.assign((size_t)n, 0.0);
        if (Knob* sf = knob("solved_focal"))
            for (int t = first_frame_; t <= last_frame_; ++t)
                out[(size_t)(t - first_frame_)] = sf->get_value_at((double)t);
    };

    const int passes = std::max(1, zoom_passes_);
    std::vector<double> prev, cur;
    double last_max_delta = 0.0;
    int    passes_run     = 0;
    bool   converged      = false;

    for (int p = 0; p < passes; ++p) {
        snapshot_focal(prev);

        // (1) fixed-pose focal fit. Reuses the proven IRLS solve; writes the
        //     solved_focal curve via key_focal_at. If it fails (no pose / too few
        //     features / bad inputs) it has already set a descriptive status, so
        //     stop and leave the pose at the last good pass.
        if (!solve_intrinsics_impl(/*do_focal=*/true, /*do_principal=*/false)) {
            oss << "  [STOP] focal fit failed on pass " << (p + 1) << ".\n"
                << "  Pose left at the last completed pass; see the focal-solve\n"
                << "  status above for the cause.";
            set_status(oss.str());
            return;
        }

        // (2) fixed-focal pose re-solve under the just-solved per-frame focal.
        //     The mechanism depends on the tracking mode:
        //
        //     - FLOW mode (default): on_refine() over the Refine Range. Because a
        //       solved_focal curve now exists (have_focal_curve) it seeds each
        //       frame's intrinsics from it and holds them FIXED while bundling the
        //       interior poses, baking through the guarded key_pose_matrix_at.
        //
        //     - USER-TRACK-ONLY mode (track_only_user_): there is NO flow DB, so
        //       on_refine cannot run (it requires db_path_). The pose re-solve is
        //       instead the pin/track PnP (track_via_pins), with the solved focal
        //       SEEDED so the alternation actually couples — the PnP holds the
        //       lens fixed and solves pose under it. We cover the whole span from
        //       the reference frame in both directions (ref->last and ref->first);
        //       track_via_pins bakes through the same guarded key_pose_matrix_at.
        if (track_only_user_) {
            const bool ok_fwd = track_via_pins(/*forward=*/true,  /*seed_solved_intrinsics=*/true);
            const bool ok_bwd = track_via_pins(/*forward=*/false, /*seed_solved_intrinsics=*/true);
            if (!ok_fwd && !ok_bwd) {
                oss << "  [STOP] pose re-solve via pins failed on pass " << (p + 1)
                    << ".\n  Need >=3 pins with a key or track link. Connect pins to\n"
                    << "  tracks (or key a few corners) and re-run; focal from this\n"
                    << "  pass is kept, pose left at the last good pass.";
                set_status(oss.str());
                return;
            }
        } else {
            on_refine();
        }

        // Convergence: largest per-frame focal change introduced this pass.
        snapshot_focal(cur);
        double max_d = 0.0;
        const size_t n = std::min(cur.size(), prev.size());
        for (size_t i = 0; i < n; ++i)
            max_d = std::max(max_d, std::abs(cur[i] - prev[i]));

        last_max_delta = max_d;
        ++passes_run;

        // Require at least one full alternation before the early-out so a curve
        // that starts near-converged still gets one focal+pose coupling.
        if (p > 0 && max_d < zoom_tol_mm_) {
            converged = true;
            break;
        }
    }

    oss << "  [PASS] Zoom solve ran " << passes_run << " pass(es); "
        << (converged ? "converged" : "stopped at pass limit")
        << " (max |\xCE\x94focal| last pass = " << last_max_delta << " mm, tol "
        << zoom_tol_mm_ << ").\n"
        << "  Pose inside the Refine Range was re-baked; range ends + anchors held.\n"
        << "  Tick 'Export Solved Focal' to bake the per-frame focal onto the Camera2.\n"
        << "  Validate: reproj RMS should DROP and inlier ratio stay high (>= 0.7);\n"
        << "  if the focal pins to a Min/Max FOV bound the shot is too low-parallax\n"
        << "  for a trustworthy zoom — tighten the bounds or add anchors.";
    set_status(oss.str());

    // The solved_focal curve + the re-baked pose are display state; nudge a redraw.
    asapUpdate();
}


// =============================================================================
// points_are_coplanar — best-fit-plane flatness test for the zoom guard.
//
// A varying-focal solve needs pins spread across the object's DEPTH: four points
// on a single flat face cannot separate "longer lens" from "farther away" (the
// focal/depth ambiguity). We refuse a coplanar pin layout rather than return a
// confident-but-meaningless focal.
//
// Method: build the 3x3 covariance of the points about their centroid and take
// its eigenvalues (ascending). The smallest eigenvalue is the variance along the
// best-fit-plane normal — i.e. how far the points bulge OUT of their own plane.
// Compare its RMS (sqrt) to the overall point-set scale (sqrt of the largest
// eigenvalue): if the out-of-plane spread is below `tol_frac` of the in-plane
// spread, the set is effectively flat. Scale-relative, so it works for a unit
// cube or a 10-metre set alike. < 4 points is treated as coplanar (degenerate).
// =============================================================================
bool PolychaseTracker::points_are_coplanar(const std::vector<Eigen::Vector3f>& pts,
                                           float tol_frac)
{
    if (pts.size() < 4) return true;

    Eigen::Vector3f c = Eigen::Vector3f::Zero();
    for (const auto& p : pts) c += p;
    c /= (float)pts.size();

    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (const auto& p : pts) {
        const Eigen::Vector3f d = p - c;
        cov += d * d.transpose();
    }
    cov /= (float)pts.size();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
    if (es.info() != Eigen::Success) return true;  // be conservative: refuse
    const Eigen::Vector3f ev = es.eigenvalues();    // ascending: ev[0] <= ev[1] <= ev[2]

    const float out_of_plane = std::sqrt(std::max(0.0f, ev[0]));   // normal-direction RMS
    const float in_plane     = std::sqrt(std::max(0.0f, ev[2]));   // largest in-plane RMS
    if (in_plane < 1e-8f) return true;                              // all points coincident

    return (out_of_plane / in_plane) < tol_frac;
}


// =============================================================================
// on_solve_zoom_from_pins — Zoom tab entry point (ZOOM_TAB_PLAN.md, step 2a).
//
// Validates the pin set for a TRUSTWORTHY varying-focal solve, then delegates to
// the proven on_solve_zoom() (the alternating focal/pose solver that converged to
// the ground-truth 16->26mm on the test cube). The guards are REFUSALS with a
// clear status, never silent clamps:
//   (1) >= 2 refine anchors  — need at least two zoom frames to interpolate focal.
//   (2) >= 4 pins            — minimum to pin pose+focal at a frame.
//   (3) pins NON-COPLANAR    — must straddle the object's depth (front+back), or
//                              the focal/depth ambiguity makes the result garbage.
//
// Step 2a reads the EXISTING pin list (the same pins you track with). 2b adds a
// separate zoom-pin store; 2c the armed Zoom Pin Edit overlay + ZOOM badge. The
// solve math underneath is unchanged and already verified, so this step only adds
// the safety rails and the tab button.
// =============================================================================
void PolychaseTracker::on_solve_zoom_from_pins()
{
    std::ostringstream oss;
    oss << "[" << timestamp() << "] Solve Zoom (Zoom Pins)\n";

    // ---- guard (0): pin-only mode required (no flow database) ----
    // The zoom-pin solve drives the focal fit from pins / user tracks, which has
    // NO optical-flow database. solve_intrinsics_impl()'s flow branch demands a
    // db_path_ and fails with "Database path not set" otherwise. Rather than
    // silently flip a mode the user didn't choose (it changes how Track behaves),
    // refuse with the one-tick instruction. 'Use Only User Tracks' ON is the
    // pin/zoom-pin workflow this tab is built for.
    if (!track_only_user_) {
        oss << "  [REFUSE] Turn ON 'Use Only User Tracks' first.\n"
            << "  The zoom-pin solve runs from pins (no optical-flow database). With\n"
            << "  that box unticked the focal fit takes the flow path and fails with\n"
            << "  'Database path not set'. Tick it (main tab) and press this again.";
        set_status(oss.str());
        return;
    }

    // ---- guard (1): enough anchors to interpolate a focal curve ----
    const int n_anchors = (int)refine_anchors_.size();
    if (n_anchors < 2) {
        oss << "  [REFUSE] Need at least 2 anchor frames to solve a zoom (have "
            << n_anchors << ").\n"
            << "  Align the wireframe at a few frames across the zoom (e.g. first,\n"
            << "  middle, last) and 'Set Pose Key' / 'Add Anchor' at each, so the\n"
            << "  focal can be interpolated between known frames.";
        set_status(oss.str());
        return;
    }

    // ---- guard (2): enough pins ----
    const std::vector<Pin>& pv = pins();
    if ((int)pv.size() < 4) {
        oss << "  [REFUSE] Need at least 4 pins to solve focal (have "
            << (int)pv.size() << ").\n"
            << "  Place pins on the object's corners, spread across its depth.";
        set_status(oss.str());
        return;
    }

    // ---- guard (3): pins must be non-coplanar (depth spread) ----
    // Pull each pin's LOCAL-space vertex position from the current mesh and test
    // the set's flatness. Coplanar => the zoom can't be separated from distance.
    Op* geo = input_geo_op();
    if (geo) {
        GeoMesh gm;
        geo->validate(true);
        if (extract_mesh(geo, gm) && gm.local_vertices.rows() > 0) {
            const long nv = (long)gm.local_vertices.rows();
            std::vector<Eigen::Vector3f> pin_pts;
            pin_pts.reserve(pv.size());
            for (const Pin& p : pv) {
                if ((long)p.vertex_idx < nv) {
                    pin_pts.emplace_back(gm.local_vertices(p.vertex_idx, 0),
                                         gm.local_vertices(p.vertex_idx, 1),
                                         gm.local_vertices(p.vertex_idx, 2));
                }
            }
            if ((int)pin_pts.size() >= 4 && points_are_coplanar(pin_pts)) {
                oss << "  [REFUSE] The pinned vertices are coplanar (all on one\n"
                    << "  face / flat layout). A zoom needs pins across the object's\n"
                    << "  DEPTH — add corners on a back face so foreshortening can\n"
                    << "  tell a longer lens from a farther object. (This is the\n"
                    << "  focal/depth ambiguity, not a solver limit.)";
                set_status(oss.str());
                return;
            }
        }
        // If the mesh couldn't be read we don't block on coplanarity — the solve
        // itself still has the FOV-bound safety net and will report if it pins.
    }

    // ---- all guards passed: run the proven alternating focal/pose solve ----
    // on_solve_zoom() writes its own [PASS]/[FAIL] status (focal curve, pass count,
    // convergence, the low-parallax warning), so we let it own the final message.
    on_solve_zoom();
}


// =============================================================================
// on_flatten_focal — replace the solved_focal curve with its mean over
//                    First..Last (ZOOM_SOLVE_IMPLEMENTATION.md, Option 3).
//
// NON-DESTRUCTIVE: only the solved_focal curve is touched; pose is never read or
// written, so none of the pose invariants apply. This is the honest output for a
// CONSTANT lens — Solve Focal returns a near-flat-but-jittery curve, and this
// collapses it to the single best value. Non-finite / non-positive samples are
// skipped from the mean (they were skipped frames). We key every frame in the
// span so the exported curve is dense and unambiguous, matching how the solve
// writes it. The curve has no overlay-culling risk (unlike pose), so a direct
// set_value_at loop is fine here — we still go through the animated knob.
// =============================================================================
void PolychaseTracker::on_flatten_focal()
{
    std::ostringstream oss;
    oss << "[" << timestamp() << "] Flatten Solved Focal (mean)\n";

    Knob* k = knob("solved_focal");
    if (!k || !k->is_animated()) {
        oss << "  [FAIL] No solved focal curve. Run 'Solve Focal Length' (or "
               "'Solve Zoom') first.";
        set_status(oss.str());
        return;
    }
    if (last_frame_ <= first_frame_) {
        oss << "  [FAIL] Last Frame must be greater than First Frame.";
        set_status(oss.str());
        return;
    }

    double sum = 0.0;
    int    n   = 0;
    for (int t = first_frame_; t <= last_frame_; ++t) {
        const double v = k->get_value_at((double)t);
        if (std::isfinite(v) && v > 1e-6) { sum += v; ++n; }
    }
    if (n == 0) {
        oss << "  [FAIL] curve has no valid samples in " << first_frame_
            << ".." << last_frame_ << ".";
        set_status(oss.str());
        return;
    }

    const double avg = sum / (double)n;
    for (int t = first_frame_; t <= last_frame_; ++t)
        k->set_value_at(avg, (double)t);
    k->changed();

    oss << "  [PASS] Solved focal flattened to " << avg << " mm over "
        << first_frame_ << ".." << last_frame_ << " (" << n << " valid sample(s)).\n"
        << "  Pose untouched. Tick 'Export Solved Focal' to bake it.";
    set_status(oss.str());
    asapUpdate();
}


// =============================================================================
// on_smooth_focal — centred moving-average of the solved_focal curve over a
//                   focal_smooth_window_-frame window (Option 3).
//
// NON-DESTRUCTIVE (curve only). For a SLOW drift this tames per-frame jitter
// while keeping the trend, where Flatten would wrongly erase the real change.
// We SNAPSHOT the whole curve first, then write the smoothed values back, so the
// averaging reads the original samples (no in-place feedback). The window is
// clamped to odd and >= 1; non-finite / non-positive samples are skipped from
// each window's average (so isolated dropped frames don't poison their
// neighbours), and frames whose window is entirely invalid are left as-is.
// =============================================================================
void PolychaseTracker::on_smooth_focal()
{
    std::ostringstream oss;
    oss << "[" << timestamp() << "] Smooth Solved Focal (moving average)\n";

    Knob* k = knob("solved_focal");
    if (!k || !k->is_animated()) {
        oss << "  [FAIL] No solved focal curve. Run 'Solve Focal Length' (or "
               "'Solve Zoom') first.";
        set_status(oss.str());
        return;
    }
    if (last_frame_ <= first_frame_) {
        oss << "  [FAIL] Last Frame must be greater than First Frame.";
        set_status(oss.str());
        return;
    }

    // Window: force odd and >= 1 so it is centred (half on each side).
    int win = std::max(1, focal_smooth_window_);
    if ((win % 2) == 0) ++win;
    const int half = win / 2;

    const int span = last_frame_ - first_frame_ + 1;

    // Snapshot the original samples (value + validity) so the smoothing reads the
    // pre-smoothing curve, not partially-written values.
    std::vector<double> val((size_t)span, 0.0);
    std::vector<char>   ok((size_t)span, 0);
    int valid = 0;
    for (int t = first_frame_; t <= last_frame_; ++t) {
        const double v = k->get_value_at((double)t);
        const bool good = std::isfinite(v) && v > 1e-6;
        val[(size_t)(t - first_frame_)] = v;
        ok[(size_t)(t - first_frame_)]  = good ? 1 : 0;
        if (good) ++valid;
    }
    if (valid == 0) {
        oss << "  [FAIL] curve has no valid samples in " << first_frame_
            << ".." << last_frame_ << ".";
        set_status(oss.str());
        return;
    }

    // Compute smoothed values into a separate buffer.
    std::vector<double> out((size_t)span, 0.0);
    std::vector<char>   wrote((size_t)span, 0);
    for (int i = 0; i < span; ++i) {
        double s = 0.0; int c = 0;
        for (int d = -half; d <= half; ++d) {
            const int j = i + d;
            if (j < 0 || j >= span) continue;
            if (!ok[(size_t)j]) continue;
            s += val[(size_t)j]; ++c;
        }
        if (c > 0) { out[(size_t)i] = s / (double)c; wrote[(size_t)i] = 1; }
    }

    // Write the smoothed samples back (leave entirely-invalid windows untouched).
    int changed = 0;
    for (int i = 0; i < span; ++i) {
        if (!wrote[(size_t)i]) continue;
        k->set_value_at(out[(size_t)i], (double)(first_frame_ + i));
        ++changed;
    }
    k->changed();

    oss << "  [PASS] Solved focal smoothed (window " << win << " frames) over "
        << first_frame_ << ".." << last_frame_ << "; " << changed
        << " frame(s) updated.\n"
        << "  Pose untouched. Re-run with a larger window for more smoothing, or "
           "use 'Flatten' for a constant lens.";
    set_status(oss.str());
    asapUpdate();
}

} // namespace pcn