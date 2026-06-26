// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Peter Mercell
//
// Developed with assistance from Claude (Anthropic).

// =============================================================================
// tracker_refine.cpp — Refine Sequence for the PolychaseTracker plugin.
//
// Port of Polychase's global bundle adjustment between keyframes
// (cpp/refiner.cc :: RefineTrajectory) into the Nuke NDK plugin. See
// REFINE_PLAN.md for the full derivation. Implements:
//
//   PolychaseTracker::on_refine()             — button entry ("Refine Range").
//       Re-solves the frames BETWEEN anchors inside the "1001-1100" Range knob
//       against the same optical-flow DB Track used, holding the range ends and
//       any interior anchors fixed as ground truth.
//   PolychaseTracker::build_segments_in_range(from,to) — {from,to} ∪ interior
//       anchors -> consecutive [A,B] pairs with an interior frame to solve.
//   The anchor list (add/clear/load/save/label) — the explicit replacement for
//       Blender's KEYFRAME vs GENERATED keyframe types.
//
// CONVENTION BRIDGE — Track's, inverted (see tracker_track.cpp for the forward
// direction). The DB stores OpenCV / Y-down / real-pixel keypoints; Track baked
//     model'(t) = view_cv_seed^-1 * view_cv(t) * model0
// with view_cv_seed = flip * imatrix(cam @ seed), model0 = pose curve @ seed.
// We have the (hand-corrected) object pose curve and need the per-frame CV
// camera pose the solver optimises, so invert it:
//     view_cv(t) = view_cv_seed * model'(t) * model0^-1
// then after the solve bake the refined interior frames back with the SAME
// transform pair Track uses (model'(t) = view_cv_seed^-1 * view_cv'(t) * model0).
// Anchors A and B are skipped on the bake, so the artist's corrections stay
// exact, and because view_cv_seed / model0 are sampled FROM the curve at A,
// view_cv(A) is automatically the frozen ground truth (no drift at the seam).
//
// SYNCHRONOUS, like Track: RefineTrajectory is a C++17-clean free function in
// libpolychase.a (the C++20 RefinerThread is not used), so it drops into the
// exact slot Track lives in — synchronous call, TrackProgress dialog, Cancel via
// tracking_cancel_. The UI blocks until done (same limitation as Track).
// =============================================================================
#include "polychase_tracker.h"

// Shared OpenGL<->OpenCV convention bridge (one definition for Track/Refine/
// user-track anchoring).
#include "pcn_convention.h"

// Upstream tracker types reused by the bridge (CameraTrajectory, CameraState,
// CameraIntrinsics, AcceleratedMesh, Pose). tracker.h is already known to
// compile inside this C++17 plugin (tracker_track.cpp includes it).
#include "tracker.h"

// The refiner free function + its option/update structs. C++17-clean header;
// the C++20 internals (std::jthread / std::atomic_ref) live in the .cc compiled
// into libpolychase.a, exactly like TrackSequence. Lives under POLYCHASE_ROOT/cpp,
// which is already a (SYSTEM) include dir, and adds no new link deps.
#include "refiner.h"

// Qt-free progress-dialog facade (no-op when headless / built without Qt).
#include "track_progress.h"

#include <Eigen/LU>          // RowMajorMatrix4f::inverse()
#include <Eigen/Geometry>    // Eigen::Quaternionf

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using namespace DD::Image;

namespace pcn {

// -----------------------------------------------------------------------------
// File-local helpers.
//
// The convention bridge (gl_to_cv_flip / tracker_intrinsics /
// build_local_accel_mesh) now lives in the shared pcn_convention.h and is
// re-exported just below; cam_imatrix_at and view_cv_to_pose remain local to
// Refine (the latter is Refine-only).
// -----------------------------------------------------------------------------

// The OpenGL<->OpenCV convention bridge (gl_to_cv_flip / tracker_intrinsics /
// build_local_accel_mesh) is shared with Track + user-track anchoring via
// pcn_convention.h, so Refine uses the EXACT same bridge by construction — the
// former hand-copied duplicates (and their ODR-clash caveat) are gone. Re-export
// under the original names so every call site below is unchanged.
using conv::gl_to_cv_flip;
using conv::tracker_intrinsics;
using conv::build_local_accel_mesh;

// world->camera (imatrix) of a CameraOp sampled at a specific frame.
static DD::Image::Matrix4 cam_imatrix_at(DD::Image::CameraOp* cam, double frame)
{
    OutputContext orig = cam->outputContext();
    OutputContext c    = orig;
    c.setFrame(frame);
    cam->setOutputContext(c);
    cam->validate(true);
    const DD::Image::Matrix4 m = cam->imatrix();   // world-to-camera
    cam->setOutputContext(orig);
    cam->validate(true);
    return m;
}

// Extract a clean rigid Pose (unit-quaternion rotation + translation) from a
// model-view CV matrix. Track bakes pure rotation+translation, but the gizmo can
// write a non-unit pose_scale; the refiner solves a RIGID camera trajectory, so
// we strip any scale from the rotation block (column-normalize) before forming
// the quaternion (see REFINE_PLAN.md §7 — "normalize the rotation block").
// Translation is taken straight from the matrix (scale-independent). For the
// common scale==1 case this is exact and the convention round-trips byte-for-byte.
static void view_cv_to_pose(const RowMajorMatrix4f& view_cv, Pose& pose)
{
    Eigen::Matrix3f R = view_cv.block<3, 3>(0, 0);
    for (int c = 0; c < 3; ++c) {
        const float n = R.col(c).norm();
        if (n > 1e-12f) R.col(c) /= n;
    }
    pose.q = Eigen::Quaternionf(R);
    pose.q.normalize();
    pose.t = view_cv.block<3, 1>(0, 3);
}


// =============================================================================
// Anchor list — the explicit boundary set (Nuke knob keyframes carry no
// KEYFRAME/GENERATED type, so we maintain our own). Persisted in the hidden,
// saved "refine_anchors" String_knob; mirrored in refine_anchors_, which is
// AUTHORITATIVE for the session once loaded/edited. Mirrors the pins_blob_
// round-trip (in-memory truth + content-diff-guarded reload for undo/.nk load).
// =============================================================================

void PolychaseTracker::load_anchors_from_knob()
{
    refine_anchors_.clear();
    const std::string text(refine_anchors_blob_ ? refine_anchors_blob_ : "");

    // Parse signed integers; any non-[0-9-] character separates (so "1,40,75",
    // "1 40 75" and "1, 40, 75" all parse the same). Frames can be negative.
    std::string tok;
    auto flush = [&]() {
        if (tok.empty() || tok == "-") { tok.clear(); return; }
        try { refine_anchors_.insert(std::stoi(tok)); } catch (...) {}
        tok.clear();
    };
    for (char c : text) {
        if ((c >= '0' && c <= '9') || (c == '-' && tok.empty())) tok += c;
        else flush();
    }
    flush();

    refine_anchors_cache_  = text;
    refine_anchors_loaded_ = true;
}

void PolychaseTracker::save_anchors_to_knob()
{
    std::ostringstream o;
    bool first = true;
    for (int f : refine_anchors_) { if (!first) o << ','; o << f; first = false; }
    const std::string s = o.str();

    // Ignore a no-op rewrite (and avoid re-firing our own echo handler).
    if (s != refine_anchors_cache_) {
        refine_anchors_cache_ = s;
        if (Knob* k = knob("refine_anchors")) {
            ScopedFlags guard(suppress_anchor_callback_);
            k->set_text(s.c_str());
        }
    }
    refresh_anchor_label();
}

void PolychaseTracker::refresh_anchor_label()
{
    std::ostringstream o;
    if (refine_anchors_.empty()) {
        o << "(none — only First/Last act as boundaries)";
    } else {
        bool first = true;
        for (int f : refine_anchors_) { if (!first) o << ", "; o << f; first = false; }
    }
    if (Knob* k = knob("refine_anchor_list")) k->set_text(o.str().c_str());
}

bool PolychaseTracker::add_refine_anchor(int frame)
{
    // Union with whatever is persisted in the knob, WITHOUT clearing the in-memory
    // set. This is what makes repeated adds accumulate reliably: even if the loaded
    // flag was stale (which previously let a reload wipe the set down to one frame),
    // we only ever add here. The std::set dedupes, so re-adding a frame is a no-op.
    {
        const std::string text(refine_anchors_blob_ ? refine_anchors_blob_ : "");
        std::string tok;
        auto flush = [&]() {
            if (tok.empty() || tok == "-") { tok.clear(); return; }
            try { refine_anchors_.insert(std::stoi(tok)); } catch (...) {}
            tok.clear();
        };
        for (char c : text) {
            if ((c >= '0' && c <= '9') || (c == '-' && tok.empty())) tok += c;
            else flush();
        }
        flush();
        refine_anchors_loaded_ = true;
    }

    const bool added = refine_anchors_.insert(frame).second;
    if (added) {
        save_anchors_to_knob();
        PCN_LOG("[refine] anchor recorded @ frame " << frame
                << "  (now " << refine_anchors_.size() << " anchor(s))\n");
    }
    return added;
}

// Wipe + persist with NO status write / repaint — used mid-flow by Track and
// Clear Pose Keys, which own the status line.
void PolychaseTracker::reset_refine_anchors_silent()
{
    refine_anchors_.clear();
    refine_anchors_loaded_ = true;
    save_anchors_to_knob();
}

// Button entry: wipe + status + repaint (for starting the corrections over).
void PolychaseTracker::clear_refine_anchors()
{
    reset_refine_anchors_silent();
    set_status("[" + timestamp() + "] Refine anchors cleared.\n"
               "  Only First/Last Frame now bound Refine. Re-correct frames "
               "(pin/gizmo + Set Pose Key) to record new anchors.");
    asapUpdate();
}


// =============================================================================
// parse_refine_range — read the "1001-1100"-style Range knob into [from,to].
// Any non-digit separates the two numbers, so "1001-1100", "1001 1100" and
// "1001,1100" all parse. Empty/garbage falls back to [first_frame_, last_frame_]
// and that fallback is written back so the field shows what will be refined.
// Returns false only if even the fallback is degenerate (from >= to).
// =============================================================================
bool PolychaseTracker::parse_refine_range(int& from, int& to)
{
    std::vector<int> nums;
    int cur = 0; bool in = false;
    for (const char* p = refine_range_ ? refine_range_ : ""; *p; ++p) {
        if (*p >= '0' && *p <= '9') { cur = cur * 10 + (*p - '0'); in = true; }
        else if (in) { nums.push_back(cur); cur = 0; in = false; }
    }
    if (in) nums.push_back(cur);

    if (nums.size() >= 2) { from = nums[0]; to = nums[1]; }
    else                  { from = first_frame_; to = last_frame_; }   // fallback

    if (from > to) std::swap(from, to);

    // Echo the resolved range back so an empty/garbage field shows what ran.
    if (Knob* k = knob("refine_range")) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%d-%d", from, to);
        if (!refine_range_ || std::string(refine_range_) != buf) k->set_text(buf);
    }
    return to > from;
}


// =============================================================================
// Segment construction — boundary set = {from, to} ∪ anchors strictly inside
// (from, to); consecutive pairs form [A,B]; keep only those with an interior
// frame (>2). The range ends act as held anchors exactly like first/last used to.
// =============================================================================
std::vector<std::pair<int, int>> PolychaseTracker::build_segments_in_range(int from, int to) const
{
    std::set<int> b;
    b.insert(from);
    b.insert(to);
    for (int k : refine_anchors_)
        if (k > from && k < to) b.insert(k);
    std::vector<int> ks(b.begin(), b.end());   // std::set => sorted ascending

    std::vector<std::pair<int, int>> segs;
    for (size_t i = 0; i + 1 < ks.size(); ++i)
        segs.push_back({ks[i], ks[i + 1]});

    // Drop segments with no interior frame to solve (RefineTrajectory also
    // CHECKs Count() > 2; the full fill loop below guarantees every frame in
    // [A,B] is filled, so this >2 filter is the only gate needed).
    segs.erase(std::remove_if(segs.begin(), segs.end(),
               [](const std::pair<int, int>& s) { return s.second - s.first + 1 <= 2; }),
               segs.end());
    return segs;
}


// =============================================================================
// on_refine — refine every gap inside the "1001-1100" Range knob, filling each
// segment's trajectory from the (hand-corrected) pose curve, running
// RefineTrajectory, baking the refined interior frames back. Closely follows
// on_track's structure + validation.
// =============================================================================
void PolychaseTracker::on_refine()
{
    int range_from = first_frame_, range_to = last_frame_;
    const bool range_ok = parse_refine_range(range_from, range_to);

    std::ostringstream oss;
    oss << "[" << timestamp() << "] Refine Range "
        << range_from << "-" << range_to << "\n";

    // ---- input + state validation (same guards as on_track) ----
    Iop*      img = input_img();
    CameraOp* cam = input_cam();
    Op*       geo = input_geo_op();
    if (!img) { oss << "  [FAIL] input 0 (img) not connected.";  set_status(oss.str()); return; }
    if (!cam) { oss << "  [FAIL] input 1 (cam) not connected.";  set_status(oss.str()); return; }
    if (!geo) { oss << "  [FAIL] input 2 (geo) not connected.";  set_status(oss.str()); return; }
    if (!db_path_ || db_path_[0] == '\0') {
        oss << "  [FAIL] Database path not set — point it at the .db built by mvflow_to_db.";
        set_status(oss.str()); return;
    }
    if (!has_pose_keys()) {
        oss << "  [FAIL] No tracked pose to refine. Track first, then hand-correct a\n"
            << "  few frames (pin/gizmo + Set Pose Key) and Refine the gaps between them.";
        set_status(oss.str()); return;
    }
    if (!range_ok) {
        oss << "  [FAIL] Range is empty — type a frame range like 1001-1100.";
        set_status(oss.str()); return;
    }

    // ---- segments inside the range (ends + interior anchors held fixed) ----
    if (!refine_anchors_loaded_) load_anchors_from_knob();
    auto segs = build_segments_in_range(range_from, range_to);
    if (segs.empty()) {
        oss << "  [FAIL] No refinable gap inside " << range_from << "-" << range_to << ".\n"
            << "  Hand-correct (pin/gizmo + Set Pose Key) at least one interior frame so\n"
            << "  there's a gap to solve; each correction becomes an anchor automatically.";
        set_status(oss.str()); return;
    }

    // ---- dimensions ----
    int w = 0, h = 0;
    try {
        img->validate(true);
        const Format& fmt = img->info().format();
        w = fmt.width();
        h = fmt.height();
    } catch (const std::exception& e) {
        oss << "  [FAIL] could not validate input 0: " << e.what();  set_status(oss.str()); return;
    }
    if (w <= 0 || h <= 0) {
        oss << "  [FAIL] input 0 reports invalid dimensions " << w << "x" << h;  set_status(oss.str()); return;
    }

    // ---- mesh (local space) ----
    GeoMesh gm;
    geo->validate(true);
    if (!extract_mesh(geo, gm)) {
        oss << "  [FAIL] could not extract a mesh from the geo input.";  set_status(oss.str()); return;
    }
    if (gm.local_vertices.rows() < 3) {
        oss << "  [FAIL] mesh has < 3 vertices.";  set_status(oss.str()); return;
    }
    std::shared_ptr<AcceleratedMesh> accel =
        build_local_accel_mesh(gm, build_mask_array((uint32_t)gm.triangles.rows()));

    // ---- shared bridge constants (intrinsics + the GL->CV flip) ----
    cam->validate(true);
    const RowMajorMatrix4f flip = gl_to_cv_flip();
    const CameraIntrinsics intr = tracker_intrinsics(cam->projection(), (float)w, (float)h);

    // Intrinsics seeding (NOT solving). Refine re-solves POSE between anchors; it
    // no longer varies the lens — focal/principal are owned by the background Solve
    // buttons (tracker_intrinsics.cpp). But if those buttons already solved a focal
    // and/or principal, we SEED each frame's intrinsics from those curves and hold
    // them FIXED through the bundle, so the pose refinement stays consistent with
    // your solved lens instead of snapping back to the camera's base intrinsics.
    // haperture maps the solver's pixel focal <-> Nuke mm. Both probes are sampled
    // ONCE, before any bake.
    double haperture = 24.576;
    if (Knob* hk = cam->knob("haperture")) haperture = hk->get_value();
    const bool have_focal_curve     = has_solved_focal();
    const bool have_principal_curve = has_solved_principal();

    {
        std::ostringstream pre;
        pre << oss.str()
            << "  segments    = " << segs.size() << "\n"
            << "  db          = " << db_path_ << "\n"
            << "  mesh        = " << gm.local_vertices.rows() << " verts, "
                                  << gm.triangles.rows() << " tris\n"
            << "  Synchronous — the UI blocks until done. Per-segment cost prints\n"
            << "  to the terminal Nuke was launched from (PCN_DEBUG).";
        set_status(pre.str());
    }

    TrackProgress prog("Polychase \xE2\x80\x94 refining");
    tracking_cancel_ = false;

    int   segs_done    = 0;
    int   frames_baked = 0;
    int   frames_rejected = 0;   // diverged poses kept at their prior value
    bool  any_error    = false;
    std::string emsg;

    // 2D occlusion mask: built ONCE over the whole refine range and shared by every
    // segment (copying the std::function just shares the underlying bitset stack).
    // Empty (mask off / no mask input) => identical to before.
    const MaskPredicate mask2d_pred = make_mask2d_predicate(range_from, range_to);

    // User (helper) tracks. Opt-in via "Use User Tracks": when on, these are the
    // connected tracks at EXACT pinned vertices; off => Refine runs on flow +
    // anchors only. Per segment we pass ONLY the observations strictly inside the
    // segment (A,B) — never on the frozen anchor frames A/B themselves.
    const UserTracks user_tracks_all = refine_use_user_tracks_ ? build_solve_tracks()
                                                               : UserTracks{};
    const float      user_tracks_w   = 2.0f;   // fixed user-track pull in Refine

    for (size_t si = 0; si < segs.size(); ++si) {
        const int A = segs[si].first;
        const int B = segs[si].second;
        const int n = B - A + 1;

        // --- segment basis (seed = A), same convention as Track ---
        const RowMajorMatrix4f view_cv_seed     = flip * nuke_to_eigen_m4(cam_imatrix_at(cam, (double)A));
        const RowMajorMatrix4f view_cv_seed_inv = view_cv_seed.inverse();
        const RowMajorMatrix4f model0           = nuke_to_eigen_m4(pose_matrix_to_nuke((double)A));
        const RowMajorMatrix4f model0_inv       = model0.inverse();

        // --- fill the trajectory from the (hand-corrected) pose curve ---
        // view_cv(A) == view_cv_seed and view_cv(B) is the corrected anchor —
        // exactly the two frames RefineTrajectory freezes as ground truth.
        CameraTrajectory traj(A, (size_t)n);
        for (int t = A; t <= B; ++t) {
            const RowMajorMatrix4f model_t = nuke_to_eigen_m4(pose_matrix_to_nuke((double)t));
            const RowMajorMatrix4f view_cv = view_cv_seed * model_t * model0_inv;
            CameraState cs;
            // Seed intrinsics from any already-solved curves and hold them fixed
            // (otherwise Refine's pose bundle would run against the camera's base
            // lens, ignoring what you solved). fx folds in aspect, so fy = fx/aspect.
            CameraIntrinsics cs_intr = intr;
            if (have_focal_curve) {
                if (Knob* sf = knob("solved_focal")) {
                    const double focal_mm = sf->get_value_at((double)t);
                    const double fx_px = (w > 0) ? focal_mm * (double)w / haperture
                                                 : (double)cs_intr.fx;
                    const double aspect = (cs_intr.aspect_ratio != 0.0f)
                                              ? (double)cs_intr.aspect_ratio : 1.0;
                    cs_intr.fx = (float)fx_px;
                    cs_intr.fy = (float)(fx_px / aspect);
                }
            }
            // Likewise seed a solved principal point (OpenCV/y-down px), so a refine
            // after Solve Principal Point respects the shift instead of recentring.
            if (have_principal_curve) {
                if (Knob* kx = knob("solved_cx")) cs_intr.cx = (float)kx->get_value_at((double)t);
                if (Knob* ky = knob("solved_cy")) cs_intr.cy = (float)ky->get_value_at((double)t);
            }
            cs.intrinsics = cs_intr;
            view_cv_to_pose(view_cv, cs.pose);
            traj.Set(t, cs);
        }

        // --- options (mirror Blender defaults) ---
        // NOTE: these field/enum names come straight from REFINE_PLAN.md's
        // reading of cpp/refiner.h + the bundle options. If the build trips
        // here, this block is the API-contract surface to reconcile against the
        // actual headers (most likely the loss-type enumerator spelling).
        RefinerOptions opts;
        opts.bundle_opts.loss_type     = BundleOptions::LossType::CAUCHY;  // Blender uses Cauchy
        opts.bundle_opts.loss_scale    = 1.0;
        // Refine NEVER varies intrinsics now — focal/principal are solved by the
        // dedicated background buttons (tracker_intrinsics.cpp). Holding both off
        // means the seeded lens above is treated as fixed ground truth while only
        // the interior POSES are bundled. FOV bounds are forwarded but unused.
        opts.optimize_focal_length     = false;
        opts.optimize_principal_point  = false;
        opts.min_fov_deg               = min_fov_deg_;
        opts.max_fov_deg               = max_fov_deg_;
        opts.is_masked                 = mask2d_pred;   // 2D occlusion mask (shared)

        // Restrict each track's observations to this segment's INTERIOR (A,B); the
        // frozen anchors A/B must not be pulled. >=2 interior obs to be worth it.
        UserTracks seg_tracks;
        if (refine_use_user_tracks_) {
            for (const UserTrack& ut : user_tracks_all) {
                UserTrack t;
                t.object_point = ut.object_point;
                t.weight       = ut.weight;
                for (const UserTrackObservation& o : ut.observations)
                    if (o.frame_id > A && o.frame_id < B)
                        t.observations.push_back(o);
                if (t.observations.size() >= 2) seg_tracks.push_back(std::move(t));
            }
        }
        opts.user_tracks               = seg_tracks;     // interior-only helpers
        opts.user_track_weight         = user_tracks_w;

        auto cb = [&](RefinerUpdate up) -> bool {
            const int pct = (int)((100.0 * ((double)si + (double)up.progress)) / (double)segs.size());
            prog.set_percent(pct);
            if (prog.cancelled()) tracking_cancel_ = true;
            PCN_LOG("[refine] seg " << A << ".." << B << "  " << up.message << "\n");
            return !tracking_cancel_;
        };

        try {
            RefineTrajectory(std::string(db_path_), traj, model0, *accel, cb, opts);
        } catch (const std::exception& e) {
            any_error = true; emsg = e.what();
            break;
        } catch (...) {
            any_error = true; emsg = "(unknown exception type)";
            break;
        }

        // --- bake refined interior frames back (exclude anchors A, B) ---
        for (int t = A + 1; t <= B - 1; ++t) {
            const RowMajorMatrix4f view_cv_p = traj.Get(t)->pose.Rt4x4();   // refined
            const RowMajorMatrix4f model_p   = view_cv_seed_inv * view_cv_p * model0;

            // Divergence guard: a bundle that blew up yields non-finite, absurdly
            // translated, or non-rigid (column norms far from 1) poses, which then
            // distort the wireframe. Don't key those — leave the frame's prior pose
            // and report it, so a bad solve can't corrupt the whole curve.
            const Eigen::Vector3f tr = model_p.block<3, 1>(0, 3);
            const float c0 = model_p.block<3, 1>(0, 0).norm();
            const float c1 = model_p.block<3, 1>(0, 1).norm();
            const float c2 = model_p.block<3, 1>(0, 2).norm();
            const bool bad =
                !model_p.allFinite() || tr.norm() > 1.0e3f ||
                c0 < 0.1f || c0 > 10.0f || c1 < 0.1f || c1 > 10.0f ||
                c2 < 0.1f || c2 > 10.0f;
            if (bad) {
                PCN_LOG("[refine] frame " << t << " REJECTED diverged pose: |t|="
                        << tr.norm() << " scale=(" << c0 << "," << c1 << "," << c2
                        << ")\n");
                ++frames_rejected;
                continue;   // keep the existing keyed pose at this frame
            }

            key_pose_matrix_at((double)t, model_p);
            ++frames_baked;
        }

        // No intrinsic write-back: Refine holds the lens fixed (seeded above) and
        // only re-solves pose, so solved_focal / solved_cx / solved_cy are left
        // exactly as the background Solve buttons baked them — unchanged across the
        // refine, with no gaps to fill since those curves already span First..Last.
        ++segs_done;

        PCN_LOG("[refine] segment " << A << ".." << B << " done ("
                << (n - 2) << " interior frame(s))\n");
        if (tracking_cancel_) break;
    }

    // Refresh the linked camera and drop any stale preview so the overlay reads
    // the freshly refined curve (same tail as Track).
    if (frames_baked > 0) {
        bake_live_camera();
        live_scene_.reset();
        rot_base_.reset();
        rot_base_had_live_ = false;
        trans_base_.reset();
        trans_base_had_live_ = false;
        sync_blob_from_live_pose();
    }

    std::ostringstream done;
    if (any_error) {
        done << "[" << timestamp() << "] Refine FAILED\n"
             << "  " << emsg << "\n"
             << "  segments completed before failure: " << segs_done
             << " (" << frames_baked << " frame(s) baked)\n"
             << "  Causes mirror Track: the segment frames must be inside the DB's\n"
             << "  range and the proxy must overlap textured, rigid pixels.";
    } else {
        done << "[" << timestamp() << "] Refine "
             << (tracking_cancel_ ? "cancelled" : "complete") << "\n"
             << "  [PASS] " << segs_done << " segment(s), " << frames_baked
             << " interior frame(s) re-solved; anchors held fixed.\n";
        if (frames_rejected > 0)
            done << "  [WARN] " << frames_rejected << " frame(s) DIVERGED and were "
                    "left at their prior pose — the bundle blew up there. Lower Track "
                    "Weight (or turn Use User Tracks off) and re-run; see the "
                    "[refine] REJECTED lines in the terminal.\n";
        done << "  Scrub to confirm the lock, then Export as before.";
    }
    set_status(done.str());
    asapUpdate();
}

} // namespace pcn
