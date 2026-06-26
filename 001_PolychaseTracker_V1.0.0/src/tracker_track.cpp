// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Peter Mercell
//
// Developed with assistance from Claude (Anthropic).

// =============================================================================
// tracker_track.cpp — Track + Apply for the PolychaseTracker plugin.
//
// Implements:
//   PolychaseTracker::on_track(direction)  — wire up the upstream optical-flow
//       tracker (TrackSequence) synchronously and bake the solved per-frame
//       object pose onto this node's pose_translate/rotate/scale curves, so the
//       viewer wireframe locks to the feature as you scrub.
//   PolychaseTracker::on_stop()            — request cancel (see note below).
//   PolychaseTracker::on_export()          — reinterpret the baked pose curves
//       per Solve Mode (Camera | Model) and emit a ready-to-run Python script
//       that spawns + keys a Camera2 / TransformGeo.
//   PolychaseTracker::key_pose_matrix_at() — per-frame T/R/S key writer.
//
// WHY synchronous: the upstream TrackerThread is C++20 (jthread/stop_token) and
// can't compile in our C++17 plugin, but tracker.h's free function TrackSequence
// is C++17-clean and takes a std::function callback. We call it directly, the
// same way the converter builds the flow DB. A synchronous run blocks
// the UI thread (so the Stop button can't be clicked mid-run — same limitation
// the converter has); per-frame progress is mirrored to stdout.
//
// CONVENTION BRIDGE (the load-bearing part):
//   The optical-flow database stores REAL-PIXEL, Y-DOWN (top-origin) keypoints,
//   because the mvflow_to_db converter writes them in that convention. The
//   interactive overlay / pin-solve,
//   by contrast, work in Y-UP image pixels with OpenGL-convention intrinsics
//   (build_intrinsics_from_projection). So the tracker SEED must be expressed in
//   OpenCV convention + Y-down to match the DB:
//     - intrinsics: positive fx/fy (same magnitude the overlay uses — the
//       fmt_w*0.5 scale that the pin-solve was empirically tuned to), principal
//       point at (w/2, h/2), convention = OpenCV.
//     - view: flip camera-space Y and Z (diag(1,-1,-1,1)) applied to Nuke's
//       world->camera imatrix, turning "looks -Z, Y-up" into "looks +Z, Y-down".
//   model_matrix stays in Nuke world space (convention-independent).
//   The per-frame inlier_ratio is the live correctness check: a healthy track
//   sits high (≳0.7); near-zero across the board means the convention is wrong
//   (first thing to flip if it misbehaves — see notes at tracker_intrinsics()).
// =============================================================================
#include "polychase_tracker.h"

// Shared OpenGL<->OpenCV convention bridge (gl_to_cv_flip / tracker_intrinsics /
// build_local_accel_mesh) — ONE definition, previously hand-copied here and into
// tracker_refine.cpp / tracker_usertracks.cpp.
#include "pcn_convention.h"

// Upstream tracker (C++17-clean free functions). tracker.h transitively pulls
// camera_trajectory.h, database.h, geometry.h, ray_casting.h, pnp/solvers.h.
#include "tracker.h"

// Qt-free progress-dialog facade (all Qt confined to track_progress.cpp). A
// no-op when headless or when built without Qt, so the call sites below need no
// #ifdef. Drives a TrackIt progress bar + Cancel off the per-frame callback.
#include "track_progress.h"


#include <Eigen/LU>          // RowMajorMatrix4f::inverse()
#include <Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <map>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

using namespace DD::Image;

namespace pcn {

// -----------------------------------------------------------------------------
// File-local helpers.
// -----------------------------------------------------------------------------

// The OpenGL<->OpenCV convention bridge is shared (pcn_convention.h) so Track,
// Refine and user-track anchoring can never drift apart. Re-exported into this
// file's scope under the original names, so every call site below is unchanged.
//
// IF A TRACK PRODUCES NEAR-ZERO INLIERS EVERYWHERE, the convention bridge is the
// first suspect: drop the gl_to_cv_flip() on the view and set convention =
// OpenGL (negative fx/fy) — the DB might be Y-up in your build. A *partial*
// inlier ratio (~0.5) that won't improve is NOT focal/aspect — it's outlier-heavy
// correspondences (ambiguous/repetitive texture or noisy flow); inspect the DB.
using conv::gl_to_cv_flip;
using conv::tracker_intrinsics;
using conv::build_local_accel_mesh;

// camera->world matrix of a CameraOp sampled at a specific frame. Swaps the
// camera's OutputContext to `frame`, samples, and restores on the way out.
static DD::Image::Matrix4 cam_world_at(DD::Image::CameraOp* cam, double frame)
{
    OutputContext orig = cam->outputContext();
    OutputContext c    = orig;
    c.setFrame(frame);
    cam->setOutputContext(c);
    cam->validate(true);
    const DD::Image::Matrix4 m = cam->matrix();   // camera-to-world
    cam->setOutputContext(orig);
    cam->validate(true);
    return m;
}

// world->camera (imatrix) of a CameraOp sampled at a specific frame. Mirrors
// cam_world_at, which returns camera->world. Export reconstructs the SEED-frame
// view (the basis Track baked against), which may differ from the frame the
// button is pressed on — so we must sample at an explicit frame, not "now".
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

// Python float literal with enough precision for matrix-derived values.
static std::string pyf(double v)
{
    std::ostringstream o;
    o << std::setprecision(10) << v;
    return o.str();
}


// -----------------------------------------------------------------------------
// key_pose_matrix_at — decompose an object-world matrix to T/R/S and write a
// keyframe on each pose channel at `frame`. set_animated creates the curve on
// first use; set_value_at overwrites an existing key at the same frame.
//
// Refuses a non-finite pose (returns false, writes nothing). The interactive
// commit paths (key_pose_at_current_frame, run_pin_solve, resolve_pins_no_mover)
// all reject NaN/Inf before it reaches a key; the per-frame Track bake must do
// the same, or one divergent PnP frame poisons the curve — every later frame
// then reads that NaN back through effective_model_matrix and culls the overlay.
// -----------------------------------------------------------------------------
bool PolychaseTracker::key_pose_matrix_at(double frame,
                                          const RowMajorMatrix4f& model_world)
{
    if (!model_world.allFinite()) {
        PCN_LOG("[track] refused to key a non-finite pose @ frame " << frame
                << " (skipped; curve left intact)\n");
        return false;
    }

    double t[3], r[3], s[3];
    decompose_trs(model_world, t, r, s);

    auto key_xyz = [&](const char* name, const double v[3]) {
        DD::Image::Knob* k = knob(name);
        if (!k) return;
        for (int i = 0; i < 3; ++i) {
            if (!k->is_animated(i)) k->set_animated(i);
            k->set_value_at(v[i], frame, i);
        }
        k->changed();
    };
    key_xyz("pose_translate", t);
    key_xyz("pose_rotate",    r);
    key_xyz("pose_scale",     s);
    return true;
}


// -----------------------------------------------------------------------------
// key_focal_at — write one keyframe (mm) on the animated "solved_focal" knob.
// set_animated creates the curve on first use; set_value_at overwrites an
// existing key at the same frame. Single-value knob, so index 0.
// -----------------------------------------------------------------------------
void PolychaseTracker::key_focal_at(double frame, double focal_mm)
{
    DD::Image::Knob* k = knob("solved_focal");
    if (!k) return;
    if (!k->is_animated()) k->set_animated();
    k->set_value_at(focal_mm, frame);
    k->changed();
}


// True when the solved-focal curve carries animation, i.e. a focal length was
// actually solved (vs. the fixed default). Export and Refine seeding key off this.
bool PolychaseTracker::has_solved_focal() const
{
    DD::Image::Knob* k = knob("solved_focal");
    return k && k->is_animated();
}


// -----------------------------------------------------------------------------
// on_copy_focal_to_camera — bake the solved_focal curve straight onto the
// CONNECTED input camera's 'focal' knob (animated) over [first_frame_, last_frame_].
//
// This is the "I know the lens / I solved it, now make the camera OWN it" button:
// afterwards cam->knob("focal") carries the zoom, so the overlay (and Export, and
// any downstream render through that camera) reads the right lens directly and you
// can untick Preview Solved Lens. UNLIKE Export, which spawns a fresh Camera2 and
// never touches the input, this MUTATES your camera node in place — the writes are
// ordinary keyframe sets, so Ctrl+Z reverts them.
//
// Focal only: the solved mm already assume the camera's haperture (fx_px =
// focal_mm * w / haperture), so haperture/principal point are left untouched.
//
// NOTE (intentionally NOT silently worked around): cam->projection() still returns
// a STALE focal under a forced OutputContext even after this (that is a cooked-
// projection quirk, see conv::apply_curve_focal), so the DB *flow* solve in
// track_via_pins — which uses fixed seed intrinsics — does not magically start
// reading this per frame. The overlay/export read the focal CURVE (get_value_at),
// so they DO reflect it. To track a zoom under this lens, keep using the per-frame
// focal path ('Track Under Solved Focal'), which reads the curve rather than the
// cooked projection.
// -----------------------------------------------------------------------------
void PolychaseTracker::on_copy_focal_to_camera()
{
    std::ostringstream oss;
    oss << "[" << timestamp() << "] Copy Solved Focal \xE2\x86\x92 Camera\n";

    DD::Image::CameraOp* cam = input_cam();
    if (!cam) {
        oss << "  [FAIL] No camera connected to the 'cam' input.";
        set_status(oss.str());
        return;
    }
    if (!has_solved_focal()) {
        oss << "  [FAIL] No 'Solved Focal' curve. Run a focal solve (e.g. 'Solve "
               "Focal (P4Pf)') first, then copy.";
        set_status(oss.str());
        return;
    }

    DD::Image::Knob* sf = knob("solved_focal");
    DD::Image::Knob* cf = cam->knob("focal");
    if (!sf || !cf) {
        oss << "  [FAIL] Missing focal knob (solved_focal=" << (sf ? "ok" : "null")
            << ", camera focal=" << (cf ? "ok" : "null") << ").";
        set_status(oss.str());
        return;
    }

    int f0 = first_frame_, f1 = last_frame_;
    if (f1 < f0) {
        oss << "  [FAIL] Last Frame (" << f1 << ") < First Frame (" << f0 << ").";
        set_status(oss.str());
        return;
    }

    // Make the camera's focal animated (no-op if it already is; this preserves any
    // existing keys outside the range), then overwrite one key per frame from the
    // solved curve. Frames where the solved value is non-finite / non-positive are
    // left as-is rather than poisoning the camera with a garbage key.
    if (!cf->is_animated()) cf->set_animated();

    int written = 0, skipped = 0;
    for (int f = f0; f <= f1; ++f) {
        const double mm = sf->get_value_at((double)f);
        if (!std::isfinite(mm) || mm <= 1e-6) { ++skipped; continue; }
        cf->set_value_at(mm, (double)f);
        ++written;
    }
    cf->changed();

    if (written == 0) {
        oss << "  [FAIL] The solved focal had no finite value in " << f0 << ".."
            << f1 << " — nothing copied.";
        set_status(oss.str());
        return;
    }

    oss << "  [PASS] Wrote " << written << " focal key(s) onto '" << cam->node_name()
        << "' over " << f0 << ".." << f1 << " (skipped " << skipped << ").\n"
        << "  The camera now carries the zoom — you can untick 'Preview Solved Lens'.\n"
        << "  (Undo reverts the keys. haperture was left unchanged.)";
    set_status(oss.str());
    asapUpdate();
}


// -----------------------------------------------------------------------------
// key_principal_at — write one keyframe of the solved principal point to the
// hidden animated "solved_cx"/"solved_cy" curves. cx,cy are in the solver's
// OpenCV / y-down / real-pixel convention (what traj/pnp intrinsics carry).
// Export converts them to the Camera2's win_translate.
// -----------------------------------------------------------------------------
void PolychaseTracker::key_principal_at(double frame, double cx_px, double cy_px)
{
    if (DD::Image::Knob* kx = knob("solved_cx")) {
        if (!kx->is_animated()) kx->set_animated();
        kx->set_value_at(cx_px, frame);
        kx->changed();
    }
    if (DD::Image::Knob* ky = knob("solved_cy")) {
        if (!ky->is_animated()) ky->set_animated();
        ky->set_value_at(cy_px, frame);
        ky->changed();
    }
}


// True when a principal point was actually solved (curves carry animation).
// Export keys off this to decide whether to write win_translate.
bool PolychaseTracker::has_solved_principal() const
{
    DD::Image::Knob* kx = knob("solved_cx");
    DD::Image::Knob* ky = knob("solved_cy");
    return kx && ky && kx->is_animated() && ky->is_animated();
}


// -----------------------------------------------------------------------------
// on_track — seed from the keyed pose at the seed frame and solve the range
// First->Last (forward) or playhead->First (backward) as one continuous pass.
// Runs TrackSequence and bakes the solved per-frame poses onto the pose curves.
// -----------------------------------------------------------------------------
void PolychaseTracker::on_track(const char* direction)
{
    const bool forward = (direction && std::strcmp(direction, "forward") == 0);

    // "Use Only User Tracks": bypass the optical-flow DB entirely and solve an
    // exact per-frame PnP from the pin<->track bindings (see track_via_pins). It
    // reads the camera's per-frame focal CURVE, so a zoom baked onto the camera with
    // 'Copy Solved Focal -> Camera' is honoured automatically — no extra toggle.
    // (seed_solved_intrinsics=false here; that override path is reserved for the
    // focal solve's own internal re-track, which needs the solved focal before it
    // has been copied to the camera.)
    if (track_only_user_) {
        track_via_pins(forward, /*seed_solved_intrinsics=*/false);
        return;
    }

    std::ostringstream oss;
    oss << "[" << timestamp() << "] Track " << direction << "\n";

    // ---- input + state validation ----
    Iop*      img = input_img();
    CameraOp* cam = input_cam();
    Op*       geo = input_geo_op();
    if (!img) { oss << "  [FAIL] input 0 (img) not connected.";               set_status(oss.str()); return; }
    if (!cam) { oss << "  [FAIL] input 1 (cam) not connected.";               set_status(oss.str()); return; }
    if (!geo) { oss << "  [FAIL] input 2 (geo) not connected.";               set_status(oss.str()); return; }
    if (!db_path_ || db_path_[0] == '\0') {
        oss << "  [FAIL] Database path not set — point it at the .db built by mvflow_to_db.";          set_status(oss.str()); return;
    }
    if (!has_pose_keys()) {
        oss << "  [FAIL] No seed pose. Go to the FIRST frame, place pins and\n"
            << "  'Set Pose Key' there — TrackIt always solves from the first frame.";
                                                                               set_status(oss.str()); return;
    }

    // Forward TrackIt always solves the whole range from the FIRST frame,
    // regardless of where the playhead sits, so every forward press gives the
    // same consistent result. BACKWARD instead seeds from the frame the playhead
    // is on (where you keyed the pose) and solves DOWN to First Frame — the
    // "anchor at 100, track back to 1" workflow.
    const int cur_frame  = editing_frame();
    const int seed_frame = forward ? first_frame_ : cur_frame;

    // ---- range endpoint ----
    // TrackIt is one continuous span: seed frame -> end frame.
    const int end_frame = forward ? last_frame_ : first_frame_;

    if (forward && end_frame <= seed_frame) {
        oss << "  [FAIL] Last Frame (" << last_frame_ << ") must be greater than "
            << "the current frame (" << seed_frame << ").";                   set_status(oss.str()); return;
    }
    if (!forward && end_frame >= seed_frame) {
        oss << "  [FAIL] First Frame (" << first_frame_ << ") must be less than "
            << "the current frame (" << seed_frame << ").";                   set_status(oss.str()); return;
    }

    // ---- dimensions ----
    int w = 0, h = 0;
    try {
        img->validate(true);
        const Format& fmt = img->info().format();
        w = fmt.width();
        h = fmt.height();
    } catch (const std::exception& e) {
        oss << "  [FAIL] could not validate input 0: " << e.what();           set_status(oss.str()); return;
    }
    if (w <= 0 || h <= 0) {
        oss << "  [FAIL] input 0 reports invalid dimensions " << w << "x" << h; set_status(oss.str()); return;
    }

    // ---- mesh (local space) ----
    GeoMesh gm;
    geo->validate(true);
    if (!extract_mesh(geo, gm)) {
        oss << "  [FAIL] could not extract a mesh from the geo input.";        set_status(oss.str()); return;
    }
    if (gm.local_vertices.rows() < 3) {
        oss << "  [FAIL] mesh has < 3 vertices.";                              set_status(oss.str()); return;
    }
    std::shared_ptr<AcceleratedMesh> accel =
        build_local_accel_mesh(gm, build_mask_array((uint32_t)gm.triangles.rows()));

    // ---- shared bridge constants (intrinsics + the GL->CV flip) ----
    cam->validate(true);
    const RowMajorMatrix4f flip = gl_to_cv_flip();

    // Horizontal aperture (mm) for the solved-focal pixel->mm map
    // (focal_mm = fx_px * haperture / w). Read once; aperture is frame-constant.
    double haperture = 24.576;
    if (Knob* hk = cam->knob("haperture")) haperture = hk->get_value();

    // Seed intrinsics. cam->projection() carries a STALE focal for an animated
    // lens (see pcn_convention.h::apply_curve_focal / ZOOM_TAB_PLAN.md), so the
    // flow-track seed would otherwise be a frozen panel-frame fx/fy. Read the
    // focal CURVE at the seed frame and rewrite the projection's diagonal terms.
    // No-op for a static lens.
    DD::Image::Matrix4 seed_proj = cam->projection();
    {
        double seed_focal_mm = 0.0;
        if (Knob* fk = cam->knob("focal")) seed_focal_mm = fk->get_value_at((double)seed_frame);
        conv::apply_curve_focal(seed_proj, seed_focal_mm, haperture);
    }
    const CameraIntrinsics intr = tracker_intrinsics(seed_proj, (float)w, (float)h);

    // Focal pixel<->mm calibration (VARIABLE_FOCAL_PLAN §3/§6). Logged every
    // Track so you can confirm fx_px*haperture/w reproduces the camera's focal
    // knob to <0.1mm BEFORE trusting any baked focal curve — that constant is the
    // whole correctness gate. (Needs PCN_DEBUG / -DPOLYCHASE_DEBUG_LOG=ON.)
    {
        const double fx_px    = (double)intr.fx;
        const double focal_rt = (w > 0) ? fx_px * haperture / (double)w : 0.0;
        double cam_focal = 0.0;
        if (Knob* fk = cam->knob("focal")) cam_focal = fk->get_value();
        PCN_LOG("[focal] calib: fx_px=" << fx_px << " haperture=" << haperture
                << " w=" << w << " -> focal_mm=" << focal_rt
                << "  (cam focal knob=" << cam_focal
                << ", delta=" << (focal_rt - cam_focal) << ")\n");
    }

    // Intrinsics are solved by the dedicated background buttons (Solve Focal /
    // Solve Principal Point, tracker_intrinsics.cpp), which fit them against the
    // kept pose without re-tracking. Track therefore neither solves NOR clears the
    // solved_focal / solved_cx / solved_cy curves — leaving them intact so a
    // re-track doesn't wipe a focal/principal you already solved. (Export gates on
    // the 'Export Solved …' checkboxes + has_solved_*, independent of Track.)

    {
        std::ostringstream pre;
        pre << "[" << timestamp() << "] TrackIt starting\n"
            << "  seed frame  = " << seed_frame << "\n"
            << "  to frame    = " << end_frame << "\n";
        pre << "  db          = " << db_path_ << "\n"
            << "  mesh        = " << gm.local_vertices.rows() << " verts, "
                                  << gm.triangles.rows() << " tris\n"
            << "  Synchronous — the UI blocks until done. Per-frame inlier\n"
            << "  ratios print to the terminal Nuke was launched from.";
        set_status(pre.str());
    }
    std::cout.flush();

    // A fresh Track re-bakes the entire pose curve, so any anchors recorded from
    // a previous round of hand-corrections are now stale. Drop them silently (we
    // own the status line) so Refine starts from a clean slate — the artist
    // re-corrects a few frames after this Track to seed new anchors. Done here,
    // after all the [FAIL] guards, so a track that bails on a bad input leaves
    // the existing corrections untouched.
    reset_refine_anchors_silent();

    // ---- accumulators across all spans ----
    // Each span solves a CAMERA trajectory with the model fixed at that span's
    // seed pose, then re-expresses each solved camera as the equivalent OBJECT
    // pose:  model'(t) = view_cv_seed^-1 * view_cv(t) * model0  (Nuke world; the
    // Y/Z flip cancels because seed and t share it). The seed frame is never keyed.
    tracking_cancel_      = false;
    int   frames_keyed    = 0;
    int   frames_skipped  = 0;        // non-finite PnP frames refused (not keyed)
    float last_inlier     = -1.0f;
    float min_inlier      = 2.0f;
    int   low_inlier_runs = 0;
    // Motion span of the solved object pose, to tell a real track from a
    // near-static solve (which looks identical to "the overlay won't animate").
    double t_lo[3] = { 1e30,  1e30,  1e30};
    double t_hi[3] = {-1e30, -1e30, -1e30};
    double r_lo[3] = { 1e30,  1e30,  1e30};
    double r_hi[3] = {-1e30, -1e30, -1e30};
    RowMajorMatrix4f model0_first = RowMajorMatrix4f::Identity();   // scale ref for the motion check

    const auto t0 = std::chrono::steady_clock::now();
    bool        error = false;
    std::string emsg;

    // Progress dialog (RAII; closes when this scope ends). No-op in headless
    // Nuke or in a Qt-free build. Both track paths below drive it from their
    // TrackSequence callback. report_progress maps a frame within a span's
    // [a0,a1] to a 0..100 percent; the spans run a0->a1 forward or backward, so
    // we measure distance from a0 either way.
    TrackProgress prog(std::string("Polychase \xE2\x80\x94 tracking ") +
                       (forward ? "forward" : "backward"));
    auto report_progress = [&prog](int frame, int a0, int a1) {
        const int span = std::abs(a1 - a0);
        const int done = std::abs(frame - a0);
        prog.set_percent(span > 0 ? (int)((100.0 * done) / span) : 100);
    };

    // ---- single continuous span: seed_frame -> end_frame -------------------
    // Seed once from the keyed pose at the seed frame, solve a CAMERA trajectory
    // with the model fixed at that pose, then re-express each solved camera as the
    // equivalent OBJECT pose:  model'(t) = view_cv_seed^-1 * view_cv(t) * model0.
    // The bridge basis (camera view + model0) is sampled at the seed frame a0.
    {
        const int a0 = seed_frame;
        const int a1 = end_frame;

        const RowMajorMatrix4f view_cv_seed     = flip * nuke_to_eigen_m4(cam_imatrix_at(cam, (double)a0));
        const RowMajorMatrix4f view_cv_seed_inv = view_cv_seed.inverse();
        const RowMajorMatrix4f model0           = nuke_to_eigen_m4(pose_matrix_to_nuke((double)a0));
        model0_first = model0;

        SceneTransformations seed;
        seed.model_matrix = model0;
        seed.view_matrix  = view_cv_seed;
        seed.intrinsics   = intr;

        TrackerOptions opts;                       // defaults: Huber, max_inlier_error=12, fixed focal/principal
        opts.frame_from         = a0;
        opts.frame_to_inclusive = a1;
        // Track NEVER varies intrinsics now — focal/principal are solved by the
        // dedicated background buttons (tracker_intrinsics.cpp) so the pose Track
        // bakes here is never coupled to a lens re-solve. The FOV bounds are still
        // forwarded harmlessly (unused while both optimise flags are false).
        opts.pnp_opts.optimize_focal_length    = false;
        opts.pnp_opts.optimize_principal_point = false;
        opts.pnp_opts.min_fov_deg = min_fov_deg_;
        opts.pnp_opts.max_fov_deg = max_fov_deg_;

        // 2D occlusion mask: exclude masked image regions over the solved span.
        // Empty (mask off / no mask input) => identical to before. Sampling the
        // mask plate for every frame here is why the first Track press may pause
        // briefly when a mask is connected.
        opts.is_masked = make_mask2d_predicate(std::min(a0, a1), std::max(a0, a1));

        // User (helper) tracks: extra anchored 2D<->3D constraints. When Connect
        // has recorded bindings these are the connected tracks at EXACT pinned
        // vertices; otherwise the full anchored set. Empty => identical to before.
        opts.user_tracks       = build_solve_tracks();
        opts.user_track_weight = 8.0f;   // fixed pull (the Weight knob was removed)

        auto callback = [&](const TrackerUpdate& up) -> bool {
            const int   t   = up.frame;
            const float inl = up.pnp_result.inlier_ratio;

            // Progress + Cancel. Cancel sets the flag the returns honor.
            report_progress(t, a0, a1);
            if (prog.cancelled()) tracking_cancel_ = true;

            last_inlier = inl;
            if (inl < min_inlier) min_inlier = inl;
            if (inl < 0.5f) ++low_inlier_runs;

            const RowMajorMatrix4f view_cv_t   = up.pnp_result.camera.pose.Rt4x4();
            const RowMajorMatrix4f model_prime = view_cv_seed_inv * view_cv_t * model0;
            // Refuse a non-finite solve: skip the key AND the motion-span stats
            // (a NaN would corrupt t_lo/t_hi and the "static solve" heuristic).
            if (!key_pose_matrix_at((double)t, model_prime)) {
                ++frames_skipped;
                PCN_LOG("[PolychaseTracker]   frame " << t
                          << "  SKIPPED (non-finite pose; inlier="
                          << std::fixed << std::setprecision(3) << inl << ")\n");
                return !tracking_cancel_;
            }
            ++frames_keyed;

            // Intrinsics are NOT written here — Track only bakes pose. The focal /
            // principal curves are owned by the background Solve buttons
            // (tracker_intrinsics.cpp), which fit them against this baked pose.

            double tt[3], rr[3], ss[3];
            decompose_trs(model_prime, tt, rr, ss);
            for (int i = 0; i < 3; ++i) {
                t_lo[i] = std::min(t_lo[i], tt[i]); t_hi[i] = std::max(t_hi[i], tt[i]);
                r_lo[i] = std::min(r_lo[i], rr[i]); r_hi[i] = std::max(r_hi[i], rr[i]);
            }
            PCN_LOG("[PolychaseTracker]   frame " << t
                      << "  inlier=" << std::fixed << std::setprecision(3) << inl
                      << "  t=(" << std::setprecision(3) << tt[0] << "," << tt[1] << "," << tt[2] << ")"
                      << "  r=(" << rr[0] << "," << rr[1] << "," << rr[2] << ")"
                      << std::endl);
            return !tracking_cancel_;
        };

        try {
            TrackSequence(std::string(db_path_), seed, *accel, callback, opts);
        } catch (const std::exception& e) {
            error = true; emsg = e.what();
        } catch (...) {
            error = true; emsg = "(unknown exception type)";
        }
    }

    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count();

    // Persist the seed frame so Export can reconstruct model0 / cam-at-seed.
    track_seed_frame_ = seed_frame;
    if (Knob* k = knob("track_seed_frame")) k->set_value((double)seed_frame);

    // Refresh the Live Camera curves from the freshly-baked pose so any linked
    // Camera2 follows the new solve immediately (no manual Export needed).
    if (!error) bake_live_camera();

    // The track re-bakes the pose curve over the whole range. Drop any kept
    // working pose (live_scene_) so the overlay reads the freshly tracked curve
    // at every frame — a leftover preview from the seed-frame Set Pose would
    // otherwise mask the tracked result at that one frame.
    if (!error) {
        live_scene_.reset();
        rot_base_.reset();
        rot_base_had_live_ = false;
        trans_base_.reset();
        trans_base_had_live_ = false;
        sync_blob_from_live_pose();   // keep the undoable gizmo-pose mirror in step
    }

    std::ostringstream done;
    if (error) {
        done << "[" << timestamp() << "] Track FAILED\n"
             << "  " << emsg << "\n"
             << "  frames keyed before failure: " << frames_keyed << "\n"
             << "  elapsed: " << (ms / 1000.0) << "s\n"
             << "  (\"Not enough features\" usually means the seed pose is off,\n"
             << "   the database's frame range doesn't cover these frames, or the\n"
             << "   convention bridge needs flipping — see tracker_intrinsics.)";
    } else {
        const double tspan = std::max({t_hi[0]-t_lo[0], t_hi[1]-t_lo[1], t_hi[2]-t_lo[2]});
        const double rspan = std::max({r_hi[0]-r_lo[0], r_hi[1]-r_lo[1], r_hi[2]-r_lo[2]});
        // Relative translation motion: span vs. the seed's distance from origin.
        // A genuine track moves a few % of that; a degenerate/static solve moves
        // ~0.001%. An absolute threshold can't tell those apart across scenes.
        const double t_seed_mag = std::sqrt(
            (double)model0_first(0,3)*model0_first(0,3) +
            (double)model0_first(1,3)*model0_first(1,3) +
            (double)model0_first(2,3)*model0_first(2,3));
        const double t_rel = tspan / std::max(t_seed_mag, 1.0);
        done << "[" << timestamp() << "] Track " << (tracking_cancel_ ? "cancelled" : "complete") << "\n"
             << "  [PASS] " << frames_keyed << " frames keyed ("
             << seed_frame << (forward ? " -> " : " <- ") << end_frame << ")\n";
        if (frames_skipped)
            done << "  [WARN] " << frames_skipped << " frame(s) skipped (non-finite "
                    "solve) — those frames keep their previous pose; check seed / "
                    "convention if many.\n";
        done << "  inlier ratio: last=" << std::fixed << std::setprecision(3) << last_inlier
             << ", min=" << min_inlier;
        if (low_inlier_runs)
            done << "   [" << low_inlier_runs << " frame(s) < 0.5 — check seed / convention]";
        else
            done << "   [healthy]";
        done << "\n  pose motion span: translate=" << std::setprecision(4) << tspan
             << " (" << std::setprecision(3) << (t_rel * 100.0) << "% of scale)"
             << "  rotate=" << std::setprecision(4) << rspan << " deg\n";
        if (t_rel < 0.005 && rspan < 0.5) {
            done << "  [WARN] solved pose is essentially static. inlier=1.000 here just\n"
                 << "  means the few flow points that hit the mesh don't move. Causes:\n"
                 << "   - the proxy sits over low-texture / non-rigid / background pixels\n"
                 << "     (green screen, skin) instead of a rigid, textured, MOVING object;\n"
                 << "   - the proxy is too small on screen, so few features land on it.\n"
                 << "  Re-seed on a rigid feature and scale the proxy to cover more of it.\n";
        }
        done << "  elapsed: " << (ms / 1000.0) << "s\n"
             << "  Scrub the timeline — the wireframe should stay locked.\n"
             << "  Then pick Solve Mode and click Export.";
    }
    set_status(done.str());
    asapUpdate();
}


// -----------------------------------------------------------------------------
// on_stop — request cancel. See the synchronous-run caveat at the file header.
// -----------------------------------------------------------------------------
void PolychaseTracker::on_stop()
{
    tracking_cancel_ = true;
    std::ostringstream oss;
    oss << "[" << timestamp() << "] Stop requested.\n"
        << "  TrackIt runs synchronously in v0.2, so this only takes\n"
        << "  effect if work is still queued. Mid-run interruption needs the\n"
        << "  threaded path (deferred).";
    set_status(oss.str());
}


// -----------------------------------------------------------------------------
// on_export — reinterpret the baked pose curves per Solve Mode, bake them into a
// self-contained Python script, and run it in-process so the artist gets a
// native pick/create dialog (nuke.Panel) and the chosen Camera2 / TransformGeo
// is created + keyed in one click.
//
// Why via a script + Op::script_command: creating a DAG node + keying it is a
// Python/script-engine operation; NDK C++ can't do it directly. The full script
// is written next to the database (debugging / manual re-run) and executed with
// a tiny escaping-free exec(open(path).read()). If the write or exec fails, the
// script is echoed to the Status box as a fallback to run by hand.
//
// Math (everything in Nuke/GL world):
//   model0       = pose curve sampled at the seed frame (object->world)
//   model'(t)    = pose curve sampled at frame t          (object->world)
//   U            = geo rest matrix (object->world) read from the geo input
//   Model mode   -> a downstream TransformGeo concatenates with U, so we bake
//                   M_tg(t) = model'(t) * U^-1   (=> M_tg * U = model'(t)).
//   Camera mode  -> the geo is wired raw at rest U and the camera moves:
//                   cam_world(t) = U * model'(t)^-1 * cam_world_seed
//                   (with identity-rest geo, U = I and both reduce to the
//                    simple model'(t) / model0-based forms).
//
// We bake artist-editable translate/rotate/scaling keyframes (useMatrix off).
// The rotate is extracted with Nuke's OWN Matrix4::rotationsZXY() so the angles
// land in Nuke's convention and rot_order=ZXY rebuilds the exact rotation the
// overlay draws (our pose curves use compose_trs R=Rx*Ry*Rz, which matches no
// Nuke rot_order directly, so we re-decompose through Nuke's matrix class).
// -----------------------------------------------------------------------------
void PolychaseTracker::on_export()
{
    const bool  camera_mode = (solve_mode_ == 0);
    const char* mode_name   = camera_mode ? "Camera" : "Model";

    std::ostringstream oss;
    oss << "[" << timestamp() << "] Export (" << mode_name << " mode)\n";

    if (!has_pose_keys()) {
        oss << "  [FAIL] no solved poses to export. Track first.";
        set_status(oss.str());
        return;
    }
    if (camera_mode && !input_cam()) {
        oss << "  [FAIL] Camera mode needs input 1 (cam) connected to copy "
            << "intrinsics from.";
        set_status(oss.str());
        return;
    }

    const int seed_frame = (track_seed_frame_ != 0) ? track_seed_frame_ : first_frame_;
    const int f0 = first_frame_;
    const int f1 = last_frame_;
    if (f1 < f0) {
        oss << "  [FAIL] Last Frame (" << f1 << ") < First Frame (" << f0 << ").";
        set_status(oss.str());
        return;
    }

    const RowMajorMatrix4f model0 = nuke_to_eigen_m4(pose_matrix_to_nuke((double)seed_frame));

    // Geo rest matrix U. The overlay projects the geo's LOCAL points through the
    // pose P alone, REPLACING the geo's own object-to-world. We read U from the
    // geo input (frame-constant rest for a static proxy) and use it in BOTH modes:
    //   Model mode : a downstream TransformGeo CONCATENATES (world = M_tg * U * v),
    //                so we bake M_tg = P * U^-1  (=> M_tg * U = P, matches overlay).
    //   Camera mode: the geo is wired raw at rest U, so the moving camera must
    //                orbit U, not the solved seed pose model0:
    //                cam_world(t) = U * model'(t)^-1 * cam_world_seed.
    // With identity-rest geo U = I and both reduce to the simple forms.
    RowMajorMatrix4f U     = RowMajorMatrix4f::Identity();
    RowMajorMatrix4f U_inv = RowMajorMatrix4f::Identity();
    bool have_U = false;
    if (Op* geo = input_geo_op()) {
        geo->validate(true);
        GeoMesh gm;
        if (extract_mesh(geo, gm)) {
            U      = nuke_to_eigen_m4(gm.object_to_world);
            U_inv  = U.inverse();
            have_U = true;
        }
    }
    const RowMajorMatrix4f G = have_U ? U : model0;   // static-object world in Camera mode

    // Camera-mode constants.
    RowMajorMatrix4f cam_world_seed = RowMajorMatrix4f::Identity();
    double focal = 50.0, haperture = 24.576, vaperture = 18.672;
    double in_wt0 = 0.0, in_wt1 = 0.0;   // input camera's static win_translate
    if (camera_mode) {
        CameraOp* cam   = input_cam();
        cam_world_seed  = nuke_to_eigen_m4(cam_world_at(cam, (double)seed_frame));
        if (Knob* k = cam->knob("focal"))     focal     = k->get_value();
        if (Knob* k = cam->knob("haperture")) haperture = k->get_value();
        if (Knob* k = cam->knob("vaperture")) vaperture = k->get_value();
        if (Knob* k = cam->knob("win_translate")) {
            in_wt0 = k->get_value(0);
            in_wt1 = k->get_value(1);
        }
    }

    // Build per-frame translate/rotate/scale. The rotate is extracted with
    // Nuke's OWN Matrix4::rotationsZXY() so the angles are in Nuke's convention:
    // baking them with rot_order=ZXY rebuilds the exact rotation the overlay
    // draws. (Our pose curves use compose_trs R=Rx*Ry*Rz, which matches no Nuke
    // rot_order directly — so we re-decompose through Nuke's matrix class.)
    struct Key { int f; double t[3], r[3], s[3]; double focal; double wt[2]; };
    // Variable focal: emit an animated 'focal' on the Camera2 only when we're in
    // Camera mode, the toggle is ON, AND a focal was actually solved. Gating on the
    // live toggle (not just the curve) means unchecking 'Solve Focal Length' and
    // re-exporting reverts to the static focal even if a stale curve is still there.
    const bool var_focal = camera_mode && opt_focal_ && has_solved_focal();
    // Variable principal point -> win_translate. The solved cx,cy are in the
    // solve-time format's pixels, so pull that format from the image input; if
    // it's gone we can't normalise, so skip the principal bake rather than guess.
    float pp_w = 0.f, pp_h = 0.f;
    if (Iop* img = dynamic_cast<Iop*>(Op::input(0))) {
        img->validate(true);
        const Format& fmt = img->info().format();
        pp_w = (float)fmt.width();
        pp_h = (float)fmt.height();
    }
    // Same toggle-gating as focal: unchecking 'Solve Principal Point' reverts the
    // exported win_translate to the input camera's static shift on the next export.
    const bool var_principal =
        camera_mode && opt_principal_ && has_solved_principal() && pp_w > 0.f && pp_h > 0.f;
    std::vector<Key> keys;
    keys.reserve((size_t)(f1 - f0 + 1));
    for (int f = f0; f <= f1; ++f) {
        const RowMajorMatrix4f model_prime = nuke_to_eigen_m4(pose_matrix_to_nuke((double)f));
        RowMajorMatrix4f M = camera_mode
            ? RowMajorMatrix4f(G * model_prime.inverse() * cam_world_seed)      // camera-to-world (geo at rest G)
            : model_prime;                                                      // object-to-world (= pose P)
        // Model mode: cancel the geo's rest matrix so the baked TransformGeo,
        // which concatenates with U, reproduces the overlay (M_tg * U = P).
        if (!camera_mode && have_U)
            M = RowMajorMatrix4f(M * U_inv);
        Key k; k.f = f;
        // Translation = last column.
        for (int i = 0; i < 3; ++i) k.t[i] = (double)M((Eigen::Index)i, 3);
        // Scale = column norms of the upper-left 3x3; divide it out for a pure R.
        double inv[3];
        for (int c = 0; c < 3; ++c) {
            const double n = std::sqrt(
                (double)M(0,(Eigen::Index)c)*M(0,(Eigen::Index)c) +
                (double)M(1,(Eigen::Index)c)*M(1,(Eigen::Index)c) +
                (double)M(2,(Eigen::Index)c)*M(2,(Eigen::Index)c));
            inv[c] = (n > 1e-9) ? 1.0 / n : 0.0;
            k.s[c] = n;
        }
        // Pure rotation matrix in Nuke's accessor convention (a_rc = row r, col c).
        DD::Image::Matrix4 Rn; Rn.makeIdentity();
        Rn.a00 = (float)(M(0,0)*inv[0]); Rn.a01 = (float)(M(0,1)*inv[1]); Rn.a02 = (float)(M(0,2)*inv[2]);
        Rn.a10 = (float)(M(1,0)*inv[0]); Rn.a11 = (float)(M(1,1)*inv[1]); Rn.a12 = (float)(M(1,2)*inv[2]);
        Rn.a20 = (float)(M(2,0)*inv[0]); Rn.a21 = (float)(M(2,1)*inv[1]); Rn.a22 = (float)(M(2,2)*inv[2]);
        // Nuke's own decomposition for the ZXY order (returns radians).
        float rx = 0.f, ry = 0.f, rz = 0.f;
        Rn.rotationsZXY(rx, ry, rz);
        const double r2d = 180.0 / 3.14159265358979323846;
        k.r[0] = rx * r2d; k.r[1] = ry * r2d; k.r[2] = rz * r2d;
        // Per-frame focal (mm): the solved curve when variable, else the static
        // camera focal (the Python ignores this column in Model mode).
        k.focal = focal;
        if (var_focal) {
            if (Knob* sf = knob("solved_focal")) k.focal = sf->get_value_at((double)f);
        }
        // Principal point -> win_translate (lens shift). Convert the solved cx,cy
        // (OpenCV/y-down px) to Nuke's NDC window offset using the SAME uniform
        // half-width (w/2) scale the solver and overlay use on both axes:
        //   win_translate.x = (w - 2*cx) / w
        //   win_translate.y = (2*cy - h) / w      (y flips: OpenCV y-down -> NDC y-up)
        // Centred (cx=w/2, cy=h/2) -> (0,0). Assumes win_translate is in NDC units
        // (full frame width spans 2.0), the standard Nuke camera convention — verify
        // sign/scale once with a round-trip if a shifted plate looks off.
        k.wt[0] = 0.0; k.wt[1] = 0.0;
        if (var_principal) {
            double cxp = (double)pp_w * 0.5, cyp = (double)pp_h * 0.5;
            if (Knob* kx = knob("solved_cx")) cxp = kx->get_value_at((double)f);
            if (Knob* ky = knob("solved_cy")) cyp = ky->get_value_at((double)f);
            k.wt[0] = ((double)pp_w - 2.0 * cxp) / (double)pp_w;
            k.wt[1] = (2.0 * cyp - (double)pp_h) / (double)pp_w;
        }
        keys.push_back(k);
    }

    // ---- Model mode + 'Export Lens Camera': spawn a camera with the SOLVED lens --
    // Model mode bakes only the object motion (TransformGeo); it has no camera, so on
    // a zoom the geo is viewed through whatever render camera exists — which carries
    // the WRONG (static) focal, and the cube drifts in scale. Camera mode dodges this
    // because the camera it spawns owns the focal. The Model-mode counterpart is to
    // ALSO spawn a viewing camera: it matches the INPUT camera's per-frame WORLD pose
    // (so the viewpoint is identical) but carries the SOLVED per-frame focal. Render
    // the TransformGeo'd geo through THIS camera and it lines up. Opt-in (checkbox),
    // deterministic ('Polychase_LensCamera_1' — re-export updates it), and it NEVER
    // mutates the input camera. No-op without a solved focal or a connected camera.
    //
    // Pose is baked from cam_world_at (the camera's TRUE world matrix — lookat /
    // constraints included, so it's right even when the input is aimed by a lookat)
    // and decomposed to ZXY translate/rotate exactly like the main export, so the
    // spawned camera carries editable curves rather than a frozen matrix. The zoom
    // rides 'focal'; haperture/vaperture/win_translate are copied static.
    std::string lens_py, lens_note;
    if (export_lens_camera_ && !camera_mode) {
        CameraOp* lcam = input_cam();
        if (!lcam) {
            lens_note =
                "  [WARN] 'Export Zoom Lens Camera' is on but no camera is wired to the cam\n"
                "  input — cannot build a lens camera. Wire the shot camera and re-export.\n";
        } else if (!has_solved_focal()) {
            lens_note =
                "  [NOTE] 'Export Zoom Lens Camera' is on but there is no Solved Focal curve, so\n"
                "  a lens camera would just duplicate the input lens. Solve Focal first.\n";
        } else {
            double l_hap = haperture, l_vap = vaperture, l_wt0 = 0.0, l_wt1 = 0.0;
            if (Knob* k = lcam->knob("haperture"))     l_hap = k->get_value();
            if (Knob* k = lcam->knob("vaperture"))     l_vap = k->get_value();
            if (Knob* k = lcam->knob("win_translate")) { l_wt0 = k->get_value(0); l_wt1 = k->get_value(1); }

            Knob* sfk = knob("solved_focal");
            std::ostringstream lk;   // Python list of (f, tx,ty,tz, rx,ry,rz, focal)
            lk.setf(std::ios::fixed); lk.precision(8);
            int n_lens = 0;
            for (int f = f0; f <= f1; ++f) {
                const RowMajorMatrix4f W = nuke_to_eigen_m4(cam_world_at(lcam, (double)f));
                double t[3]; for (int i = 0; i < 3; ++i) t[i] = (double)W((Eigen::Index)i, 3);
                double inv[3];
                for (int c = 0; c < 3; ++c) {
                    const double nrm = std::sqrt(
                        (double)W(0,(Eigen::Index)c)*W(0,(Eigen::Index)c) +
                        (double)W(1,(Eigen::Index)c)*W(1,(Eigen::Index)c) +
                        (double)W(2,(Eigen::Index)c)*W(2,(Eigen::Index)c));
                    inv[c] = (nrm > 1e-9) ? 1.0 / nrm : 0.0;
                }
                DD::Image::Matrix4 Rn; Rn.makeIdentity();
                Rn.a00=(float)(W(0,0)*inv[0]); Rn.a01=(float)(W(0,1)*inv[1]); Rn.a02=(float)(W(0,2)*inv[2]);
                Rn.a10=(float)(W(1,0)*inv[0]); Rn.a11=(float)(W(1,1)*inv[1]); Rn.a12=(float)(W(1,2)*inv[2]);
                Rn.a20=(float)(W(2,0)*inv[0]); Rn.a21=(float)(W(2,1)*inv[1]); Rn.a22=(float)(W(2,2)*inv[2]);
                float rx=0.f, ry=0.f, rz=0.f; Rn.rotationsZXY(rx, ry, rz);
                const double r2d = 180.0 / 3.14159265358979323846;
                double fo = focal;
                if (sfk) { const double mm = sfk->get_value_at((double)f); if (std::isfinite(mm) && mm > 1e-6) fo = mm; }
                lk << "(" << f << "," << t[0] << "," << t[1] << "," << t[2] << ","
                   << (rx*r2d) << "," << (ry*r2d) << "," << (rz*r2d) << "," << fo << "),";
                ++n_lens;
            }

            std::ostringstream lp;
            lp << "import nuke\n"
               << "_LNAME='Polychase_LensCamera_1'\n"
               << "_HAP=" << l_hap << "\n_VAP=" << l_vap
               << "\n_WT0=" << l_wt0 << "\n_WT1=" << l_wt1 << "\n"
               << "_LKEYS=[" << lk.str() << "]\n"
               << "_CLS='Camera2'\n"
               << "for c in ('Camera2','Camera4','Camera3'):\n"
               << "    if hasattr(nuke.nodes,c): _CLS=c; break\n"
               << "n=nuke.toNode(_LNAME) or getattr(nuke.nodes,_CLS)(name=_LNAME)\n"
               << "for kn,vl in (('haperture',_HAP),('vaperture',_VAP)):\n"
               << "    if kn in n.knobs():\n"
               << "        try: n[kn].clearAnimated()\n"
               << "        except Exception: pass\n"
               << "        n[kn].setValue(vl)\n"
               << "if 'win_translate' in n.knobs():\n"
               << "    for i in range(2):\n"
               << "        try: n['win_translate'].clearAnimated(i)\n"
               << "        except Exception: pass\n"
               << "    n['win_translate'].setValue(_WT0,0); n['win_translate'].setValue(_WT1,1)\n"
               << "if 'useMatrix' in n.knobs():\n"
               << "    try: n['useMatrix'].setValue(False)\n"
               << "    except Exception: pass\n"
               << "if 'rot_order' in n.knobs():\n"
               << "    try: n['rot_order'].setValue('ZXY')\n"
               << "    except Exception: pass\n"
               << "for kn in ('translate','rotate','focal'):\n"
               << "    if kn in n.knobs():\n"
               << "        for i in range(1 if kn=='focal' else 3):\n"
               << "            try: n[kn].clearAnimated(i)\n"
               << "            except Exception: pass\n"
               << "            n[kn].setAnimated(i)\n"
               << "tk=n['translate']; rk=n['rotate']; fk=n['focal']\n"
               << "for k in _LKEYS:\n"
               << "    f=k[0]\n"
               << "    tk.setValueAt(k[1],f,0); tk.setValueAt(k[2],f,1); tk.setValueAt(k[3],f,2)\n"
               << "    rk.setValueAt(k[4],f,0); rk.setValueAt(k[5],f,1); rk.setValueAt(k[6],f,2)\n"
               << "    fk.setValueAt(k[7],f)\n"
               << "print('PolychaseTracker: lens camera %s built (%d keys)'%(_LNAME,len(_LKEYS)))\n";
            lens_py = lp.str();

            std::ostringstream m;
            m << "  Lens camera 'Polychase_LensCamera_1' built (" << n_lens
              << " key(s)): the input camera's viewpoint carrying the solved zoom focal.\n"
              << "  Render the exported TransformGeo's geo through THIS camera (not the\n"
              << "  static input cam) and the object locks across the zoom.\n";
            lens_note = m.str();
        }
    }

    // ---- Emit a self-contained pick/create dialog script ----
    // Bakes translate/rotate/scaling as keyframes (useMatrix off, rot_order=ZXY)
    // — artist-editable curves. A native nuke.Panel lets the artist pick an
    // existing node or create one. No PySide.
    // Candidate node CLASSES the picker may create. Cameras: offer classic
    // (Camera2/3) and new-3D (Camera4) — the baked keys are class-agnostic
    // (all share the CameraOp knob set: translate/rotate/focal/haperture/
    // vaperture/win_translate), so the artist targets whichever they want. The
    // Python filters this to the classes that actually exist in this Nuke build.
    // Model mode also offers the new-3D transforms (GeoTransform/GeoXform); _apply
    // tolerates knob-layout differences and warns if a channel can't be keyed.
    const std::string types_list  = camera_mode ? "['Camera2', 'Camera3', 'Camera4']"
                                                 : "['TransformGeo', 'GeoTransform', 'GeoXform']";
    const std::string node_type    = camera_mode ? "Camera" : "TransformGeo"; // status label only
    const std::string default_name = camera_mode ? "Polychase_Camera_1"
                                                  : "Polychase_TransformGeo_1";
    std::ostringstream py;
    py << "# Auto-generated by PolychaseTracker (Export, " << mode_name << " mode)\n"
       << "import nuke\n"
       << "_TYPES = " << types_list << "\n"
       << "_MODE = '" << (camera_mode ? "camera" : "model") << "'\n"
       << "_DEFAULT = '" << default_name << "'\n";
    if (camera_mode) {
        py << "_FOCAL = " << pyf(focal)     << "\n"
           << "_HAP   = " << pyf(haperture) << "\n"
           << "_VAP   = " << pyf(vaperture) << "\n"
           << "_WT0   = " << pyf(in_wt0)    << "\n"
           << "_WT1   = " << pyf(in_wt1)    << "\n"
           << "_VAR_FOCAL = "     << (var_focal     ? "True" : "False") << "\n"
           << "_VAR_PRINCIPAL = " << (var_principal ? "True" : "False") << "\n";
    }
    // rotate angles are already in Nuke's ZXY convention (Matrix4::rotationsZXY).
    py << "_KEYS = [\n";
    for (const Key& k : keys) {
        py << " (" << k.f << ","
           << pyf(k.t[0]) << "," << pyf(k.t[1]) << "," << pyf(k.t[2]) << ","
           << pyf(k.r[0]) << "," << pyf(k.r[1]) << "," << pyf(k.r[2]) << ","
           << pyf(k.s[0]) << "," << pyf(k.s[1]) << "," << pyf(k.s[2]) << ","
           << pyf(k.focal) << ","
           << pyf(k.wt[0]) << "," << pyf(k.wt[1]) << "),\n";
    }
    py << "]\n";
    py << R"PY(
_CREATE = '<create-new>'

def _apply(n):
    if 'useMatrix' in n.knobs():
        n['useMatrix'].setValue(False)
    if 'rot_order' in n.knobs():
        n['rot_order'].setValue('ZXY')   # matches the angles we baked
    fk = None
    wtk = None
    if _MODE == 'camera':
        for kn in ('focal', 'haperture', 'vaperture'):
            try: n[kn].clearAnimated()
            except Exception: pass
        n['haperture'].setValue(_HAP)
        n['vaperture'].setValue(_VAP)
        if _VAR_FOCAL:
            n['focal'].setAnimated()
            fk = n['focal']            # keyed per-frame in the loop below
        else:
            n['focal'].setValue(_FOCAL)
        if _VAR_PRINCIPAL and 'win_translate' in n.knobs():
            for i in range(2):
                try: n['win_translate'].clearAnimated(i)
                except Exception: pass
                n['win_translate'].setAnimated(i)
            wtk = n['win_translate']   # solved lens shift, keyed below
        elif 'win_translate' in n.knobs():
            # Not solving the principal point: clear any stale animated shift from a
            # previous export and pin win_translate to the input camera's static value.
            for i in range(2):
                try: n['win_translate'].clearAnimated(i)
                except Exception: pass
            n['win_translate'].setValue(_WT0, 0)
            n['win_translate'].setValue(_WT1, 1)
    chans = ('translate', 'rotate') if _MODE == 'camera' else ('translate', 'rotate', 'scaling')
    # New-system transforms (GeoTransform/GeoXform) may name channels differently;
    # only touch knobs that exist, and warn about any we couldn't key so the
    # export never silently no-ops on an unexpected class.
    missing = [kn for kn in chans if kn not in n.knobs()]
    for kn in chans:
        if kn not in n.knobs():
            continue
        k = n[kn]
        for i in range(3):
            try: k.clearAnimated(i)
            except Exception: pass
            k.setAnimated(i)
    tk = n['translate'] if 'translate' in n.knobs() else None
    rk = n['rotate']    if 'rotate'    in n.knobs() else None
    sk = n['scaling'] if (_MODE != 'camera' and 'scaling' in n.knobs()) else None
    for k in _KEYS:
        f = k[0]
        if tk is not None:
            tk.setValueAt(k[1], f, 0); tk.setValueAt(k[2], f, 1); tk.setValueAt(k[3], f, 2)
        if rk is not None:
            rk.setValueAt(k[4], f, 0); rk.setValueAt(k[5], f, 1); rk.setValueAt(k[6], f, 2)
        if sk is not None:
            sk.setValueAt(k[7], f, 0); sk.setValueAt(k[8], f, 1); sk.setValueAt(k[9], f, 2)
        if fk is not None:
            fk.setValueAt(k[10], f)
        if wtk is not None:
            wtk.setValueAt(k[11], f, 0); wtk.setValueAt(k[12], f, 1)
    _msg = 'T/R/S' + ('/focal' if fk is not None else '') + ('/shift' if wtk is not None else '')
    print('PolychaseTracker: applied %d %s keys to %s' % (len(_KEYS), _msg, n.name()))
    if missing:
        print('PolychaseTracker: WARNING %s (%s) has no %s knob(s) — those channels were NOT keyed'
              % (n.name(), n.Class(), ', '.join(missing)))

# Classes from _TYPES that actually exist in this Nuke build (so the pulldown
# never offers an uncreatable class). Falls back to the first candidate.
_CREATABLE = [c for c in _TYPES if hasattr(nuke.nodes, c)] or [_TYPES[0]]

def _make(name, cls):
    return nuke.toNode(name) or getattr(nuke.nodes, cls)(name=name)

def _pick_and_apply():
    # Headless: no dialog, just create/update the default-named node as the
    # first available class.
    if not nuke.GUI:
        _apply(_make(_DEFAULT, _CREATABLE[0])); return
    existing = [x.name() for x in nuke.allNodes() if x.Class() in _TYPES]
    items = [_CREATE] + existing
    p = nuke.Panel('Polychase Export')
    p.addEnumerationPulldown('Create as (if new)', ' '.join(_CREATABLE))
    p.addEnumerationPulldown('Target node', ' '.join(items))
    p.addSingleLineInput('New name (if creating)', _DEFAULT)
    if not p.show():
        print('PolychaseTracker: export cancelled'); return
    cls     = p.value('Create as (if new)')
    sel     = p.value('Target node')
    newname = (p.value('New name (if creating)') or _DEFAULT).strip()
    n = _make(newname, cls) if sel == _CREATE else (nuke.toNode(sel) or _make(newname, cls))
    _apply(n)

_pick_and_apply()
)PY";

    oss << "  [PASS] built " << keys.size() << " keyframes ("
        << f0 << ".." << f1 << "), seed @ " << seed_frame << "\n";

    // ---- Run it in-process, NO temp file ----
    // The whole picker/apply script (built in `py`) is base64-encoded and exec'd
    // straight through Nuke's Python engine. Base64 is pure [A-Za-z0-9+/=], so the
    // multi-line script — quotes, braces, %, newlines and all — passes through
    // script_command() with zero escaping and never touches disk. (The previous
    // build wrote a .py next to the database and exec(open(...))'d it, which
    // littered Nuke's launch dir and broke when that dir wasn't writable.)
    auto b64 = [](const std::string& in) {
        static const char T[] =
            "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        std::string out;
        out.reserve(((in.size() + 2) / 3) * 4);
        size_t i = 0;
        for (; i + 2 < in.size(); i += 3) {
            const unsigned n = ((unsigned)(unsigned char)in[i]   << 16)
                             | ((unsigned)(unsigned char)in[i+1] << 8)
                             |  (unsigned)(unsigned char)in[i+2];
            out += T[(n >> 18) & 63]; out += T[(n >> 12) & 63];
            out += T[(n >> 6)  & 63]; out += T[n & 63];
        }
        if (i < in.size()) {
            const bool two = (i + 1 < in.size());
            unsigned n = (unsigned)(unsigned char)in[i] << 16;
            if (two) n |= (unsigned)(unsigned char)in[i+1] << 8;
            out += T[(n >> 18) & 63];
            out += T[(n >> 12) & 63];
            out += two ? T[(n >> 6) & 63] : '=';
            out += '=';
        }
        return out;
    };

    // script_command() defaults to eval=true, which only accepts a single
    // EXPRESSION — so we cannot use "import base64; exec(...)" (a statement). Use
    // __import__('base64') inline so the whole thing is one exec(...) call.
    const std::string cmd =
        "exec(__import__('base64').b64decode('" + b64(py.str()) +
        "').decode('utf-8'))";

    // Node creation must go through Nuke's script engine; Op::script_command runs
    // Python (py defaults true).
    bool launched = false;
    std::string err_text;
    {
        const bool ok = this->script_command(cmd.c_str());   // py = true (default)
        if (const char* res = Op::script_result()) err_text = res;
        Op::script_unlock();
        const bool looks_error =
            err_text.find("Error")     != std::string::npos ||
            err_text.find("Traceback") != std::string::npos;
        launched = ok && !looks_error;
    }

    if (launched) {
        oss << "  Opened the " << node_type << " picker — pick or create a node "
            << "and the keys are applied.";
    } else {
        oss << "  [WARN] could not auto-run export";
        if (!err_text.empty()) oss << " (" << err_text << ")";
        oss << ".\n"
            << "  Paste this into the Script Editor instead:\n"
            << "  ----------------------------------------------------------------\n"
            << py.str()
            << "  ----------------------------------------------------------------";
    }
    // Spawn/refresh the lens camera (Model mode + Export Lens Camera). Deterministic
    // create-or-update of 'Polychase_LensCamera_1' — no picker — reusing the same
    // base64/exec plumbing as the main script. Built above; run here where b64 is in
    // scope. Independent of the TransformGeo picker outcome.
    if (!lens_py.empty()) {
        const std::string lcmd =
            "exec(__import__('base64').b64decode('" + b64(lens_py) + "').decode('utf-8'))";
        const bool lok = this->script_command(lcmd.c_str());
        std::string lerr; if (const char* r = Op::script_result()) lerr = r;
        Op::script_unlock();
        if (!lok || lerr.find("Error") != std::string::npos ||
                    lerr.find("Traceback") != std::string::npos) {
            lens_note += "  [WARN] lens camera spawn reported a problem";
            if (!lerr.empty()) lens_note += " (" + lerr + ")";
            lens_note += ".\n";
        }
    }

    if (!lens_note.empty()) oss << lens_note;
    set_status(oss.str());
}


// =============================================================================
// Live Camera Solve — maintain the object pose re-expressed as a moving camera
// (geo at rest) on the node's own live_cam_translate / live_cam_rotate curves,
// so a Camera2 expression-linked to them previews the matchmove without Export.
// The math is identical to Export's Camera mode:
//     M_cam(t) = G * model'(t)^-1 * cam_world_seed     (camera-to-world)
// with G the geo rest matrix and cam_world_seed the camera-to-world at the seed
// frame. translate = last column; rotate via Nuke's ZXY (so a Camera2 with
// rot_order = ZXY and useMatrix off reproduces it exactly).
// =============================================================================

// Decompose a camera-to-world matrix to translate + ZXY rotate (degrees).
static void camera_mode_trs(const RowMajorMatrix4f& model_prime,
                            const RowMajorMatrix4f& G,
                            const RowMajorMatrix4f& cam_world_seed,
                            double t_out[3], double r_out[3])
{
    const RowMajorMatrix4f M = G * model_prime.inverse() * cam_world_seed;
    for (int i = 0; i < 3; ++i) t_out[i] = (double)M((Eigen::Index)i, 3);

    double inv[3];
    for (int c = 0; c < 3; ++c) {
        const double n = std::sqrt(
            (double)M(0,(Eigen::Index)c)*M(0,(Eigen::Index)c) +
            (double)M(1,(Eigen::Index)c)*M(1,(Eigen::Index)c) +
            (double)M(2,(Eigen::Index)c)*M(2,(Eigen::Index)c));
        inv[c] = (n > 1e-9) ? 1.0 / n : 0.0;
    }
    DD::Image::Matrix4 Rn; Rn.makeIdentity();
    Rn.a00 = (float)(M(0,0)*inv[0]); Rn.a01 = (float)(M(0,1)*inv[1]); Rn.a02 = (float)(M(0,2)*inv[2]);
    Rn.a10 = (float)(M(1,0)*inv[0]); Rn.a11 = (float)(M(1,1)*inv[1]); Rn.a12 = (float)(M(1,2)*inv[2]);
    Rn.a20 = (float)(M(2,0)*inv[0]); Rn.a21 = (float)(M(2,1)*inv[1]); Rn.a22 = (float)(M(2,2)*inv[2]);
    float rx = 0.f, ry = 0.f, rz = 0.f;
    Rn.rotationsZXY(rx, ry, rz);
    const double r2d = 180.0 / 3.14159265358979323846;
    r_out[0] = rx * r2d; r_out[1] = ry * r2d; r_out[2] = rz * r2d;
}


bool PolychaseTracker::live_camera_basis(RowMajorMatrix4f& G,
                                         RowMajorMatrix4f& cam_world_seed)
{
    CameraOp* cam = input_cam();
    Op*       geo = input_geo_op();
    if (!cam || !geo) return false;
    cam->validate(true);
    geo->validate(true);

    RowMajorMatrix4f U = RowMajorMatrix4f::Identity();
    GeoMesh gm;
    if (extract_mesh(geo, gm)) U = nuke_to_eigen_m4(gm.object_to_world);
    G = U;   // geo-at-rest world (camera orbits this)

    const int seed_frame = (track_seed_frame_ != 0) ? track_seed_frame_ : first_frame_;
    // Cache the seed-frame camera-to-world so the per-redraw readout doesn't
    // re-sample the camera (a setOutputContext + double validate) every frame.
    if (live_cam_seed_valid_ && live_cam_seed_cached_ == seed_frame) {
        cam_world_seed = live_cam_world_seed_cached_;
    } else {
        cam_world_seed              = nuke_to_eigen_m4(cam_world_at(cam, (double)seed_frame));
        live_cam_world_seed_cached_ = cam_world_seed;
        live_cam_seed_cached_       = seed_frame;
        live_cam_seed_valid_        = true;
    }
    return true;
}


// Rewrite the live-camera curves over First..Last from the keyed pose. Runs at
// the end of Track and from the Refresh Live Camera button.
void PolychaseTracker::bake_live_camera()
{
    if (!has_pose_keys()) {
        PCN_LOG("[livecam] no solved pose to bake\n");
        return;
    }
    RowMajorMatrix4f G, cam_world_seed;
    if (!live_camera_basis(G, cam_world_seed)) {
        PCN_LOG("[livecam] need cam (input 1) + geo (input 2) to bake\n");
        return;
    }
    DD::Image::Knob* kt = knob("live_cam_translate");
    DD::Image::Knob* kr = knob("live_cam_rotate");
    if (!kt || !kr) return;

    const int f0 = first_frame_, f1 = last_frame_;
    for (int f = f0; f <= f1; ++f) {
        const RowMajorMatrix4f model_prime =
            nuke_to_eigen_m4(pose_matrix_to_nuke((double)f));
        double t[3], r[3];
        camera_mode_trs(model_prime, G, cam_world_seed, t, r);
        for (int i = 0; i < 3; ++i) { if (!kt->is_animated(i)) kt->set_animated(i); kt->set_value_at(t[i], (double)f, i); }
        for (int i = 0; i < 3; ++i) { if (!kr->is_animated(i)) kr->set_animated(i); kr->set_value_at(r[i], (double)f, i); }
    }
    kt->changed();
    kr->changed();
    PCN_LOG("[livecam] baked live camera curve " << f0 << ".." << f1 << "\n");
    asapUpdate();
}


// Refresh just the current frame's live-camera key from the working/keyed pose.
// Called right after a pin/gizmo commit so the linked camera tracks each refine.
void PolychaseTracker::update_live_camera_key_current()
{
    RowMajorMatrix4f G, cam_world_seed;
    if (!live_camera_basis(G, cam_world_seed)) return;

    const double frame = (double)editing_frame();
    const RowMajorMatrix4f model_prime = live_scene_
        ? live_scene_->model_matrix
        : nuke_to_eigen_m4(pose_matrix_to_nuke(frame));

    double t[3], r[3];
    camera_mode_trs(model_prime, G, cam_world_seed, t, r);

    auto key = [&](const char* name, const double v[3]) {
        DD::Image::Knob* k = knob(name);
        if (!k) return;
        for (int i = 0; i < 3; ++i) { if (!k->is_animated(i)) k->set_animated(i); k->set_value_at(v[i], frame, i); }
        k->changed();
    };
    key("live_cam_translate", t);
    key("live_cam_rotate",    r);
}

} // namespace pcn