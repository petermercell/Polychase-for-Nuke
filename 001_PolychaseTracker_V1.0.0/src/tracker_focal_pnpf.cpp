// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Peter Mercell
//
// Developed with assistance from Claude (Anthropic).
//
// =============================================================================
// tracker_focal_pnpf.cpp — from-scratch, per-frame focal solve for zoom shots.
//
// WHY THIS EXISTS (and why the alternating Solve Zoom / the focal SWEEP failed):
//   * Solve Zoom alternates pose<->focal and gets trapped in the constant-focal
//     local minimum (the object's zoom is explained as moving closer).
//   * The focal SWEEP re-solved the pose per candidate focal, but it leaned on the
//     iterative pin PnP warm-started from the previous frame. Over a large camera
//     sweep that PnP LAGS (~half the rotation per frame) and the lag compounds, so
//     the focal collapsed toward the short end to compensate. It also could not
//     use a per-frame camera basis because Nuke's cooked imatrix() is STALE under
//     a forced OutputContext (the same staleness class as projection()'s focal).
//
// THE FIX (per the focal_solve_reference.cpp sketch, adapted to the installed
//   PoseLib 2.0.4 API): solve the FULL pose AND focal FROM SCRATCH at every frame
//   with PoseLib's P4Pf minimal solver inside a small LO-RANSAC. From-scratch ⇒ no
//   warm-start, so no lag to accumulate; and it never reads the Nuke camera matrix,
//   so the staleness is irrelevant. Correspondences come from the USER TRACKS
//   (per-frame 2D + the anchored 3D object point) — the clean, full-length data the
//   focal solve needs. Output is ONLY the Solved Focal curve (mm/frame); 'Smooth
//   Solved Focal' is the temporal-smoothing stage (no Ceres dependency needed).
//
// CONVENTION: PoseLib's p4pf expects CENTERED pixels (principal point at origin):
//   x_centered = focal · (R·X + t).xy / (R·X + t).z   [focal in PIXELS]
// The user-track store is OpenCV Y-DOWN, and conv::tracker_intrinsics gives cx/cy
// in the SAME OpenCV Y-DOWN convention, so we center by (cx,cy) with no Y flip.
// We only read the focal back (a convention-independent scalar), so the camera
// handedness of the returned pose does not matter and no GL↔CV flip is required.
//   focal_mm = focal_px · haperture / image_width   (inverse of fx = f_mm·w/hap).
// =============================================================================

#include "polychase_tracker.h"

#include "pcn_convention.h"   // conv::tracker_intrinsics (OpenCV cx/cy from projection)

#include "PoseLib/camera_pose.h"
#include "PoseLib/solvers/p4pf.h"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <random>
#include <sstream>
#include <vector>

namespace pcn {

namespace {

// (pcn::kPi for the FOV<->focal conversion is already defined in
// polychase_pose_math.h, pulled in via polychase_tracker.h — reuse it.)

// Reproject a 3D point through (pose, focal) into CENTERED pixel space, matching
// p4pf's model x_centered = focal·(R·X + t).xy / (R·X + t).z. Returns false if the
// point is at/behind the camera (z<=0), which must not score as an inlier.
inline bool reproj_centered(const poselib::CameraPose& pose, double focal,
                            const Eigen::Vector3d& X, Eigen::Vector2d& out)
{
    const Eigen::Vector3d Xc = pose.apply(X);   // R·X + t
    if (Xc.z() <= 1e-9) return false;
    out = focal * Xc.head<2>() / Xc.z();
    return true;
}

struct P4PfResult {
    double focal_px = 0.0;
    int    inliers  = 0;
    double rms_px   = 1e30;
    bool   ok       = false;
};

// From-scratch P4Pf LO-RANSAC over centered 2D <-> local-3D correspondences.
// Samples 4 points, solves pose+focal with poselib::p4pf, scores ALL points by
// centered-pixel reprojection, keeps the model with the most inliers (tie-break
// on lower RMS). A focal sanity window (from the FOV bounds) rejects wild minimal
// solutions. No warm start anywhere — each frame is independent.
P4PfResult p4pf_ransac(const std::vector<Eigen::Vector2d>& x,   // centered pixels
                       const std::vector<Eigen::Vector3d>& X,   // local 3D
                       double thresh_px, double f_min_px, double f_max_px,
                       int max_iters, std::mt19937& rng)
{
    P4PfResult best;
    const int N = static_cast<int>(x.size());
    if (N < 4) return best;

    const double t2 = thresh_px * thresh_px;
    std::uniform_int_distribution<int> pick(0, N - 1);

    std::vector<Eigen::Vector2d> xs(4);
    std::vector<Eigen::Vector3d> Xs(4);
    std::vector<poselib::CameraPose> poses;
    std::vector<double> focals;

    for (int it = 0; it < max_iters; ++it) {
        // sample 4 distinct indices
        int idx[4];
        for (int s = 0; s < 4; ++s) {
            int v; bool dup;
            do { v = pick(rng); dup = false; for (int j = 0; j < s; ++j) if (idx[j] == v) dup = true; } while (dup);
            idx[s] = v;
        }
        for (int s = 0; s < 4; ++s) { xs[s] = x[idx[s]]; Xs[s] = X[idx[s]]; }

        poses.clear(); focals.clear();
        const int nsol = poselib::p4pf(xs, Xs, &poses, &focals, /*filter_solutions=*/true);

        for (int k = 0; k < nsol; ++k) {
            const double f = focals[(size_t)k];
            if (!(f > f_min_px && f < f_max_px) || !std::isfinite(f)) continue;  // focal sanity

            int inl = 0; double sse = 0.0;
            for (int i = 0; i < N; ++i) {
                Eigen::Vector2d p;
                if (!reproj_centered(poses[(size_t)k], f, X[(size_t)i], p)) continue;
                const double e2 = (p - x[(size_t)i]).squaredNorm();
                if (e2 < t2) { ++inl; sse += e2; }
            }
            const double rms = (inl > 0) ? std::sqrt(sse / inl) : 1e30;
            if (inl > best.inliers || (inl == best.inliers && inl > 0 && rms < best.rms_px)) {
                best.inliers = inl;
                best.focal_px = f;
                best.rms_px = rms;
                best.ok = (inl >= 4);
            }
        }
        if (best.inliers >= N) break;   // unanimous consensus — done
    }
    return best;
}

} // namespace

// =============================================================================
// on_solve_focal_pnpf — Zoom-tab entry point.
// =============================================================================
void PolychaseTracker::on_solve_focal_pnpf()
{
    std::ostringstream oss;
    oss << "[" << timestamp() << "] Solve Focal (P4Pf) — from-scratch per-frame focal\n";

    DD::Image::CameraOp* cam = input_cam();
    DD::Image::Iop*      img = input_img();
    if (!cam) { oss << "  [FAIL] camera input not connected."; set_status(oss.str()); return; }
    if (last_frame_ <= first_frame_) {
        oss << "  [FAIL] Last Frame must be greater than First Frame.";
        set_status(oss.str()); return;
    }
    cam->validate(true);
    if (img) img->validate(true);

    float fmt_w = 2048.0f, fmt_h = 1080.0f;
    if (img) { const DD::Image::Format& fmt = img->info().format(); fmt_w = (float)fmt.width(); fmt_h = (float)fmt.height(); }
    const double w = (double)fmt_w;

    double haperture = 24.576;
    if (DD::Image::Knob* hk = cam->knob("haperture")) haperture = hk->get_value();

    // Principal point (OpenCV / Y-DOWN) via the plugin's single convention bridge.
    // We use only cx/cy here; the projection's stale focal is irrelevant (P4Pf
    // solves focal from scratch). The user-track store is OpenCV Y-DOWN too, so the
    // 2D and the principal point share a convention — center with no Y flip.
    const DD::Image::Matrix4 base_proj = cam->projection();
    const CameraIntrinsics    intr     = conv::tracker_intrinsics(base_proj, fmt_w, fmt_h);
    const double cx = (double)intr.cx;
    const double cy = (double)intr.cy;

    // Focal sanity window (pixels) from the Min/Max FOV bounds the other solvers clamp to.
    // f = (w/2) / tan(fov/2).
    const double f_min_px = (w * 0.5) / std::tan(0.5 * max_fov_deg_ * kPi / 180.0);  // widest FOV -> shortest focal
    const double f_max_px = (w * 0.5) / std::tan(0.5 * min_fov_deg_ * kPi / 180.0);  // narrowest FOV -> longest focal
    if (!(f_max_px > f_min_px) || !std::isfinite(f_min_px) || !std::isfinite(f_max_px)) {
        oss << "  [FAIL] bad focal window from FOV bounds (min_fov=" << min_fov_deg_
            << " max_fov=" << max_fov_deg_ << ").";
        set_status(oss.str()); return;
    }

    UserTracks tracks = build_solve_tracks();
    if (tracks.size() < 4) {
        oss << "  [FAIL] Need >= 4 user tracks (got " << tracks.size() << ").\n"
            << "  Pose the cube at the Reference Frame, then 'Load Tracks' so the log\n"
            << "  prints '[utracks] anchored N/N' — that is the per-frame 2D + 3D source.";
        set_status(oss.str()); return;
    }
    PCN_LOG("[fpnpf] source = USER TRACKS (" << tracks.size() << " track(s)); pp=("
            << cx << "," << cy << ") f_window_px=[" << f_min_px << "," << f_max_px << "]\n");

    std::mt19937 rng(0xC0FFEEu);
    const double thresh_px = 3.0;     // inlier reprojection threshold (centered px)
    const int    max_iters = 300;     // plenty for ~8 clean points / 4-pt samples

    int    solved = 0, skipped = 0;
    double sum_rms = 0.0;

    for (int t = first_frame_; t <= last_frame_; ++t) {
        std::vector<Eigen::Vector2d> x;   // centered pixels (OpenCV Y-DOWN)
        std::vector<Eigen::Vector3d> X;   // local 3D (object frame)
        x.reserve(tracks.size());
        X.reserve(tracks.size());

        for (const UserTrack& ut : tracks) {
            const Eigen::Vector2f* o = nullptr;
            for (const UserTrackObservation& ob : ut.observations)
                if (ob.frame_id == t) { o = &ob.image_point; break; }
            if (!o) continue;
            x.emplace_back((double)o->x() - cx, (double)o->y() - cy);
            X.emplace_back((double)ut.object_point.x(),
                           (double)ut.object_point.y(),
                           (double)ut.object_point.z());
        }

        if (x.size() < 4) {
            ++skipped;
            PCN_LOG("[fpnpf] f=" << t << " SKIP (" << x.size() << " obs; need >=4)\n");
            continue;
        }

        const P4PfResult r = p4pf_ransac(x, X, thresh_px, f_min_px, f_max_px, max_iters, rng);
        if (!r.ok) {
            ++skipped;
            PCN_LOG("[fpnpf] f=" << t << " FAIL (no P4Pf consensus; obs=" << x.size() << ")\n");
            continue;
        }

        const double focal_mm = r.focal_px * haperture / w;
        key_focal_at((double)t, focal_mm);
        ++solved; sum_rms += r.rms_px;

        PCN_LOG("[fpnpf] f=" << t << " focal=" << focal_mm << "mm  focal_px=" << r.focal_px
                << "  inliers=" << r.inliers << "/" << x.size() << "  rms=" << r.rms_px << "px\n");
    }

    if (solved == 0) {
        oss << "  [FAIL] No frame produced a P4Pf consensus. Check that the tracks\n"
            << "  span depth (not all coplanar) and have clean 2D across the shot.";
        set_status(oss.str());
        return;
    }

    const int span = last_frame_ - first_frame_ + 1;
    oss << "  [PASS] P4Pf focal solved " << solved << " / " << span << " frame(s)"
        << " (skipped " << skipped << ").\n"
        << "    mean inlier RMS = " << (sum_rms / (double)solved) << " px (centered)\n"
        << "  Only the Solved Focal curve was written. The per-frame [fpnpf] log shows\n"
        << "  each frame's focal + inlier count; a frame with few inliers was noisy.\n"
        << "  Use 'Smooth Solved Focal' to clean the curve, then 'Export Solved Focal'\n"
        << "  to bake it onto the camera and track under the known lens.";
    set_status(oss.str());
    asapUpdate();
}

} // namespace pcn