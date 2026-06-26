// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Peter Mercell
//
// Developed with assistance from Claude (Anthropic).

// =============================================================================
// pcn_convention.h — the SINGLE definition of the OpenGL<->OpenCV convention
// bridge shared by Track, Refine and the user-track anchoring.
//
// These three helpers used to be hand-copied into tracker_track.cpp,
// tracker_refine.cpp and tracker_usertracks.cpp (the latter under a `uts_`
// prefix). They are the load-bearing mapping between
//   - the interactive overlay / pin-solve : OpenGL convention, Y-UP image
//     pixels, NEGATIVE fx/fy (build_intrinsics_from_projection in
//     polychase_util.h), and
//   - the optical-flow database             : OpenCV convention, Y-DOWN,
//     real-pixel keypoints with POSITIVE fx/fy and the Y/Z view flip.
// A divergence between the copies silently corrupts a solve (the symptom is
// near-zero inlier ratios everywhere, or a track that drifts vertically), which
// is exactly why they must have ONE definition. They live in namespace
// pcn::conv so the consumers keep their existing call spellings via a `using`.
//
// `inline` (not `static`): one shared definition across every TU that includes
// this, no ODR clash. DDImage is required (Matrix4 / CameraIntrinsics /
// AcceleratedMesh), so unlike polychase_pose_math.h this header is plugin-only.
//
// CONVENTION (do not change without re-deriving the overlay side in
// build_intrinsics_from_projection — the two halves must agree):
//   fx = |a00| * w/2,  fy = |a11| * w/2   (uniform w/2 scale -> square pixels;
//       Nuke's projection() carries equal diagonal terms, the format aspect is
//       NOT in the matrix, so h/2 on fy would wrongly squash it by w/h).
//   cx = w/2 * (1 - a02)                  (lens shift honoured; centred -> w/2)
//   cy = h/2 + a12 * w/2                   (OpenCV / Y-DOWN; the overlay's y-up
//       form is h/2 - a12*w/2 — the sign flip IS the Y convention).
//   convention = OpenCV; the Y/Z flip (gl_to_cv_flip) lives in the VIEW matrix.
// =============================================================================
#ifndef PCN_CONVENTION_H
#define PCN_CONVENTION_H

#include "polychase_util.h"   // RowMajorMatrix4f, GeoMesh, CameraIntrinsics, etc.

#include <cmath>
#include <memory>
#include <utility>

namespace pcn {
namespace conv {

// diag(1, -1, -1, 1): GL camera space (X right, Y up, looks -Z) ->
// OpenCV camera space (X right, Y down, looks +Z). Its own inverse.
inline RowMajorMatrix4f gl_to_cv_flip()
{
    RowMajorMatrix4f f = RowMajorMatrix4f::Identity();
    f(1, 1) = -1.0f;
    f(2, 2) = -1.0f;
    return f;
}

// OpenCV / Y-down / real-pixel intrinsics matching the optical-flow DB. See the
// CONVENTION block at the top of this header for the derivation. The PCN_LOG is
// a no-op unless the plugin is built with debug logging.
inline CameraIntrinsics tracker_intrinsics(const DD::Image::Matrix4& proj,
                                           float w, float h)
{
    const float scale = w * 0.5f;
    CameraIntrinsics intr;
    intr.fx           = std::abs(proj.a00) * scale;
    intr.fy           = std::abs(proj.a11) * scale;
    intr.cx           = scale * (1.0f - proj.a02);
    intr.cy           = h * 0.5f + proj.a12 * scale;
    intr.aspect_ratio = (std::abs(intr.fy) > 1e-6f) ? (intr.fx / intr.fy) : 1.0f;
    intr.width        = w;
    intr.height       = h;
    intr.convention   = CameraConvention::OpenCV;
    PCN_LOG("[PolychaseTracker]   intrinsics " << (int)w << "x" << (int)h
              << "  a00=" << proj.a00 << " a11=" << proj.a11
              << "  fx=" << intr.fx << " fy=" << intr.fy
              << "  fx/fy=" << (std::abs(intr.fy) > 1e-6f ? intr.fx / intr.fy : 0.0f)
              << "\n");
    return intr;
}

// -----------------------------------------------------------------------------
// apply_curve_focal — THE single definition of the stale-focal workaround.
//
// cam->projection() does NOT reflect an ANIMATED focal when the camera's
// OutputContext is forced to a frame (solve paths) or served from the viewer
// cache (overlay): it returns a FROZEN focal. On a constant lens the stale value
// is coincidentally correct, which is why this was invisible until a zoom plate
// exposed it (sheared cube, flat a00, anchoring ray-miss, every-other-frame
// overlay flicker). See ZOOM_TAB_PLAN.md for the full diagnosis.
//
// The caller already holds the CameraOp, so it reads `focal`/`haperture` off the
// knobs at the target frame with get_value_at() (a curve read, no cook) and hands
// the values here. We rewrite the diagonal projection terms in this header's
// uniform-(w/2)-scale convention:
//     a00 = a11 = 2 * focal_mm / haperture
// (because tracker_intrinsics / build_intrinsics_from_projection map fx = |a00|*w/2
// and fx_px = focal_mm*w/haperture, so a00 = 2*fx_px/w = 2*focal_mm/haperture).
// The SIGN of each term is preserved (the GL overlay side carries NEGATIVE fx,
// the OpenCV side takes std::abs), and a02/a12 (principal point / lens shift) are
// left untouched. No-op for non-finite / non-positive inputs, so a STATIC lens is
// unchanged (the override equals what the cook already produced).
//
// Keeping this in ONE place mirrors the discipline of the convention bridge above:
// the focal map must agree across track_via_pins, the user-track anchoring, the
// focal fit, the flow track and the overlay, or a zoom solve silently diverges.
//
// NOTE: this addresses the FOCAL only. An animated camera POSITION/rotation would
// leave imatrix() stale the same way and needs its own curve read — that is the
// deferred "real camera" case (ZOOM_TAB_PLAN.md scope caveat).
inline void apply_curve_focal(DD::Image::Matrix4& proj,
                              double focal_mm, double haperture)
{
    if (std::isfinite(focal_mm) && focal_mm > 1e-6 &&
        std::isfinite(haperture) && haperture > 1e-6) {
        const float a = (float)(2.0 * focal_mm / haperture);
        proj.a00 = (proj.a00 < 0.0f) ? -a : a;
        proj.a11 = (proj.a11 < 0.0f) ? -a : a;
    }
}

// LOCAL-space embree mesh from a GeoMesh (model_matrix is object->world and is
// applied separately by RayCast / SolveFrame — do NOT bake it into the verts).
inline std::shared_ptr<AcceleratedMesh> build_local_accel_mesh(const GeoMesh& gm,
                                                               ArrayXu masked)
{
    RowMajorArrayX3f verts = gm.local_vertices;
    RowMajorArrayX3u tris  = gm.triangles;
    return std::make_shared<AcceleratedMesh>(
        std::move(verts), std::move(tris), std::move(masked));
}

}  // namespace conv
}  // namespace pcn

#endif  // PCN_CONVENTION_H
