// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Peter Mercell
//
// Developed with assistance from Claude (Anthropic).

// =============================================================================
// polychase_util.h  —  shared types + leaf helpers for the PolychaseTracker
// plugin. Everything lives in namespace `pcn`. Free functions are `inline` so
// the header can be included by multiple .cpp files without an ODR violation.
//
// Also defines PCN_LOG, the single gate for the plugin's diagnostic terminal
// output (see below). This header is pulled in by every translation unit via
// polychase_tracker.h, so the gate is visible everywhere.
// =============================================================================
#ifndef POLYCHASE_UTIL_H
#define POLYCHASE_UTIL_H

#include "DDImage/CameraOp.h"
#include "DDImage/GeoOp.h"
#include "DDImage/Scene.h"
#include "DDImage/GeometryList.h"
#include "DDImage/GeoInfo.h"
#include "DDImage/Primitive.h"
#include "DDImage/Matrix4.h"
#include "DDImage/Vector4.h"
#include "DDImage/OutputContext.h"
#include "DDImage/ViewerContext.h"

#ifdef PCN_NEW_3D
// New (USD/usg) 3D system. GeomOp + GeometryProviderI are in libDDImage; the
// usg data API (Stage/MeshPrim/PointBasedPrim/XformCache) is in libFnUsdAbstraction
// (linked when POLYCHASE_NEW_3D=ON). Headers live under ${NUKE_ROOT}/include.
// VERIFY ON BUILD: these paths/line refs come from the macOS SDK survey; if a
// header path differs on the Linux install, adjust here.
#include "DDImage/GeomOp.h"                 // GeomOp, Op::geomOp()
#include "DDImage/GeometryProviderI.h"      // getGeometryStage()
#include "usg/geom/Stage.h"                 // Stage::traverse() -> PrimRange
#include "usg/geom/Prim.h"                  // Prim::isA<T>()
#include "usg/geom/MeshPrim.h"              // getFaceVerts()
#include "usg/geom/PointBasedPrim.h"        // getPoints()
#include "usg/geom/XformCache.h"            // getLocalToWorldTransform()
#include "usg/base/ArrayTypes.h"            // Vec3fArray, Int32Array
#endif

#include "ray_casting.h"
#include "geometry.h"
#include "eigen_typedefs.h"
#include "pose.h"
#include "pin_mode.h"
#include "user_constraints.h"   // MaskPredicate, UserTracks (plugin-wide)

#include <Eigen/Core>

// Plugin-local helpers split out of this header:
//   - pose math (compose_trs / decompose_trs) — extracted so it can be unit-
//     tested without DDImage (test/test_pose_math.cpp). RowMajorMatrix4f here is
//     the project typedef from eigen_typedefs.h above, so tell the extracted
//     header NOT to define its own (PCN_POSE_MATH_EXTERNAL_TYPEDEFS).
//   - ScopedFlags  — RAII guard for the suppress_*_callback_ re-entrancy flags.
//   - BlobMirror   — shared cache/suppress/reload machinery for the hidden
//     String_knob blobs (pins / mask / anchors / live pose).
#define PCN_POSE_MATH_EXTERNAL_TYPEDEFS
#include "polychase_pose_math.h"
#include "pcn_scoped_guard.h"
#include "pcn_blob_mirror.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

// -----------------------------------------------------------------------------
// Debug logging — the one switch for all diagnostic terminal output.
//
// Every diagnostic print in the plugin goes through PCN_LOG instead of
// std::cout, so the terminal noise is silenced (and re-enabled) in one place.
// It is OFF by default. To restore the verbose per-frame / per-event prints to
// the terminal Nuke was launched from, either flip this to 1:
//
//     #define PCN_DEBUG 1
//
// or build with -DPCN_DEBUG=1 (the CMake option POLYCHASE_DEBUG_LOG does this).
//
// Usage is identical to a std::cout chain, minus the leading `std::cout <<`:
//     PCN_LOG("[solve] kept previous pose\n");
//     PCN_LOG("[solve] threw: " << e.what() << "\n");
// When PCN_DEBUG is 0 the macro expands to a no-op, so the strings and stream
// work cost nothing in a normal (silent) build.
// -----------------------------------------------------------------------------
#ifndef PCN_DEBUG
#define PCN_DEBUG 0
#endif

#if PCN_DEBUG
#define PCN_LOG(...) do { std::cout << __VA_ARGS__; } while (0)
#else
#define PCN_LOG(...) do {} while (0)
#endif

namespace pcn {

using namespace DD::Image;

// Input indices — single source of truth for what each pin means.
// kInputMask is the optional 2D occlusion mask (a Roto / any Iop with alpha).
enum InputIndex { kInputImg = 0, kInputCam = 1, kInputGeo = 2, kInputMask = 3 };

// -----------------------------------------------------------------------------
// Unit-cube mesh for the smoke test (kept in Diagnostics).
// -----------------------------------------------------------------------------
inline std::shared_ptr<AcceleratedMesh> build_cube_mesh()
{
    RowMajorArrayX3f vertices(8, 3);
    vertices <<
        -1.f, -1.f, -1.f,
         1.f, -1.f, -1.f,
        -1.f,  1.f, -1.f,
         1.f,  1.f, -1.f,
        -1.f, -1.f,  1.f,
         1.f, -1.f,  1.f,
        -1.f,  1.f,  1.f,
         1.f,  1.f,  1.f;

    RowMajorArrayX3u triangles(12, 3);
    triangles <<
        0u, 1u, 3u,  0u, 3u, 2u,
        4u, 6u, 7u,  4u, 7u, 5u,
        0u, 2u, 6u,  0u, 6u, 4u,
        1u, 5u, 7u,  1u, 7u, 3u,
        0u, 4u, 5u,  0u, 5u, 1u,
        2u, 3u, 7u,  2u, 7u, 6u;

    ArrayXu masked_triangles;
    return std::make_shared<AcceleratedMesh>(
        std::move(vertices), std::move(triangles), std::move(masked_triangles));
}

// =============================================================================
// Pin data model + geometry helpers.
// =============================================================================
//
// A "pin" anchors a 2D screen position to a fixed 3D point ON the mesh,
// expressed in mesh-local barycentric coordinates. Once placed, the pin sticks
// to that mesh point regardless of camera or object transform: each draw
// projects the current world position through the current view+projection,
// so animation, rotation, and per-object transforms automatically follow.
//
// User keyframe overrides (per-frame screen-position corrections) will be
// added alongside the FindTransformation solve.
// =============================================================================

struct Pin
{
    unsigned id            = 0;
    unsigned vertex_idx    = 0;     // global vertex index — the pin's IDENTITY
                                    // (invariant: one Pin per vertex_idx)
    int      created_frame = 0;     // diagnostic / future keyframe seed

    // ===== New pin model (Phase 1) ==========================================
    // A pin is a vertex with an optional linked user track and a 2D keyframe
    // curve. resolve_pin_2d() turns these into the pin's screen position at a
    // frame: manual key wins, else the linked track, else interpolation between
    // keys. keys map a frame -> 2D screen position in Nuke Y-UP pixels (same coord
    // system as target_x/y below and the wireframe glVertex2f calls).
    //
    // linked_track is the CURRENT anchored-set index of the linked user track, or
    // -1. It is volatile — re-resolved from linked_track_name every time the user-
    // track set is rebuilt (reresolve_pin_links), so reloading / reordering /
    // dropping tracks can't silently bind the wrong one. linked_track_name is the
    // STABLE id (the Tracker node's track name) and is what persists; if the name
    // no longer exists in the anchored set, linked_track resolves to -1 (the pin
    // shows unlinked) rather than pointing at a stale index.
    int                                       linked_track = -1;
    std::string                               linked_track_name;   // stable link id
    std::map<int, Eigen::Vector2f>            keys;
    // ========================================================================

    // -------- Drag-to-reposition (LEGACY — kept until the pin solve moves to
    // resolve_pin_2d in a later phase; creation/drag still write these so the
    // existing run_pin_solve keeps working unchanged). --------
    // When the user drags a pin, target_x_px/target_y_px hold the screen
    // position they're asking the vertex to project to. The solver minimizes the
    // distance between this target and the projection of vertex_idx.
    //
    // Convention: image-pixel coords (Y up, origin bottom-left of format),
    // same coord system as the wireframe's glVertex2f calls.
    float    target_x_px        = 0.0f;
    float    target_y_px        = 0.0f;
    bool     is_target_user_set = false;
};

// -----------------------------------------------------------------------------
// Pin serialization — persistence + undo support.
//
// New-model format (Phase 1), pipe-separated entries, each entry SEMICOLON-
// separated so it cannot be confused with the old comma format:
//
//   id;vertex;linked_track;cf;tx;ty;us;<keys>
//
// where <keys> is a comma-separated list of "frame:x:y" (Nuke Y-up px), possibly
// empty. cf/tx/ty/us are the transitional legacy fields (still written so the
// existing pin solve works until it moves to resolve_pin_2d).
//
// Old-format blobs (comma-separated, no ';') simply won't parse → pins start
// empty, which is the intended clean break for the new pin architecture.
//
// Stored in a hidden String_knob so Nuke's .nk save/load and undo handle
// persistence automatically.
// -----------------------------------------------------------------------------
inline std::string serialize_pin_list(const std::vector<Pin>& pins)
{
    std::ostringstream oss;
    oss.precision(8);
    for (size_t i = 0; i < pins.size(); ++i) {
        if (i > 0) oss << '|';
        oss << pins[i].id            << ';'
            << pins[i].vertex_idx    << ';'
            << pins[i].linked_track  << ';'
            << pins[i].created_frame << ';'
            << pins[i].target_x_px   << ';'
            << pins[i].target_y_px   << ';'
            << (pins[i].is_target_user_set ? 1 : 0) << ';';
        bool first = true;
        for (const auto& kv : pins[i].keys) {
            if (!first) oss << ',';
            oss << kv.first << ':' << kv.second.x() << ':' << kv.second.y();
            first = false;
        }
        // Field 8: the stable link id (Tracker track name). Sanitize the field
        // separators ';' and '|' out of the name so they can't corrupt the blob.
        std::string nm = pins[i].linked_track_name;
        for (char& c : nm) if (c == ';' || c == '|') c = '_';
        oss << ';' << nm;
    }
    return oss.str();
}

inline std::vector<Pin> deserialize_pin_list(const char* s)
{
    std::vector<Pin> out;
    if (!s || !*s) return out;

    auto split = [](const std::string& in, char sep) {
        std::vector<std::string> v;
        size_t p = 0;
        while (p <= in.size()) {
            size_t c = in.find(sep, p);
            if (c == std::string::npos) { v.push_back(in.substr(p)); break; }
            v.push_back(in.substr(p, c - p));
            p = c + 1;
        }
        return v;
    };

    for (const std::string& entry : split(std::string(s), '|')) {
        // New format only — an entry with no ';' is an old-format pin: skip it.
        if (entry.find(';') == std::string::npos) continue;
        const std::vector<std::string> f = split(entry, ';');
        if (f.size() < 7) continue;
        try {
            Pin pin;
            pin.id                 = (unsigned)std::stoul(f[0]);
            pin.vertex_idx         = (unsigned)std::stoul(f[1]);
            pin.linked_track       = std::stoi(f[2]);
            pin.created_frame      = std::stoi(f[3]);
            pin.target_x_px        = std::stof(f[4]);
            pin.target_y_px        = std::stof(f[5]);
            pin.is_target_user_set = (std::stoi(f[6]) != 0);
            if (f.size() >= 8 && !f[7].empty()) {
                for (const std::string& k : split(f[7], ',')) {
                    const std::vector<std::string> kp = split(k, ':');
                    if (kp.size() != 3) continue;
                    const int   fr = std::stoi(kp[0]);
                    const float x  = std::stof(kp[1]);
                    const float y  = std::stof(kp[2]);
                    pin.keys[fr] = Eigen::Vector2f(x, y);
                }
            }
            // Field 8: stable link id (track name). Absent in older blobs.
            if (f.size() >= 9) pin.linked_track_name = f[8];
            out.push_back(std::move(pin));
        } catch (...) {
            // skip malformed entry
        }
    }
    return out;
}

inline unsigned compute_next_pin_id(const std::vector<Pin>& pins)
{
    unsigned max_id_plus_one = 0;
    for (const auto& p : pins) {
        if (p.id + 1u > max_id_plus_one) max_id_plus_one = p.id + 1u;
    }
    return max_id_plus_one;
}

// -----------------------------------------------------------------------------
// GL state coord helpers.
//
// The 2D viewer applies its own pan+zoom transform via OpenGL's modelview
// and projection matrices. To convert between image-pixel coords (what we
// pass to glVertex2f) and viewer-widget screen coords (what mouse_x/mouse_y
// return), we capture the GL state during DRAW_OPAQUE and re-apply it later.
// -----------------------------------------------------------------------------

// Project image-pixel coords through cached GL state to viewer screen pixels
// (GL convention: Y up, origin at bottom-left of viewport).
inline bool project_image_to_screen_gl(const double mv[16],
                                       const double pj[16],
                                       const int    vp[4],
                                       double img_x, double img_y,
                                       double& sx, double& sy)
{
    // MV * (img_x, img_y, 0, 1) → eye (column-major matrices)
    const double ex = mv[0]*img_x + mv[4]*img_y + mv[12];
    const double ey = mv[1]*img_x + mv[5]*img_y + mv[13];
    const double ez = mv[2]*img_x + mv[6]*img_y + mv[14];
    const double ew = mv[3]*img_x + mv[7]*img_y + mv[15];
    // P * eye → clip
    const double cx_ = pj[0]*ex + pj[4]*ey + pj[8]*ez + pj[12]*ew;
    const double cy_ = pj[1]*ex + pj[5]*ey + pj[9]*ez + pj[13]*ew;
    const double cw_ = pj[3]*ex + pj[7]*ey + pj[11]*ez + pj[15]*ew;
    if (std::abs(cw_) < 1e-9) return false;
    const double ndc_x = cx_ / cw_;
    const double ndc_y = cy_ / cw_;
    sx = vp[0] + (ndc_x + 1.0) * 0.5 * vp[2];
    sy = vp[1] + (ndc_y + 1.0) * 0.5 * vp[3];
    return true;
}

// -----------------------------------------------------------------------------
// Nuke ↔ Eigen Matrix4 conversion.
//
// Nuke's DD::Image::Matrix4 is column-major in storage but exposes elements
// via named members a00..a33 where `a_rc` is the element at row r, col c.
// (operator[](int) returns a column pointer, not a single float, so it's
// awkward for individual-element loops — named members are clearer here.)
//
// The mathematical matrix is identical between Nuke Matrix4 and Eigen
// RowMajorMatrix4f; only storage layout differs.
// -----------------------------------------------------------------------------
inline RowMajorMatrix4f nuke_to_eigen_m4(const DD::Image::Matrix4& m)
{
    RowMajorMatrix4f e;
    e << m.a00, m.a01, m.a02, m.a03,
         m.a10, m.a11, m.a12, m.a13,
         m.a20, m.a21, m.a22, m.a23,
         m.a30, m.a31, m.a32, m.a33;
    return e;
}

inline DD::Image::Matrix4 eigen_to_nuke_m4(const RowMajorMatrix4f& e)
{
    DD::Image::Matrix4 m;
    m.a00 = e(0,0); m.a01 = e(0,1); m.a02 = e(0,2); m.a03 = e(0,3);
    m.a10 = e(1,0); m.a11 = e(1,1); m.a12 = e(1,2); m.a13 = e(1,3);
    m.a20 = e(2,0); m.a21 = e(2,1); m.a22 = e(2,2); m.a23 = e(2,3);
    m.a30 = e(3,0); m.a31 = e(3,1); m.a32 = e(3,2); m.a33 = e(3,3);
    return m;
}

// -----------------------------------------------------------------------------
// Translate / Rotate(deg, XYZ: R = Rx*Ry*Rz) / Scale  <->  row-major 4x4.
// Used to store the solved object pose as animated translate/rotate/scale knobs
// (so it keyframes on the timeline and persists in the .nk). The compose/
// decompose pair round-trips exactly (test/test_pose_math.cpp verifies it over a
// grid of poses incl. gimbal lock).
//
// MOVED: the bodies now live in polychase_pose_math.h (included above) so they
// can be unit-tested without dragging in DDImage. compose_trs / decompose_trs
// remain in namespace pcn and are unchanged at every call site.
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// CameraIntrinsics from Nuke projection matrix.
//
// Derive polychase OpenGL-convention intrinsics straight from Nuke's
// cam->projection() so the solve uses the SAME image mapping the wireframe
// rendering uses. If they disagree, solver output won't align with what the
// user sees on screen and pins slide off the wireframe as you drag.
//
// CRITICAL: the wireframe / pin draw maps NDC -> pixels with a UNIFORM scale
// on BOTH axes (scale = fmt_w * 0.5), centered at (fmt_w/2, fmt_h/2):
//     px = ndc_x * (fmt_w/2) + fmt_w/2
//     py = ndc_y * (fmt_w/2) + fmt_h/2     <-- Y also uses fmt_w/2
// So the intrinsics must reproduce exactly that mapping. polychase's
// Project(X) = (fx*X.x/X.z + cx, fy*X.y/X.z + cy) with OpenGL z<0 in front
// gives, after matching term-by-term:
//     fx = -a00 * fmt_w/2,   fy = -a11 * fmt_w/2     (same scale -> square px)
//     cx =  fmt_w/2,         cy =  fmt_h/2
//
// Using fmt_h*0.5 for fy (the earlier bug) squashed the solver's vertical
// metric by fmt_w/fmt_h (~1.9x for 2K_DCP): X aligned, Y did not, so the
// dragged pin couldn't reach its vertex and the fit drifted vertically.
//
// Nuke projection matrix (OpenGL convention, no lens shift):
//   a00 = 2 * |f_px_x| / format_width    (positive)
//   a11 = 2 * |f_px_y| / format_height   (positive)
//
// Lens shift (Nuke win_translate) IS now honoured: a02/a12 are the projection's
// NDC principal-point offset, so cx = half_w*(1 - a02), cy = h/2 - a12*half_w
// (OpenGL/y-up). A centred camera (a02=a12=0) reduces to fmt_w/2, fmt_h/2 —
// identical to before. The solver-side builders (tracker_track/refine) use the
// y-down form cy = h/2 + a12*half_w. Export inverts this to write win_translate.
// -----------------------------------------------------------------------------
inline CameraIntrinsics build_intrinsics_from_projection(
    const DD::Image::Matrix4& proj,
    float fmt_w, float fmt_h)
{
    const float ndc_per_unit_x = proj.a00;
    const float ndc_per_unit_y = proj.a11;

    // Uniform NDC->pixel scale, matching the wireframe/pin draw path.
    const float scale = fmt_w * 0.5f;

    CameraIntrinsics intr;
    intr.fx = -ndc_per_unit_x * scale;
    intr.fy = -ndc_per_unit_y * scale;   // was fmt_h*0.5f — caused the vertical drift
    intr.cx = scale * (1.0f - proj.a02);
    intr.cy = fmt_h * 0.5f - proj.a12 * scale;
    // aspect_ratio is the fx/fy linkage, used only when optimize_focal_length
    // is true (we pass false). With the uniform scale this is a00/a11 (~1 for
    // square pixels); set it consistently anyway.
    intr.aspect_ratio = (std::abs(intr.fy) > 1e-6f) ? (intr.fx / intr.fy) : 1.0f;
    intr.width  = fmt_w;
    intr.height = fmt_h;
    intr.convention = CameraConvention::OpenGL;
    return intr;
}
//
// Approach: probe two known image-pixel points (0,0) and (1,1) through the
// forward transform, derive the inverse 2D affine. Assumes the viewer
// applies an axis-aligned affine — true for Nuke 2D viewer.
inline bool mouse_to_image_pixel(const double mv[16],
                                 const double pj[16],
                                 const int    vp[4],
                                 int mouse_x_nuke,
                                 int mouse_y_nuke_topdown,
                                 double& img_x, double& img_y)
{
    // Flip Y to GL convention (0 at bottom)
    const double sy_gl = (double)vp[3] - (double)mouse_y_nuke_topdown;
    const double sx    = (double)mouse_x_nuke;

    // Probe image (0,0) and (1,1) to get scale + offset of the viewer transform
    double s0x, s0y, s1x, s1y;
    if (!project_image_to_screen_gl(mv, pj, vp, 0.0, 0.0, s0x, s0y)) return false;
    if (!project_image_to_screen_gl(mv, pj, vp, 1.0, 1.0, s1x, s1y)) return false;
    const double dx = s1x - s0x;
    const double dy = s1y - s0y;
    if (std::abs(dx) < 1e-9 || std::abs(dy) < 1e-9) return false;
    img_x = (sx    - s0x) / dx;
    img_y = (sy_gl - s0y) / dy;
    return true;
}

// -----------------------------------------------------------------------------
// GeoMesh — local-space triangulation of the first object in a GeoOp's output.
//
// Only the first object is handled. Multi-object scenes need a future
// extension (add object_idx to Pin, walk all objects). The polychase test
// data is single-mesh so this restriction is fine for now.
// -----------------------------------------------------------------------------
struct GeoMesh
{
    RowMajorArrayX3f     local_vertices;     // (N, 3) — LOCAL coords
    RowMajorArrayX3u     triangles;          // (M, 3) — vertex indices
    DD::Image::Matrix4   object_to_world;    // info.matrix at extraction
    bool                 valid = false;
};

// -----------------------------------------------------------------------------
// Triangulate the FIRST object of a GeoOp's current output.
//
// For each primitive face with N vertices, emits (N-2) triangles via a fan
// from vertex 0 — produces ([0,1,2], [0,2,3], ...). This is the same
// triangulation that polychase's mesh format expects.
//
// Caller must have already called geo->validate(true).
// -----------------------------------------------------------------------------
inline bool extract_first_object_mesh(DD::Image::GeoOp* geo, GeoMesh& out)
{
    using namespace DD::Image;
    out.valid = false;
    if (!geo) return false;

    Scene scene;
    GeometryList geos;
    geo->get_geometry(scene, geos);
    if (geos.objects() == 0) return false;

    const GeoInfo& info = geos[0];
    const PointList* pts = info.point_list();
    if (!pts) return false;

    const unsigned nverts = (unsigned)pts->size();
    if (nverts == 0) return false;

    // Copy LOCAL vertex positions. We apply info.matrix at projection time
    // (in draw) and at AcceleratedMesh build time (in click). Storing local
    // means the GeoMesh is stable across transform changes — only the
    // separately-tracked object_to_world differs.
    out.local_vertices.resize(nverts, 3);
    for (unsigned i = 0; i < nverts; ++i) {
        const Vector3& v = (*pts)[i];
        out.local_vertices(i, 0) = v.x;
        out.local_vertices(i, 1) = v.y;
        out.local_vertices(i, 2) = v.z;
    }

    // Fan-triangulate each primitive face.
    std::vector<std::array<unsigned, 3>> tri_buf;
    tri_buf.reserve(64);

    const unsigned num_prims = info.primitives();
    for (unsigned p = 0; p < num_prims; ++p) {
        const Primitive* prim = info.primitive(p);
        if (!prim) continue;
        const unsigned num_faces = prim->faces();
        for (unsigned f = 0; f < num_faces; ++f) {
            const unsigned nv = prim->face_vertices((int)f);
            if (nv < 3) continue;

            unsigned small_buf[32];
            std::vector<unsigned> heap_buf;
            unsigned* face_idx = small_buf;
            if (nv > 32) {
                heap_buf.resize(nv);
                face_idx = heap_buf.data();
            }
            prim->get_face_vertices((int)f, face_idx);

            const unsigned v0 = prim->vertex(face_idx[0]);
            for (unsigned i = 1; i + 1 < nv; ++i) {
                const unsigned v1 = prim->vertex(face_idx[i]);
                const unsigned v2 = prim->vertex(face_idx[i + 1]);
                tri_buf.push_back({v0, v1, v2});
            }
        }
    }
    if (tri_buf.empty()) return false;

    out.triangles.resize((Eigen::Index)tri_buf.size(), 3);
    for (size_t i = 0; i < tri_buf.size(); ++i) {
        out.triangles((Eigen::Index)i, 0) = tri_buf[i][0];
        out.triangles((Eigen::Index)i, 1) = tri_buf[i][1];
        out.triangles((Eigen::Index)i, 2) = tri_buf[i][2];
    }

    out.object_to_world = info.matrix;
    out.valid = true;
    return true;
}

#ifdef PCN_NEW_3D
// -----------------------------------------------------------------------------
// mat4d_to_Matrix4 — convert Foundry's double 4x4 (fdk::Mat4d, as returned by
// usg::XformCache::getLocalToWorldTransform) to DD::Image::Matrix4 (float).
//
// Uses the unambiguous a{Row}{Col} members (a02 = row0,col2 — same convention the
// projection code already relies on). fdk::Mat4d indexes as m[i][j]; the ONE thing
// to verify is whether that's [row][col] (assumed) or [col][row]. If
// object_to_world comes out TRANSPOSED (wireframe mirrored / mesh inside-out),
// swap to m[c][r] below. Round-trip one known GeoCube transform to confirm.
// -----------------------------------------------------------------------------
inline DD::Image::Matrix4 mat4d_to_Matrix4(const fdk::Mat4d& m)
{
    DD::Image::Matrix4 out;
    out.a00=(float)m[0][0]; out.a01=(float)m[0][1]; out.a02=(float)m[0][2]; out.a03=(float)m[0][3];
    out.a10=(float)m[1][0]; out.a11=(float)m[1][1]; out.a12=(float)m[1][2]; out.a13=(float)m[1][3];
    out.a20=(float)m[2][0]; out.a21=(float)m[2][1]; out.a22=(float)m[2][2]; out.a23=(float)m[2][3];
    out.a30=(float)m[3][0]; out.a31=(float)m[3][1]; out.a32=(float)m[3][2]; out.a33=(float)m[3][3];
    return out;
}

// -----------------------------------------------------------------------------
// Extract the first MeshPrim from a new-system geometry op's usg::Stage into the
// SAME GeoMesh layout the classic path produces: LOCAL points + fan triangles +
// object_to_world. `time` is the consumer's current frame (fdk::TimeValue=double);
// for a static rigid proxy it's frame-invariant.
//
// Mirrors extract_first_object_mesh's fan: base = face vertex 0, emit
// (base, v[i], v[i+1]). Honors the prim's clockwise flag so winding matches the
// classic path (matters for back-face-culled tint + raycast normals).
//
// VERIFY ON BUILD: exact usg method signatures/line refs are from the SDK survey
// (MeshPrim.h:143 getFaceVerts; PointBasedPrim.h:68 getPoints; XformCache.h:23).
// -----------------------------------------------------------------------------
inline bool extract_mesh_usg(DD::Image::GeometryProviderI* gp, double time, GeoMesh& out)
{
    out.valid = false;
    if (!gp) return false;

    usg::StageRef stage = gp->getGeometryStage();        // GeometryProviderI.h:192
    if (!stage) return false;

    usg::XformCache xc(time);                             // XformCache(fdk::TimeValue)
    for (usg::Prim prim : stage->traverse()) {           // Stage.h:178
        if (!prim.isA<usg::MeshPrim>()) continue;        // Prim.h:139
        usg::MeshPrim mesh(prim);                         // MeshPrim.h:59

        usg::Vec3fArray P = mesh.getPoints(time);         // PointBasedPrim.h:68
        const size_t nverts = P.size();
        if (nverts == 0) continue;

        usg::Int32Array counts, idx;
        bool cw = false;
        mesh.getFaceVerts(time, counts, idx, &cw);        // MeshPrim.h:143
        if (counts.size() == 0 || idx.size() == 0) continue;

        // LOCAL vertices (object_to_world applied separately, like the classic path).
        out.local_vertices.resize((Eigen::Index)nverts, 3);
        for (size_t i = 0; i < nverts; ++i) {
            out.local_vertices((Eigen::Index)i, 0) = P[i].x;
            out.local_vertices((Eigen::Index)i, 1) = P[i].y;
            out.local_vertices((Eigen::Index)i, 2) = P[i].z;
        }

        // Fan-triangulate. `idx` is the flat per-face-vertex point-index stream;
        // walk it face by face using `counts`.
        std::vector<std::array<unsigned, 3>> tri_buf;
        tri_buf.reserve(idx.size());
        size_t off = 0;
        for (size_t f = 0; f < counts.size(); ++f) {
            const int nv = counts[f];
            if (nv >= 3 && off + (size_t)nv <= idx.size()) {
                const unsigned v0 = (unsigned)idx[off];
                for (int i = 1; i + 1 < nv; ++i) {
                    const unsigned a = (unsigned)idx[off + i];
                    const unsigned b = (unsigned)idx[off + i + 1];
                    // cw -> reverse so the emitted winding matches the classic
                    // (CCW) fan; keeps tint back-face culling + ray normals correct.
                    if (cw) tri_buf.push_back({v0, b, a});
                    else    tri_buf.push_back({v0, a, b});
                }
            }
            off += (size_t)(nv > 0 ? nv : 0);
        }
        if (tri_buf.empty()) continue;

        out.triangles.resize((Eigen::Index)tri_buf.size(), 3);
        for (size_t i = 0; i < tri_buf.size(); ++i) {
            out.triangles((Eigen::Index)i, 0) = tri_buf[i][0];
            out.triangles((Eigen::Index)i, 1) = tri_buf[i][1];
            out.triangles((Eigen::Index)i, 2) = tri_buf[i][2];
        }

        out.object_to_world = mat4d_to_Matrix4(xc.getLocalToWorldTransform(prim));  // XformCache.h:23
        out.valid = true;
        return true;   // first mesh only — matches classic single-object behaviour
    }
    return false;
}
#endif // PCN_NEW_3D

// -----------------------------------------------------------------------------
// extract_mesh — front door. Dispatches to the classic GeoOp path or the new
// usg path based on the connected op, returning the identical GeoMesh either way.
// A node is a GeoOp (classic) or a GeomOp (new), never both, so order is moot.
// Caller passes the raw geo input Op* (PolychaseTracker::input_geo_op()).
// -----------------------------------------------------------------------------
inline bool extract_mesh(DD::Image::Op* in, GeoMesh& out)
{
    out.valid = false;
    if (!in) return false;
#ifdef PCN_NEW_3D
    if (DD::Image::GeomOp* gm = in->geomOp()) {          // Op.h:2230 — new geometry only
        const double t = in->outputContext().frame();
        return extract_mesh_usg(gm->asGeometryProvider(), t, out);  // GeomOp.h:555
    }
#endif
    if (DD::Image::GeoOp* g = in->geoOp())               // Op.h:2226 — classic
        return extract_first_object_mesh(g, out);
    return false;
}

// -----------------------------------------------------------------------------
// Build a world-space AcceleratedMesh from a GeoMesh.
// Used for ray-casting against the current geo at click time. Embree BVH
// construction happens inside the AcceleratedMesh ctor.
//
// Currently unused after the switch to vertex-snap pins — kept
// for future surface-anchored pins or drag-to-reposition workflows where we
// need real ray intersection.
// -----------------------------------------------------------------------------
[[maybe_unused]]
inline std::shared_ptr<AcceleratedMesh> build_accelerated_world_mesh(const GeoMesh& gm)
{
    using namespace DD::Image;
    if (!gm.valid) return nullptr;

    const Eigen::Index nverts = gm.local_vertices.rows();
    RowMajorArrayX3f world_verts(nverts, 3);
    for (Eigen::Index i = 0; i < nverts; ++i) {
        const Vector4 vl(gm.local_vertices(i, 0),
                         gm.local_vertices(i, 1),
                         gm.local_vertices(i, 2),
                         1.0f);
        const Vector4 vw = gm.object_to_world * vl;
        world_verts(i, 0) = vw.x;
        world_verts(i, 1) = vw.y;
        world_verts(i, 2) = vw.z;
    }
    RowMajorArrayX3u tris = gm.triangles;
    ArrayXu masked;
    return std::make_shared<AcceleratedMesh>(
        std::move(world_verts), std::move(tris), std::move(masked));
}

// -----------------------------------------------------------------------------
// pixel_to_world_ray — given a click in image-pixel coords, build the
// world-space ray that hits that pixel.
//
// Approach (avoids needing Matrix4::inverse): extract the projection's
// X/Y scale factors by applying it to direction-only vectors —
//   proj * (1,0,0,0)  →  (M00, 0, 0, 0)   gives M00
//   proj * (0,1,0,0)  →  (0, M11, 0, 0)   gives M11
// In camera space a ray through pixel (px, py) at z=-1 has
//   cam = (ndc_x / M00, ndc_y / M11, -1)
// where ndc_{x,y} match the forward draw's mapping. Transform to world by
// multiplying through camera->matrix() (camera-to-world).
//
// Currently unused after the switch to vertex-snap pins — kept
// for future surface-anchored pins or drag-to-reposition workflows.
// -----------------------------------------------------------------------------
[[maybe_unused]]
inline void pixel_to_world_ray(DD::Image::CameraOp* cam,
                        float px, float py,
                        float fmt_w, float fmt_h, float scale,
                        DD::Image::Vector3& ray_origin_world,
                        DD::Image::Vector3& ray_dir_world)
{
    using namespace DD::Image;
    const Matrix4 proj = cam->projection();
    const Vector4 col_x = proj * Vector4(1.0f, 0.0f, 0.0f, 0.0f);
    const Vector4 col_y = proj * Vector4(0.0f, 1.0f, 0.0f, 0.0f);
    const float M00 = col_x.x;
    const float M11 = col_y.y;
    if (std::abs(M00) < 1e-6f || std::abs(M11) < 1e-6f) {
        ray_origin_world = Vector3(0.0f, 0.0f, 0.0f);
        ray_dir_world    = Vector3(0.0f, 0.0f, -1.0f);
        return;
    }

    // Pixel → NDC (matches forward draw: px = ndc * scale + fmt_w*0.5)
    const float ndc_x = (px - fmt_w * 0.5f) / scale;
    const float ndc_y = (py - fmt_h * 0.5f) / scale;

    // Camera-space ray direction at z = -1
    const Vector3 cam_dir(ndc_x / M00, ndc_y / M11, -1.0f);

    // Transform to world: rotation only for direction (w=0), full for origin (w=1).
    const Matrix4 cam_to_world = cam->matrix();
    const Vector4 d4 = cam_to_world * Vector4(cam_dir.x, cam_dir.y, cam_dir.z, 0.0f);
    Vector3 dir_w(d4.x, d4.y, d4.z);
    const float len = std::sqrt(dir_w.x*dir_w.x + dir_w.y*dir_w.y + dir_w.z*dir_w.z);
    if (len > 1e-6f) {
        dir_w.x /= len;
        dir_w.y /= len;
        dir_w.z /= len;
    }
    ray_dir_world = dir_w;

    const Vector4 o4 = cam_to_world * Vector4(0.0f, 0.0f, 0.0f, 1.0f);
    ray_origin_world = Vector3(o4.x, o4.y, o4.z);
}

// -----------------------------------------------------------------------------
// Compute barycentric coords (u, v) of a 3D point on a triangle.
// Returns weights such that  p ≈ (1-u-v)*v0 + u*v1 + v*v2.
//
// Currently unused after the switch to vertex-snap pins — kept
// for future surface-anchored pins or drag-to-reposition workflows.
// -----------------------------------------------------------------------------
[[maybe_unused]]
inline void compute_barycentric(const Eigen::Vector3f& p,
                         const Eigen::Vector3f& v0,
                         const Eigen::Vector3f& v1,
                         const Eigen::Vector3f& v2,
                         float& bary_u, float& bary_v)
{
    const Eigen::Vector3f e1 = v1 - v0;
    const Eigen::Vector3f e2 = v2 - v0;
    const Eigen::Vector3f n  = e1.cross(e2);
    const float n_sq = n.dot(n);
    if (n_sq < 1e-12f) {
        bary_u = 0.0f;
        bary_v = 0.0f;
        return;
    }
    const Eigen::Vector3f pv = p - v0;
    bary_u = (e2.cross(pv)).dot(n) / n_sq;
    bary_v = (pv.cross(e1)).dot(n) / n_sq;
}


} // namespace pcn

#endif // POLYCHASE_UTIL_H
