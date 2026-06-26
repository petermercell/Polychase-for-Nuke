// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Peter Mercell
//
// Developed with assistance from Claude (Anthropic).

// tracker_gizmo.cpp — pose gizmo overlay + interaction.
//
// Draws a transform gizmo (proxy-axis handles + a dolly handle + a centre
// marker) over the plate in the 2D viewer, anchored at the proxy origin and
// oriented by the proxy, and handles grabbing/dragging those handles to move the
// pose. Rotation is numerical (the Rotate X/Y/Z knobs), so there is no viewer
// rotation ring. This file also owns the plugin's own viewer-gesture undo
// (the move-history stack) and the offset<->pose maps shared with the pin solve.
//
// Projection matches the wireframe exactly (same view_proj + uniform NDC->pixel
// scale/cx/cy), so the gizmo sits on the proxy.
//
// Header order: polychase_tracker.h (pulls DDImage's glew) before <GL/gl.h>.
#include "polychase_tracker.h"

// Nuke 17's NDK removed the global DD::Image::Undo::disable()/enable() (the
// DDImage/Undo.h header is gone and the symbols are no longer exported). The
// supported replacement is the per-knob Knob::undoless(bool) toggle — see the
// UndoSuspend helper below, which uses it to tick the Translate/Dolly fields
// live during a gizmo drag while still recording the whole gesture as a single
// undo step on release.
#include <GL/gl.h>
#include <initializer_list>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

using namespace DD::Image;

namespace pcn {

namespace {

struct V3 { float x, y, z; };

// RAII: suspend Nuke's undo recording on a set of knobs for its lifetime, via the
// per-knob Knob::undoless() toggle (the Nuke-17 replacement for the removed global
// DD::Image::Undo::disable()). Used to tick the Translate/Dolly knobs live during a
// gizmo drag without dropping one undo entry per mouse-move — on_gizmo_release then
// records the whole gesture as a single step. Null knob pointers are skipped, and
// undoless(false) is only restored on the knobs we actually toggled, so it stays
// balanced.
struct UndoSuspend {
    DD::Image::Knob* k_[8] = {nullptr, nullptr, nullptr, nullptr,
                              nullptr, nullptr, nullptr, nullptr};
    int n_ = 0;
    UndoSuspend(std::initializer_list<DD::Image::Knob*> ks) {
        for (DD::Image::Knob* k : ks)
            if (k && n_ < 8) { k_[n_++] = k; k->undoless(true); }
    }
    ~UndoSuspend() { for (int i = 0; i < n_; ++i) k_[i]->undoless(false); }
    UndoSuspend(const UndoSuspend&)            = delete;
    UndoSuspend& operator=(const UndoSuspend&) = delete;
};

inline V3 sub(const V3& a, const V3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3 add(const V3& a, const V3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3 mul(const V3& a, float s)     { return {a.x * s, a.y * s, a.z * s}; }
inline float len(const V3& a)           { return std::sqrt(a.x*a.x + a.y*a.y + a.z*a.z); }
inline V3 norm(const V3& a) {
    const float n = len(a);
    return (n > 1e-9f) ? V3{a.x/n, a.y/n, a.z/n} : V3{0, 0, 0};
}
inline V3 cross(const V3& a, const V3& b) {
    return {a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x};
}
inline float dot(const V3& a, const V3& b) { return a.x*b.x + a.y*b.y + a.z*b.z; }

// Map a screen point (relative to centre cxs,cys, radius r) onto a trackball
// sphere: inside the disc -> sphere cap (z toward viewer), outside -> equator.
inline V3 map_sphere(float mx, float my, float cxs, float cys, float r) {
    const float x = (mx - cxs) / r, y = (my - cys) / r;
    const float d2 = x*x + y*y;
    if (d2 < 1.0f) return V3{x, y, std::sqrt(1.0f - d2)};
    const float inv = 1.0f / std::sqrt(d2);
    return V3{x*inv, y*inv, 0.0f};
}

// Rodrigues axis-angle -> Nuke Matrix4 (row-major a_RC; applied as M*v).
inline Matrix4 axis_angle_m4(const V3& ax, float ang) {
    const float c = std::cos(ang), s = std::sin(ang), t = 1.0f - c;
    const float x = ax.x, y = ax.y, z = ax.z;
    Matrix4 m; m.makeIdentity();
    m.a00 = t*x*x + c;   m.a01 = t*x*y - s*z; m.a02 = t*x*z + s*y;
    m.a10 = t*x*y + s*z; m.a11 = t*y*y + c;   m.a12 = t*y*z - s*x;
    m.a20 = t*x*z - s*y; m.a21 = t*y*z + s*x; m.a22 = t*z*z + c;
    return m;
}
inline Matrix4 transl_m4(float x, float y, float z) {
    Matrix4 m; m.makeIdentity();
    m.a03 = x; m.a13 = y; m.a23 = z;
    return m;
}

// World point through obj_to_world (point: w=1).
inline V3 xf_point(const Matrix4& m, float x, float y, float z) {
    const Vector4 c = m * Vector4(x, y, z, 1.0f);
    return {c.x, c.y, c.z};
}
// World direction through a matrix (vector: w=0, ignores translation).
inline V3 xf_dir(const Matrix4& m, float x, float y, float z) {
    const Vector4 c = m * Vector4(x, y, z, 0.0f);
    return {c.x, c.y, c.z};
}

// Shared gizmo geometry (world space), used by BOTH draw and hit-test so the
// handles you see are exactly the handles you can grab. L is the world length
// that projects to `gizmo_px` pixels at the proxy's depth.
struct GizmoGeom {
    V3   O;
    V3   axisDir[3];   // unit world axes (X,Y,Z) of the proxy
    V3   axisEnd[3];   // O + L*axisDir
    V3   dolly;        // O + 0.55*L*camU  (depth handle, screen-up of centre)
    V3   camR, camU;
    float L  = 0.0f;
    bool  ok = false;
};

// Project world -> image-px (scale only; cx/cy cancel in length differences).
inline bool proj_imgpx(const Matrix4& vp, float scale, const V3& w,
                       float& px, float& py) {
    const Vector4 c = vp * Vector4(w.x, w.y, w.z, 1.0f);
    if (c.w <= 0.001f) return false;
    px = (c.x / c.w) * scale;
    py = (c.y / c.w) * scale;
    return true;
}

// Local-space bounding-box centre of a mesh — the natural pivot ("middle of
// the wireframe"), independent of where the modeller put the local origin.
inline V3 mesh_local_center(const GeoMesh& gm) {
    const Eigen::Index n = gm.local_vertices.rows();
    if (n <= 0) return {0.0f, 0.0f, 0.0f};
    float mnx = 1e30f, mny = 1e30f, mnz = 1e30f;
    float mxx = -1e30f, mxy = -1e30f, mxz = -1e30f;
    for (Eigen::Index i = 0; i < n; ++i) {
        const float x = gm.local_vertices(i, 0);
        const float y = gm.local_vertices(i, 1);
        const float z = gm.local_vertices(i, 2);
        mnx = std::min(mnx, x); mny = std::min(mny, y); mnz = std::min(mnz, z);
        mxx = std::max(mxx, x); mxy = std::max(mxy, y); mxz = std::max(mxz, z);
    }
    return { 0.5f*(mnx+mxx), 0.5f*(mny+mxy), 0.5f*(mnz+mxz) };
}

inline GizmoGeom compute_geom(const Matrix4& model, const Matrix4& c2w,
                              const Matrix4& view_proj, float scale,
                              double gizmo_px, const V3& local_center) {
    GizmoGeom g;
    g.O          = xf_point(model, local_center.x, local_center.y, local_center.z);
    g.axisDir[0] = norm(xf_dir(model, 1, 0, 0));
    g.axisDir[1] = norm(xf_dir(model, 0, 1, 0));
    g.axisDir[2] = norm(xf_dir(model, 0, 0, 1));
    g.camR       = norm(xf_dir(c2w, 1, 0, 0));
    g.camU       = norm(xf_dir(c2w, 0, 1, 0));
    float ox, oy, qx, qy;
    if (!proj_imgpx(view_proj, scale, g.O, ox, oy)) return g;       // ok=false
    if (!proj_imgpx(view_proj, scale, add(g.O, g.camR), qx, qy)) return g;
    const float probe = std::sqrt((qx - ox) * (qx - ox) + (qy - oy) * (qy - oy));
    if (probe < 1e-3f) return g;
    g.L = (float)gizmo_px * (1.0f / probe);
    for (int i = 0; i < 3; ++i) g.axisEnd[i] = add(g.O, mul(g.axisDir[i], g.L));
    g.dolly = add(g.O, mul(g.camU, 0.55f * g.L));
    g.ok = true;
    return g;
}

// ---------------------------------------------------------------------------
// Unified offset pose model (Eigen). One base = the upstream entry-point pose U.
// The persistent offsets are: world translate T, dolly (signed distance along the
// camera->centroid ray), and rotation R (deg, Rx*Ry*Rz) ABOUT the geo centroid.
// compose_offset_pose() is the forward map (offsets -> pose); decompose_offset_
// pose() is its exact inverse (pose -> T,R with dolly folded into T) so a pin
// solve can be read straight back into the knobs. Both share the codebase's
// compose_trs/decompose_trs rotation convention, which round-trips exactly.
// ---------------------------------------------------------------------------
inline RowMajorMatrix4f transl_eigen(const Eigen::Vector3f& t) {
    RowMajorMatrix4f M = RowMajorMatrix4f::Identity();
    M(0, 3) = t.x(); M(1, 3) = t.y(); M(2, 3) = t.z();
    return M;
}

inline RowMajorMatrix4f compose_offset_pose(
    const RowMajorMatrix4f& U, const V3& lc, const Eigen::Vector3f& cam_pos,
    const double T[3], double dolly, const double R[3])
{
    const Eigen::Vector4f lc4(lc.x, lc.y, lc.z, 1.0f);
    const Eigen::Vector3f O0 = (U * lc4).head<3>();             // world centroid of U

    const double zero_t[3] = {0.0, 0.0, 0.0}, ones[3] = {1.0, 1.0, 1.0};
    const RowMajorMatrix4f Rmat = compose_trs(zero_t, R, ones); // rotation-only
    const RowMajorMatrix4f Arot = transl_eigen(O0) * Rmat * transl_eigen(-O0);
    const RowMajorMatrix4f Mrot = Arot * U;                     // rotate U about O0

    const Eigen::Vector3f Tw((float)T[0], (float)T[1], (float)T[2]);
    // Dolly axis = the CURRENT-centroid -> camera ray (current centroid = upstream
    // centroid + translate; rotation is about the centroid so it doesn't move it).
    // This is the same pivot the gizmo dolly uses in the viewer. The decompose side
    // recovers the ray from (O0 + Tfull), which is colinear with this through the
    // camera, so the forward/inverse split stays exact.
    Eigen::Vector3f ray = (O0 + Tw) - cam_pos;
    const float rl = ray.norm();
    ray = (rl > 1e-6f) ? (ray / rl) : Eigen::Vector3f(0.f, 0.f, 1.f);
    const Eigen::Vector3f Ttot = Tw + (float)dolly * ray;      // dolly slides along the ray

    return transl_eigen(Ttot) * Mrot;
}

inline void decompose_offset_pose(
    const RowMajorMatrix4f& working, const RowMajorMatrix4f& U, const V3& lc,
    const Eigen::Vector3f& cam_pos, double T_out[3], double& dolly_out, double R_out[3])
{
    // Rotation: working_3x3 = R * U_3x3  =>  R = working_3x3 * U_3x3^-1.
    const Eigen::Matrix3f Wr = working.block<3, 3>(0, 0);
    const Eigen::Matrix3f Ur = U.block<3, 3>(0, 0);
    const Eigen::Matrix3f Rm = Wr * Ur.inverse();
    RowMajorMatrix4f Rmat4 = RowMajorMatrix4f::Identity();
    Rmat4.block<3, 3>(0, 0) = Rm;
    double t_dummy[3], s_dummy[3];
    decompose_trs(Rmat4, t_dummy, R_out, s_dummy);

    // Full world translation: working.col3 = (Arot*U).col3 + Ttot  =>  Ttot = diff.
    const Eigen::Vector4f lc4(lc.x, lc.y, lc.z, 1.0f);
    const Eigen::Vector3f O0 = (U * lc4).head<3>();
    const RowMajorMatrix4f Arot = transl_eigen(O0) * Rmat4 * transl_eigen(-O0);
    const RowMajorMatrix4f Mrot = Arot * U;
    const Eigen::Vector3f Tfull((float)working(0, 3) - (float)Mrot(0, 3),
                                (float)working(1, 3) - (float)Mrot(1, 3),
                                (float)working(2, 3) - (float)Mrot(2, 3));

    // Split Ttot into dolly (component along the CURRENT-centroid->camera ray) and
    // the perpendicular world translate — the exact inverse of compose's split.
    // (O0 + Tfull) is the current centroid; it's colinear with compose's (O0 + T)
    // through the camera, so this ray equals the one compose used.
    Eigen::Vector3f ray = (O0 + Tfull) - cam_pos;
    const float rl = ray.norm();
    ray = (rl > 1e-6f) ? (ray / rl) : Eigen::Vector3f(0.f, 0.f, 1.f);
    dolly_out = (double)Tfull.dot(ray);
    const Eigen::Vector3f Tperp = Tfull - (float)dolly_out * ray;
    T_out[0] = Tperp.x(); T_out[1] = Tperp.y(); T_out[2] = Tperp.z();
}

}  // namespace

void PolychaseTracker::draw_gizmo(const DD::Image::Matrix4& view_proj,
                                  const DD::Image::Matrix4& obj_to_world,
                                  DD::Image::CameraOp* cam,
                                  float scale, float cx, float cy,
                                  const DD::Image::Vector3& pivot_local) const
{
    if (manipulator_mode_ == 1) return;   // legacy "Pins-only" mode (now unused)
    if (!cam) return;

    // The gizmo is ALWAYS drawn, but while pin-edit is armed
    // it is inert (hit-tests no-op) and drawn dimmed so the brightened pins read
    // as the active editor. dimA scales every alpha below.
    const float dimA = pin_input_active() ? 0.32f : 1.0f;

    // World->image-pixel projection, identical mapping to the wireframe.
    auto project = [&](const V3& w, float& px, float& py) -> bool {
        const Vector4 clip = view_proj * Vector4(w.x, w.y, w.z, 1.0f);
        if (clip.w <= 0.001f) return false;       // behind camera
        px = (clip.x / clip.w) * scale + cx;
        py = (clip.y / clip.w) * scale + cy;
        return true;
    };

    // Proxy + camera geometry (shared with the hit-test). Pivot at the mesh centre.
    const Matrix4 c2w = cam->matrix();   // camera-to-world
    const V3 lc = { pivot_local.x, pivot_local.y, pivot_local.z };
    const GizmoGeom g = compute_geom(obj_to_world, c2w, view_proj, scale, gizmo_size_px_, lc);
    if (!g.ok) return;

    float ox, oy;
    if (!project(g.O, ox, oy)) return;           // centre marker (image px)

    // ------------------------------------------------------------------ GL ---
    glPushAttrib(GL_LINE_BIT | GL_CURRENT_BIT | GL_ENABLE_BIT | GL_POINT_BIT
                 | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_DEPTH_TEST);
    glLineWidth(2.0f);

    auto line_world = [&](const V3& a, const V3& b) {
        float ax, ay, bx, by;
        if (project(a, ax, ay) && project(b, bx, by)) {
            glVertex2f(ax, ay);
            glVertex2f(bx, by);
        }
    };

    auto handle_square = [&](const V3& w, float cr, float cg, float cb) {
        float hx, hy;
        if (!project(w, hx, hy)) return;
        glLineWidth(1.0f);
        glColor4f(cr, cg, cb, 0.8f * dimA);          // stalk from centre
        glBegin(GL_LINES); glVertex2f(ox, oy); glVertex2f(hx, hy); glEnd();
        const float hs = 5.0f;
        glColor4f(cr, cg, cb, 1.0f * dimA);          // filled square
        glBegin(GL_QUADS);
        glVertex2f(hx - hs, hy - hs); glVertex2f(hx + hs, hy - hs);
        glVertex2f(hx + hs, hy + hs); glVertex2f(hx - hs, hy + hs);
        glEnd();
    };

    // --- Axis lines (translate group). ---
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glColor4f(1.00f, 0.25f, 0.25f, dimA);  line_world(g.O, g.axisEnd[0]);  // X
    glColor4f(0.30f, 1.00f, 0.30f, dimA);  line_world(g.O, g.axisEnd[1]);  // Y
    glColor4f(0.35f, 0.55f, 1.00f, dimA);  line_world(g.O, g.axisEnd[2]);  // Z
    glEnd();

    // --- Dolly handle (depth group). ---
    handle_square(g.dolly, 0.30f, 0.85f, 0.95f);   // cyan

    // Rotation is now numerical (Rotate X/Y/Z knobs, about the mesh centre) —
    // no viewer ring handle.

    // --- Centre marker (screen-plane translate, part of the translate group). ---
    glPointSize(8.0f);
    glEnable(GL_POINT_SMOOTH);
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f * dimA);
    glBegin(GL_POINTS); glVertex2f(ox, oy); glEnd();

    glPopAttrib();
}

// =============================================================================
// Interaction: screen-plane translate (grab the centre handle).
// =============================================================================
namespace {

// Resolve the image format dimensions + NDC->pixel scale, matching the draw.
inline void format_scale(DD::Image::Iop* img, float& fmt_w, float& fmt_h, float& scale) {
    fmt_w = 2048.0f; fmt_h = 1080.0f;
    if (img) {
        const DD::Image::Format& fmt = img->info().format();
        fmt_w = (float)fmt.width();
        fmt_h = (float)fmt.height();
    }
    scale = fmt_w * 0.5f;
}

// Ray/plane intersection. Returns false if the ray is parallel to the plane.
inline bool ray_plane(const Vector3& ro, const Vector3& rd,
                      const Vector3& p0, const Vector3& n, Vector3& hit) {
    const float denom = n.x*rd.x + n.y*rd.y + n.z*rd.z;
    if (std::abs(denom) < 1e-7f) return false;
    const V3 d = { p0.x - ro.x, p0.y - ro.y, p0.z - ro.z };
    const float t = (n.x*d.x + n.y*d.y + n.z*d.z) / denom;
    hit = Vector3(ro.x + rd.x*t, ro.y + rd.y*t, ro.z + rd.z*t);
    return true;
}

// 2D distance from point p to segment a-b (screen px).
inline float pt_seg_dist(float px, float py, float ax, float ay,
                         float bx, float by) {
    const float vx = bx - ax, vy = by - ay;
    const float wx = px - ax, wy = py - ay;
    const float vv = vx*vx + vy*vy;
    float t = (vv > 1e-6f) ? (wx*vx + wy*vy) / vv : 0.0f;
    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
    const float qx = ax + t*vx, qy = ay + t*vy;
    const float dx = px - qx, dy = py - qy;
    return std::sqrt(dx*dx + dy*dy);
}

}  // namespace

// Hit-test all gizmo handles. Returns: 4 dolly, 0 centre, 1/2/3 X/Y/Z axis, -1 none.
int PolychaseTracker::gizmo_hit_test(DD::Image::ViewerContext* ctx,
                                     const double mv[16], const double pj[16],
                                     const int vp[4])
{
    if (pin_input_active()) return -1;         // pins armed → gizmo inert
    if (manipulator_mode_ == 1) return -1;   // legacy Pins-only mode (unused)
    CameraOp* cam = input_cam();
    Op*       geo = input_geo_op();
    Iop*      img = input_img();
    if (!cam || !geo) return -1;
    cam->validate(true); geo->validate(true); if (img) img->validate(true);

    GeoMesh gm;
    if (!extract_mesh(geo, gm)) return -1;

    float fmt_w, fmt_h, scale; format_scale(img, fmt_w, fmt_h, scale);
    const float cx = fmt_w * 0.5f, cy = fmt_h * 0.5f;

    const Matrix4 c2w       = cam->matrix();
    const Matrix4 view_proj = cam->projection() * cam->imatrix();
    const Matrix4 model     = effective_model_matrix(gm.object_to_world,
                                                     (double)editing_frame());
    const GizmoGeom g = compute_geom(model, c2w, view_proj, scale, gizmo_size_px_,
                                     mesh_local_center(gm));
    if (!g.ok) return -1;

    // World → viewer screen px (GL Y-up), matching find_pin_under_cursor.
    auto to_screen = [&](const V3& w, double& sx, double& sy) -> bool {
        const Vector4 c = view_proj * Vector4(w.x, w.y, w.z, 1.0f);
        if (c.w <= 0.001f) return false;
        const double ipx = (double)((c.x / c.w) * scale + cx);
        const double ipy = (double)((c.y / c.w) * scale + cy);
        return project_image_to_screen_gl(mv, pj, vp, ipx, ipy, sx, sy);
    };

    const float mx = (float)ctx->mouse_x();
    const float my = (float)(vp[3] - ctx->mouse_y());   // GL Y-up

    double osx, osy;
    if (!to_screen(g.O, osx, osy)) return -1;

    // 1) Dolly handle (depth) — distinct square, may overlap the Y axis.
    {
        double dsx, dsy;
        if (to_screen(g.dolly, dsx, dsy)) {
            const float dx = (float)dsx - mx, dy = (float)dsy - my;
            if (dx*dx + dy*dy <= 12.0f * 12.0f) return 4;
        }
    }

    // 2) Centre (screen-plane translate) — the inner disc.
    {
        const float dx = (float)osx - mx, dy = (float)osy - my;
        if (dx*dx + dy*dy <= 14.0f * 14.0f) return 0;
    }

    // 3) Axis lines (translate) — nearest within tolerance, skipping edge-on.
    {
        int   best_axis = -1;
        float best_d    = 8.0f;   // px tolerance
        for (int i = 0; i < 3; ++i) {
            double esx, esy;
            if (!to_screen(g.axisEnd[i], esx, esy)) continue;
            const float seg_len = std::sqrt((float)((esx-osx)*(esx-osx) + (esy-osy)*(esy-osy)));
            if (seg_len < 10.0f) continue;   // edge-on: not reliably grabbable
            const float d = pt_seg_dist(mx, my, (float)osx, (float)osy, (float)esx, (float)esy);
            if (d < best_d) { best_d = d; best_axis = i; }
        }
        if (best_axis >= 0) return 1 + best_axis;
    }

    // Rotation handle removed — rotation is numerical (Rotate X/Y/Z knobs).
    return -1;
}

bool PolychaseTracker::on_gizmo_push(DD::Image::ViewerContext* ctx,
                                     const double mv[16], const double pj[16],
                                     const int vp[4])
{
    if (pin_input_active()) return false;      // pins armed → gizmo inert
    const int handle = gizmo_hit_test(ctx, mv, pj, vp);
    if (handle < 0) { gizmo_active_handle_ = -1; return false; }   // clear stale grab

    CameraOp* cam = input_cam();
    Op*       geo = input_geo_op();
    Iop*      img = input_img();
    if (!cam || !geo) return false;

    GeoMesh gm;
    if (!extract_mesh(geo, gm)) return false;

    float fmt_w, fmt_h, scale; format_scale(img, fmt_w, fmt_h, scale);
    const float cx = fmt_w * 0.5f, cy = fmt_h * 0.5f;

    const Matrix4 c2w       = cam->matrix();
    const Matrix4 view_proj = cam->projection() * cam->imatrix();

    // Persistent offsets: do NOT bake/zero on grab. The drag adds onto whatever
    // offsets are already dialed in, and the grab snapshot below is the pre-drag
    // baseline the release uses to record ONE undo step. The single shared base is
    // the upstream entry-point pose (recompute_pose_from_offsets), so translate,
    // dolly and rotate all compose together — they no longer fight, so there's
    // nothing to bake.
    auto offval = [&](const char* nm) { Knob* k = knob(nm); return k ? k->get_value() : 0.0; };
    gizmo_grab_off_[0] = offval("trans_x");
    gizmo_grab_off_[1] = offval("trans_y");
    gizmo_grab_off_[2] = offval("trans_z");
    gizmo_grab_off_[3] = offval("dolly");
    gizmo_grab_off_[4] = offval("rot_x");
    gizmo_grab_off_[5] = offval("rot_y");
    gizmo_grab_off_[6] = offval("rot_z");

    // Freeze the starting pose. The pivot (gizmo_plane_origin_) is the mesh
    // bounding-box centre in world — NOT the local origin — so rotation/dolly
    // happen about the middle of the wireframe.
    gizmo_start_model_  = effective_model_matrix(gm.object_to_world,
                                                     (double)editing_frame());

    const V3 lc = mesh_local_center(gm);
    const GizmoGeom g = compute_geom(gizmo_start_model_, c2w, view_proj, scale,
                                     gizmo_size_px_, lc);
    if (!g.ok) return false;

    gizmo_plane_origin_ = Vector3(g.O.x, g.O.y, g.O.z);   // pivot = mesh centre

    // Freeze the translate/dolly base for this gesture: trans_base_ is the pose
    // WITHOUT any offset (the offset knobs are 0 here — reset above), and
    // trans_base_had_live_ records whether a live pose existed at grab so a
    // return-to-0 reverts correctly. The drag fills the offset knobs on RELEASE
    // and undo replays them through preview_translate_offset(), which reads the
    // SAME trans_base_ — that's what keeps the undo reconstruction exact.
    trans_base_          = gizmo_start_model_;
    trans_base_had_live_ = (bool)live_scene_;
    trans_base_dolly_dir_ = Vector3(0, 0, 1);   // overwritten by the dolly branch below

    // Cursor in GL screen px (for axis/dolly screen-delta math).
    gizmo_grab_mx_ = (float)ctx->mouse_x();
    gizmo_grab_my_ = (float)(vp[3] - ctx->mouse_y());

    if (handle == 0) {
        // --- Screen-plane translate: ray/plane intersect at the proxy depth. ---
        const Vector4 fwd = c2w * Vector4(0.0f, 0.0f, -1.0f, 0.0f);
        Vector3 n(fwd.x, fwd.y, fwd.z);
        const float ln = std::sqrt(n.x*n.x + n.y*n.y + n.z*n.z);
        if (ln > 1e-9f) { n.x/=ln; n.y/=ln; n.z/=ln; }
        gizmo_plane_normal_ = n;

        double img_x = 0.0, img_y = 0.0;
        if (!mouse_to_image_pixel(mv, pj, vp, ctx->mouse_x(), ctx->mouse_y(), img_x, img_y))
            return false;
        Vector3 ro, rd;
        pixel_to_world_ray(cam, (float)img_x, (float)img_y, fmt_w, fmt_h, scale, ro, rd);
        if (!ray_plane(ro, rd, gizmo_plane_origin_, gizmo_plane_normal_, gizmo_grab_hit_))
            return false;
    }
    else if (handle >= 1 && handle <= 3) {
        // --- Axis-constrained translate: freeze axis + its screen direction. ---
        const int i = handle - 1;
        gizmo_axis_dir_ = Vector3(g.axisDir[i].x, g.axisDir[i].y, g.axisDir[i].z);

        // Screen px per 1 world unit along the axis (+ unit screen direction).
        auto to_screen = [&](const V3& w, double& sx, double& sy) -> bool {
            const Vector4 c = view_proj * Vector4(w.x, w.y, w.z, 1.0f);
            if (c.w <= 0.001f) return false;
            const double ipx = (double)((c.x / c.w) * scale + cx);
            const double ipy = (double)((c.y / c.w) * scale + cy);
            return project_image_to_screen_gl(mv, pj, vp, ipx, ipy, sx, sy);
        };
        double osx, osy, asx, asy;
        const V3 a_unit = add(g.O, g.axisDir[i]);   // O + 1 world unit along axis
        if (!to_screen(g.O, osx, osy) || !to_screen(a_unit, asx, asy)) return false;
        const float sdx = (float)(asx - osx), sdy = (float)(asy - osy);
        const float slen = std::sqrt(sdx*sdx + sdy*sdy);
        if (slen < 1e-3f) return false;             // axis pointing at camera
        gizmo_px_per_world_  = slen;
        gizmo_screen_dir_[0] = sdx / slen;
        gizmo_screen_dir_[1] = sdy / slen;
    }
    else if (handle == 4) {
        // --- Dolly: freeze camera centre + the O0-to-camera vector. ---
        const Vector4 c0 = c2w * Vector4(0.0f, 0.0f, 0.0f, 1.0f);
        gizmo_cam_center_ = Vector3(c0.x, c0.y, c0.z);
        gizmo_dolly_v_    = Vector3(gizmo_plane_origin_.x - c0.x,
                                    gizmo_plane_origin_.y - c0.y,
                                    gizmo_plane_origin_.z - c0.z);
        // Unit viewing-ray dir the dolly slides along — the basis the `dolly`
        // knob is measured in, frozen so the knob/undo reconstruction matches.
        const float dl = std::sqrt(gizmo_dolly_v_.x*gizmo_dolly_v_.x +
                                   gizmo_dolly_v_.y*gizmo_dolly_v_.y +
                                   gizmo_dolly_v_.z*gizmo_dolly_v_.z);
        if (dl > 1e-9f)
            trans_base_dolly_dir_ = Vector3(gizmo_dolly_v_.x/dl,
                                            gizmo_dolly_v_.y/dl,
                                            gizmo_dolly_v_.z/dl);
    }

    gizmo_active_handle_ = handle;
    gizmo_moved_         = false;   // no movement yet this gesture

    // Move-history: record the pre-move state and arm capture for this session.
    arm_and_seed_move_baseline();

    asapUpdate();
    return true;
}

// Record the current (pre-operation) pose offsets + pin set as a history baseline
// and arm capture for this session. Called at the start of a gizmo grab and a pin
// grab so the FIRST move/pin has a "before" to diff against and undo back to. For
// a fresh history we seed; for a loaded/divergent one we append the grab-from
// state (dropping any redo branch) so Undo returns exactly where the grab started.
void PolychaseTracker::arm_and_seed_move_baseline()
{
    double cur[7];
    if (!read_offset_values(cur)) { move_hist_armed_ = true; return; }
    const std::string cur_pins = serialize_pin_list(pins_);

    std::vector<MoveSnapshot> hist; int pos;
    load_move_history(hist, pos);
    if (hist.empty()) {
        MoveSnapshot s; std::copy(cur, cur + 7, s.v); s.frame = editing_frame(); s.pins = cur_pins; capture_tracked_extras(s);
        hist.push_back(s);
        store_move_history(hist, 0);
        PCN_LOG("[PolychaseTracker] move-history baseline seeded @f" << s.frame << "\n");
    } else {
        bool same = (pos >= 0 && pos < (int)hist.size() && cur_pins == hist[pos].pins);
        if (same)
            for (int i = 0; i < 7; ++i)
                if (std::fabs(cur[i] - hist[pos].v[i]) > 1e-6) { same = false; break; }
        if (!same) {
            if (pos + 1 < (int)hist.size()) hist.resize(pos + 1);
            MoveSnapshot s; std::copy(cur, cur + 7, s.v); s.frame = editing_frame(); s.pins = cur_pins; capture_tracked_extras(s);
            hist.push_back(s);
            store_move_history(hist, (int)hist.size() - 1);
            PCN_LOG("[PolychaseTracker] move-history resume baseline @f" << s.frame << "\n");
        }
    }
    move_hist_armed_ = true;
}

void PolychaseTracker::on_gizmo_drag(DD::Image::ViewerContext* ctx,
                                     const double mv[16], const double pj[16],
                                     const int vp[4])
{
    if (gizmo_active_handle_ < 0) return;
    CameraOp* cam = input_cam();
    Iop*      img = input_img();
    if (!cam) return;

    float fmt_w, fmt_h, scale; format_scale(img, fmt_w, fmt_h, scale);
    const float mx = (float)ctx->mouse_x();
    const float my = (float)(vp[3] - ctx->mouse_y());   // GL Y-up

    Matrix4 m = gizmo_start_model_;   // frozen pose; we rewrite its translation

    if (gizmo_active_handle_ == 0) {
        // --- Screen-plane translate: cursor ray ∩ proxy-depth plane. ---
        double img_x = 0.0, img_y = 0.0;
        if (!mouse_to_image_pixel(mv, pj, vp, ctx->mouse_x(), ctx->mouse_y(), img_x, img_y))
            return;
        Vector3 ro, rd;
        pixel_to_world_ray(cam, (float)img_x, (float)img_y, fmt_w, fmt_h, scale, ro, rd);
        Vector3 hit;
        if (!ray_plane(ro, rd, gizmo_plane_origin_, gizmo_plane_normal_, hit)) return;
        m.a03 += (hit.x - gizmo_grab_hit_.x);
        m.a13 += (hit.y - gizmo_grab_hit_.y);
        m.a23 += (hit.z - gizmo_grab_hit_.z);
    }
    else if (gizmo_active_handle_ >= 1 && gizmo_active_handle_ <= 3) {
        // --- Axis-constrained translate: project screen drag onto the axis.
        // Displacement is added to the START translation (not the pivot). ---
        const float ddx = mx - gizmo_grab_mx_;
        const float ddy = my - gizmo_grab_my_;
        const float along_px    = ddx*gizmo_screen_dir_[0] + ddy*gizmo_screen_dir_[1];
        const float world_along = along_px / gizmo_px_per_world_;
        m.a03 = gizmo_start_model_.a03 + gizmo_axis_dir_.x * world_along;
        m.a13 = gizmo_start_model_.a13 + gizmo_axis_dir_.y * world_along;
        m.a23 = gizmo_start_model_.a23 + gizmo_axis_dir_.z * world_along;
    }
    else if (gizmo_active_handle_ == 4) {
        // --- Dolly: slide the pivot along the camera ray; apply that world
        // delta to the start translation. Drag up = closer / bigger. ---
        const float ddy = my - gizmo_grab_my_;
        const float k = std::exp(-ddy * 0.006f);
        const float newOx = gizmo_cam_center_.x + gizmo_dolly_v_.x * k;
        const float newOy = gizmo_cam_center_.y + gizmo_dolly_v_.y * k;
        const float newOz = gizmo_cam_center_.z + gizmo_dolly_v_.z * k;
        m.a03 = gizmo_start_model_.a03 + (newOx - gizmo_plane_origin_.x);
        m.a13 = gizmo_start_model_.a13 + (newOy - gizmo_plane_origin_.y);
        m.a23 = gizmo_start_model_.a23 + (newOz - gizmo_plane_origin_.z);
    }
    else {
        return;
    }

    // Decompose the dragged world pose straight into the unified offset knobs
    // (translate + rotate; a gizmo move never changes rotation, and dolly folds
    // into translate — the dolly KNOB is reserved for typed dolly input). This is
    // the SAME inverse map the pin solve uses, so every pose source reads back
    // into the same knobs. Live tick under undo suspension (one undo on release);
    // the displayed value lives in the Knob, so set_value is required to move it.
    {
        DD::Image::Knob* ktx = knob("trans_x");
        DD::Image::Knob* kty = knob("trans_y");
        DD::Image::Knob* ktz = knob("trans_z");
        DD::Image::Knob* krx = knob("rot_x");
        DD::Image::Knob* kry = knob("rot_y");
        DD::Image::Knob* krz = knob("rot_z");
        DD::Image::Knob* kd  = knob("dolly");

        GeoMesh gmd;
        if (input_geo_op() && extract_mesh(input_geo_op(), gmd)) {
            const RowMajorMatrix4f U = nuke_to_eigen_m4(gmd.object_to_world);
            const V3 lc = mesh_local_center(gmd);
            Eigen::Vector3f cam_pos(0.f, 0.f, 0.f);
            if (CameraOp* cam = input_cam()) {
                cam->validate(true);
                const Vector4 c = cam->matrix() * Vector4(0.f, 0.f, 0.f, 1.f);
                cam_pos = Eigen::Vector3f(c.x, c.y, c.z);
            }
            double T[3], R[3], dolly = 0.0;
            decompose_offset_pose(nuke_to_eigen_m4(m), U, lc, cam_pos, T, dolly, R);

            {
                ScopedFlags guard(suppress_trans_callback_, suppress_rot_callback_);
                UndoSuspend no_undo{ktx, kty, ktz, krx, kry, krz, kd};
                if (ktx) ktx->set_value(T[0]);
                if (kty) kty->set_value(T[1]);
                if (ktz) ktz->set_value(T[2]);
                if (kd)  kd->set_value(dolly);
                if (krx) krx->set_value(R[0]);
                if (kry) kry->set_value(R[1]);
                if (krz) krz->set_value(R[2]);
            }
        }
    }

    // Publish as the live pose. Only model_matrix is read by the overlay/keying;
    // carry any existing view/intrinsics forward if a live solve already exists.
    SceneTransformations st{};
    if (live_scene_) st = *live_scene_;
    st.model_matrix = nuke_to_eigen_m4(m);
    live_scene_ = st;
    live_edit_frame_ = editing_frame();
    gizmo_moved_ = true;   // a real move happened → on release it becomes undoable

    asapUpdate();
}

void PolychaseTracker::on_gizmo_release()
{
    if (gizmo_active_handle_ < 0) return;
    gizmo_active_handle_ = -1;
    // The drag set the offset knobs + live_scene_. Capture is handled by
    // ensure_move_history() on the next redraw (it watches the shared offset knobs
    // and records once the gizmo handle is released), so there's nothing to push
    // here. Republish live_scene_ so the keyed-overlay path stays in sync.
    if (gizmo_moved_) {
        gizmo_moved_ = false;
        sync_blob_from_live_pose();
    }
    asapUpdate();
}

// ---------------------------------------------------------------------------
// Move history (our own undo for viewer gestures), backed by the shared
// move_hist_blob knob so it survives the viewer/panel Op-instance split. We track
// the seven SHARED offset knob values (Translate XYZ, Dolly, Rotate XYZ) rather
// than any in-memory pose: those numbers are what change on a gizmo move and are
// identical on every Op instance. Serialized form: first line "pos <cursor>",
// then one snapshot per line ("<frame> v0 v1 v2 v3 v4 v5 v6").
//
// ensure_move_history() is the capture point — it runs every redraw (from the
// wireframe draw pass) and appends a snapshot whenever the offsets settle to a
// value different from the current history entry. Comparing against the SHARED
// cursor (hist[pos]) — not an instance-local "previous" — is what makes it
// self-consistent across instances and stops an Undo restore from re-recording
// itself. restore_move_state() writes the seven knobs back (same path a typed
// edit uses) and republishes live_scene_ via live_pose_blob so the viewer
// instance rebuilds the wireframe.
// ---------------------------------------------------------------------------
namespace {
const char* const kOffsetKnobNames[7] =
    { "trans_x", "trans_y", "trans_z", "dolly", "rot_x", "rot_y", "rot_z" };

// Ordinary scalar knobs tracked in the move history (Int/Double/Bool/Enumeration
// all read & write as a double). The Database path is tracked separately as a
// string. Keep this list in sync with the NO_UNDO flags and the knob_changed
// routing in PolychaseTracker.cpp. Reordering/extending is safe: snapshots store
// the count and restore applies the overlap, so an older blob still loads.
const char* const kExtraKnobNames[] = {
    "first_frame", "last_frame", "gizmo_size", "wire_gradient",
    "solve_mode"
};
const int kNumExtraKnobs = (int)(sizeof(kExtraKnobNames) / sizeof(kExtraKnobNames[0]));

// The pin set is an opaque blob that may contain spaces/newlines, so we hex-encode
// it as the snapshot's last field — guaranteed free of our line/space delimiters.
std::string hex_encode(const std::string& in)
{
    static const char* H = "0123456789abcdef";
    std::string out;
    out.reserve(in.size() * 2);
    for (unsigned char c : in) { out += H[c >> 4]; out += H[c & 0xF]; }
    return out;
}
std::string hex_decode(const std::string& in)
{
    auto nyb = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        return -1;
    };
    std::string out;
    out.reserve(in.size() / 2);
    for (size_t i = 0; i + 1 < in.size(); i += 2) {
        int hi = nyb(in[i]), lo = nyb(in[i + 1]);
        if (hi < 0 || lo < 0) break;
        out += static_cast<char>((hi << 4) | lo);
    }
    return out;
}

// Serialized snapshot line (all fields space-separated, one line per snapshot):
//   <frame> v0..v6 <pinsHexOr-> <r> <g> <b> <nExtra> <e0..e_{n-1}> <dbHexOr->
// Everything from <r> onward is optional for backward compatibility: pre-colour
// blobs stop after the pins field, the colour-only version stops after <b>.
std::string serialize_snapshot(const double v[7], int frame, const std::string& pins,
                               const float color[3], const std::vector<double>& extra,
                               const std::string& db_path)
{
    std::ostringstream o;
    o << frame << std::setprecision(9);
    for (int i = 0; i < 7; ++i) o << ' ' << v[i];
    o << ' ' << (pins.empty() ? "-" : hex_encode(pins));   // "-" = no pins
    o << ' ' << color[0] << ' ' << color[1] << ' ' << color[2];
    o << ' ' << extra.size();
    for (double e : extra) o << ' ' << e;
    o << ' ' << (db_path.empty() ? "-" : hex_encode(db_path));
    return o.str();
}
bool parse_snapshot(const std::string& line, double v[7], int& frame, std::string& pins,
                    float color[3], std::vector<double>& extra, std::string& db_path)
{
    std::istringstream in(line);
    in >> frame;
    for (int i = 0; i < 7; ++i) in >> v[i];
    if (in.fail()) return false;
    pins.clear();
    std::string hx;
    if (in >> hx && hx != "-") pins = hex_decode(hx);   // absent (old format) → no pins
    // Colour is optional: snapshots written before the wireframe-colour feature
    // have no trailing r g b, so leave the caller's default (green-cyan) when absent.
    float r, g, b;
    if (in >> r >> g >> b) { color[0] = r; color[1] = g; color[2] = b; }
    // Extra scalar knobs + db path are optional too (absent before this feature).
    // We store the count exactly as written so the round-trip is stable even if the
    // code's kNumExtraKnobs later changes; restore applies the overlap.
    extra.clear();
    db_path.clear();
    int n = 0;
    if (in >> n && n >= 0 && n < 4096) {
        extra.resize((size_t)n);
        for (int i = 0; i < n; ++i) in >> extra[i];
        std::string dbhx;
        if (in >> dbhx && dbhx != "-") db_path = hex_decode(dbhx);
    }
    return true;
}
bool offsets_equal(const double a[7], const double b[7])
{
    for (int i = 0; i < 7; ++i)
        if (std::fabs(a[i] - b[i]) > 1e-6) return false;
    return true;
}
bool color_equal(const float a[3], const float b[3])
{
    for (int i = 0; i < 3; ++i)
        if (std::fabs(a[i] - b[i]) > 1e-6f) return false;
    return true;
}
} // namespace

bool PolychaseTracker::read_offset_values(double out[7])
{
    for (int i = 0; i < 7; ++i) {
        DD::Image::Knob* k = knob(kOffsetKnobNames[i]);
        if (!k) return false;
        out[i] = k->get_value();
    }
    return true;
}

void PolychaseTracker::load_move_history(std::vector<MoveSnapshot>& hist, int& pos)
{
    // In-memory is authoritative for the session. Parse the shared blob only once,
    // on a fresh instance (e.g. the panel/button instance picking up what the
    // viewer instance recorded), never on every redraw — the bound member lags our
    // own writes and would hand back a stale stack.
    if (!move_hist_loaded_) {
        move_hist_.clear();
        move_pos_ = -1;
        const char* blob = move_hist_blob_ ? move_hist_blob_ : "";
        std::istringstream in(blob);
        std::string line;
        bool header = true;
        while (std::getline(in, line)) {
            if (line.empty()) continue;
            if (header) {
                header = false;
                std::istringstream h(line);
                std::string tag;
                h >> tag >> move_pos_;
                if (tag != "pos") { move_pos_ = -1; move_hist_.clear(); break; }
                continue;
            }
            MoveSnapshot s;
            if (parse_snapshot(line, s.v, s.frame, s.pins, s.color, s.extra, s.db_path))
                move_hist_.push_back(s);
        }
        if (move_pos_ >= (int)move_hist_.size()) move_pos_ = (int)move_hist_.size() - 1;
        move_hist_cache_ = blob;
        move_hist_loaded_ = true;
    }
    hist = move_hist_;
    pos  = move_pos_;
}

void PolychaseTracker::store_move_history(const std::vector<MoveSnapshot>& hist, int pos)
{
    // Update the authoritative in-memory copy, then mirror to the shared blob so a
    // different Op instance (panel button) and persistence can see it.
    move_hist_   = hist;
    move_pos_    = pos;
    move_hist_loaded_ = true;

    // Keep committed_ pointing at the cursor entry's tracked-extra state. This is the
    // single choke point every history mutation passes through (record, coalesce,
    // undo, redo, baseline seed), so maintaining it here keeps the "before" value
    // for the next ordinary-knob edit correct without each caller having to.
    if (pos >= 0 && pos < (int)hist.size()) committed_ = hist[pos];
    else                                    committed_ = MoveSnapshot{};

    std::ostringstream o;
    o << "pos " << pos << '\n';
    for (const auto& s : hist)
        o << serialize_snapshot(s.v, s.frame, s.pins, s.color, s.extra, s.db_path) << '\n';
    const std::string s = o.str();
    if (s != move_hist_cache_) {
        if (DD::Image::Knob* k = knob("move_hist_blob")) k->set_text(s.c_str());
        move_hist_cache_ = s;
    }
}

void PolychaseTracker::ensure_move_history()
{
    // Don't sample mid-gesture: wait for the gizmo/pin drag to settle so we record
    // one entry per move, not one per redraw.
    if (gizmo_active_handle_ >= 0 || dragging_pin_idx_ >= 0) return;

    double cur[7];
    if (!read_offset_values(cur)) return;
    const std::string cur_pins = serialize_pin_list(pins_);   // authoritative in-memory pin set

    std::vector<MoveSnapshot> hist;
    int pos = -1;
    load_move_history(hist, pos);

    if (hist.empty()) {                              // seed the baseline (pre-first-move)
        MoveSnapshot s; std::copy(cur, cur + 7, s.v); s.frame = editing_frame(); s.pins = cur_pins; capture_tracked_extras(s);
        hist.push_back(s);
        store_move_history(hist, 0);
        PCN_LOG("[PolychaseTracker] move-history baseline seeded (draw) @f"
                  << s.frame << "\n");
        return;
    }

    // Settled at the current history position (covers the steady state and the
    // moment right after an Undo/Redo restore) → nothing to record. A move counts
    // if EITHER the pose offsets OR the pin set changed.
    const bool same = (pos >= 0 && pos < (int)hist.size()
                       && offsets_equal(cur, hist[pos].v)
                       && cur_pins == hist[pos].pins);
    if (same) return;

    // Not armed yet (e.g. just reloaded a script: the saved history is present but
    // the transient offsets came back as zero). Don't auto-append on load — wait
    // for a real grab to resume capture, so the saved trail is never mutated.
    if (!move_hist_armed_) return;

    // A genuine new value → drop any redo branch and append it.
    if (pos + 1 < (int)hist.size()) hist.resize(pos + 1);
    MoveSnapshot s; std::copy(cur, cur + 7, s.v); s.frame = editing_frame(); s.pins = cur_pins; capture_tracked_extras(s);
    hist.push_back(s);
    pos = (int)hist.size() - 1;

    const int kCap = 256;
    while ((int)hist.size() > kCap) { hist.erase(hist.begin()); --pos; }

    store_move_history(hist, pos);
    PCN_LOG("[PolychaseTracker] record move -> index " << pos
              << "/" << (int)hist.size() - 1 << " @f" << s.frame
              << "  T(" << s.v[0] << ", " << s.v[1] << ", " << s.v[2] << ")"
              << "  Dolly " << s.v[3]
              << "  R(" << s.v[4] << ", " << s.v[5] << ", " << s.v[6] << ")"
              << "  pinsLen=" << s.pins.size() << "\n");
}

// Explicitly commit the current state (offsets + pin set) as a distinct history
// entry. Used to capture a NEW pin the instant it is placed — before the
// place-and-drag motion moves it — so "pin placed" and "pin moved" become two
// separate undo steps (undo the move keeps the pin; undo again removes it).
void PolychaseTracker::record_move_snapshot(const char* why)
{
    if (!move_hist_armed_) return;
    double cur[7];
    if (!read_offset_values(cur)) return;
    const std::string cur_pins = serialize_pin_list(pins_);

    std::vector<MoveSnapshot> hist;
    int pos = -1;
    load_move_history(hist, pos);

    const bool same = (pos >= 0 && pos < (int)hist.size()
                       && offsets_equal(cur, hist[pos].v)
                       && cur_pins == hist[pos].pins);
    if (same) return;

    if (pos + 1 < (int)hist.size()) hist.resize(pos + 1);
    MoveSnapshot s; std::copy(cur, cur + 7, s.v); s.frame = editing_frame(); s.pins = cur_pins; capture_tracked_extras(s);
    hist.push_back(s);
    pos = (int)hist.size() - 1;

    const int kCap = 256;
    while ((int)hist.size() > kCap) { hist.erase(hist.begin()); --pos; }

    store_move_history(hist, pos);
    PCN_LOG("[PolychaseTracker] record (" << why << ") -> index " << pos
              << "/" << (int)hist.size() - 1 << " @f" << s.frame
              << "  pinsLen=" << s.pins.size() << "\n");
}

void PolychaseTracker::restore_move_state(const MoveSnapshot& s, const MoveSnapshot* from)
{
    // Does this undo/redo step actually change the POSE? A display/parameter-only
    // step (colour, Axis Gradient, Gizmo Size, First/Last Frame, Solve Mode) leaves
    // the offsets and frame identical to the snapshot we
    // came from. In that case we must NOT touch the offset knobs or republish the
    // pose — otherwise undoing e.g. Axis Gradient while parked mid-timeline would
    // yank the wireframe to this snapshot's frame/pose. Only a genuine pose/pin
    // move (offsets or frame differ) rewrites the pose and republishes. With no
    // `from` (defensive default) we do the full restore as before.
    const bool pose_changed =
        !from || !offsets_equal(s.v, from->v) || s.frame != from->frame;

    if (pose_changed) {
        // Write the seven offset knobs back to the snapshot — same path a typed edit
        // takes. Suppress the per-knob callbacks and recompute once at the end.
        {
            ScopedFlags guard(suppress_trans_callback_, suppress_rot_callback_);
            for (int i = 0; i < 7; ++i)
                if (DD::Image::Knob* k = knob(kOffsetKnobNames[i])) k->set_value(s.v[i]);
        }

        live_edit_frame_ = s.frame;
        recompute_pose_from_offsets();   // rebuild live_scene_ from the restored offsets
    }

    // Restore the tracked ordinary knobs this snapshot carried — wireframe colour,
    // Axis Gradient and the numeric scalar knobs (First/Last Frame, Gizmo Size,
    // Solve Mode). set_value fires knob_changed, so suppress
    // our own recorder to keep an undo from logging a fresh edit. committed_ is
    // realigned centrally by store_move_history(); last_tracked_knob_ is cleared so
    // the next ordinary edit starts a fresh undo step rather than coalescing.
    {
        ScopedFlags guard(suppress_tracked_callback_);
        if (DD::Image::Knob* kc = knob("wire_color"))
            for (int i = 0; i < 3; ++i) kc->set_value(s.color[i], i);
        for (int i = 0; i < 3; ++i) wire_color_[i] = s.color[i];   // draw reads the member
        for (size_t i = 0; i < s.extra.size() && (int)i < kNumExtraKnobs; ++i)
            if (DD::Image::Knob* k = knob(kExtraKnobNames[i])) k->set_value(s.extra[i]);
    }
    last_tracked_knob_.clear();

    // Restore the pin set this snapshot was taken with, so undoing a pin create
    // removes it and undoing a pin move puts it back (only when it actually differs,
    // to avoid needless pin-list churn on a pure display undo).
    if (s.pins != serialize_pin_list(pins_)) restore_pins_from_blob(s.pins);

    // Republish through the shared live_pose_blob ONLY on a real pose change, so the
    // viewer instance rebuilds its wireframe. A display-only undo skips this and the
    // overlay stays exactly where it is.
    if (pose_changed) sync_blob_from_live_pose();
    asapUpdate();
    PCN_LOG("[PolychaseTracker]   restored -> @f" << s.frame
              << (pose_changed ? "  (pose)" : "  (display-only)")
              << "  T(" << s.v[0] << ", " << s.v[1] << ", " << s.v[2] << ")"
              << "  Dolly " << s.v[3]
              << "  R(" << s.v[4] << ", " << s.v[5] << ", " << s.v[6] << ")"
              << "  pinsLen=" << s.pins.size() << "\n");
}

// ---------------------------------------------------------------------------
// capture_tracked_extras — fill the snapshot's ordinary-knob state (colour, the
// numeric scalar knobs, the Database path) from the LIVE knobs. The pose offsets,
// frame and pins are filled by the caller; this stamps everything else so a pose
// move also carries the current colour/frames/mode (undoing a move never disturbs
// them). Reads via the knobs (shared across Op instances) where possible.
// ---------------------------------------------------------------------------
void PolychaseTracker::capture_tracked_extras(MoveSnapshot& s) const
{
    if (DD::Image::Knob* kc = knob("wire_color"))
        for (int i = 0; i < 3; ++i) s.color[i] = (float)kc->get_value(i);
    else
        for (int i = 0; i < 3; ++i) s.color[i] = wire_color_[i];

    s.extra.resize((size_t)kNumExtraKnobs);
    for (int i = 0; i < kNumExtraKnobs; ++i) {
        DD::Image::Knob* k = knob(kExtraKnobNames[i]);
        s.extra[(size_t)i] = k ? k->get_value() : 0.0;
    }

    // Database path is intentionally NOT tracked yet: it's a string and reading it
    // safely across the viewer/panel Op-instance split needs an accessor this
    // codebase doesn't use. The serialized slot is reserved (always empty for now).
    s.db_path.clear();
}

bool PolychaseTracker::tracked_extras_equal(const MoveSnapshot& a, const MoveSnapshot& b) const
{
    if (!color_equal(a.color, b.color)) return false;
    if (a.extra.size() != b.extra.size()) return false;
    for (size_t i = 0; i < a.extra.size(); ++i)
        if (std::fabs(a.extra[i] - b.extra[i]) > 1e-9) return false;
    return a.db_path == b.db_path;
}

void PolychaseTracker::prime_committed_from_live()
{
    // Capture the current ordinary-knob values as the "before" reference, so the
    // first edit after the panel opens has a correct baseline to undo back to.
    capture_tracked_extras(committed_);
    last_tracked_knob_.clear();
}

// ---------------------------------------------------------------------------
// on_tracked_knob_changed — record an ordinary-knob edit (colour, First/Last
// Frame, Gizmo Size, Solve Mode) as a first-class step in
// the SAME move-history stack as pose/pin moves. This is why these now undo with
// Ctrl+Z (the viewer hotkey routes to the move history) without a Nuke-undo
// fallback that fought the live_pose_blob entries.
//
// `committed_` holds the values as of the current cursor (the "before"); the live
// knobs are the "after". Consecutive edits of the SAME knob (slider/picker drags
// fire many ticks) coalesce into one undo step; distinct knobs each get their own.
// ---------------------------------------------------------------------------
void PolychaseTracker::on_tracked_knob_changed(const char* which)
{
    // Has anything actually changed vs the committed cursor? Compare extras only.
    MoveSnapshot probe;                 // pose/frame/pins irrelevant for this test
    capture_tracked_extras(probe);
    if (tracked_extras_equal(probe, committed_)) { asapUpdate(); return; }

    double cur[7];
    const bool have_pose = read_offset_values(cur);
    const std::string cur_pins = serialize_pin_list(pins_);

    std::vector<MoveSnapshot> hist; int pos = -1;
    load_move_history(hist, pos);

    // Guarantee a "before" entry holding the OLD extras to undo back to. If the
    // stack is empty (no move and no draw-seed yet), seed it from the CURRENT pose/
    // pins + the committed (pre-edit) ordinary-knob values.
    if (hist.empty()) {
        MoveSnapshot b;
        if (have_pose) std::copy(cur, cur + 7, b.v);
        b.frame = editing_frame();
        b.pins  = cur_pins;
        for (int i = 0; i < 3; ++i) b.color[i] = committed_.color[i];
        b.extra   = committed_.extra;
        b.db_path = committed_.db_path;
        hist.push_back(b);
        pos = 0;
    }

    // The AFTER snapshot INHERITS the predecessor's pose/frame/pins and only changes
    // the extras. This is what makes a display/parameter edit pose-neutral: the entry
    // before and after differ solely in the ordinary-knob values, so undo/redo flips
    // just that knob and restore_move_state() skips the pose republish — the overlay
    // never moves (the bug where unchecking Axis Gradient mid-timeline jumped the
    // wireframe). We do NOT snapshot the live offsets here; a pose move is recorded
    // separately by ensure_move_history().
    MoveSnapshot after = hist[pos];
    capture_tracked_extras(after);

    // Coalesce ONLY at the tip, only when the SAME knob is being edited again and the
    // Coalescing policy: only CONTINUOUS-drag controls — the colour picker and the
    // Gizmo Size slider — fire many knob_changed ticks per single gesture, so we
    // collapse their consecutive ticks into ONE undo step. Everything else is a
    // DISCRETE edit: a checkbox (Axis Gradient), the Solve Mode menu, the
    // frame fields. Those get one step PER edit so Undo/Redo replay exactly what the
    // user did — toggle on/off seven times → seven undo steps, redo puts them back.
    const std::string w = which ? which : "";
    const bool coalescing_knob = (w == "wire_color" || w == "gizmo_size");

    bool coalesced = false;
    if (coalescing_knob && pos == (int)hist.size() - 1 && pos >= 1 &&
        last_tracked_knob_ == w) {
        const MoveSnapshot& prev = hist[pos - 1];
        const MoveSnapshot& curS = hist[pos];
        if (offsets_equal(curS.v, prev.v) && curS.pins == prev.pins) {
            hist[pos] = after;          // collapse this drag tick into the tip
            coalesced = true;
        }
    }
    if (!coalesced) {
        if (pos + 1 < (int)hist.size()) hist.resize(pos + 1);   // drop redo branch
        hist.push_back(after);
        pos = (int)hist.size() - 1;
        const int kCap = 256;
        while ((int)hist.size() > kCap) { hist.erase(hist.begin()); --pos; }
    }

    store_move_history(hist, pos);     // also realigns committed_ to hist[pos]
    move_hist_armed_ = true;           // an ordinary edit is a real edit — keep capturing
    last_tracked_knob_ = w;

    PCN_LOG("[PolychaseTracker] record knob '" << (which ? which : "?")
              << "' -> index " << pos << "/" << (int)hist.size() - 1
              << (coalesced ? " (coalesced)" : "") << "\n");
    asapUpdate();
}

void PolychaseTracker::do_undo_move()
{
    std::vector<MoveSnapshot> hist; int pos = -1;
    load_move_history(hist, pos);
    if (pos > 0) {
        const MoveSnapshot from = hist[pos];   // the state we're leaving
        pos -= 1;
        store_move_history(hist, pos);
        restore_move_state(hist[pos], &from);
        PCN_LOG("[PolychaseTracker] Undo Move -> index " << pos
                  << "/" << (int)hist.size() - 1 << "\n");
    } else {
        set_status("Undo Move: nothing to undo");
        PCN_LOG("[PolychaseTracker] Undo Move: nothing to undo (pos="
                  << pos << ", size=" << hist.size() << ")\n");
    }
}

void PolychaseTracker::do_redo_move()
{
    std::vector<MoveSnapshot> hist; int pos = -1;
    load_move_history(hist, pos);
    if (pos >= 0 && pos < (int)hist.size() - 1) {
        const MoveSnapshot from = hist[pos];   // the state we're leaving
        pos += 1;
        store_move_history(hist, pos);
        restore_move_state(hist[pos], &from);
        PCN_LOG("[PolychaseTracker] Redo Move -> index " << pos
                  << "/" << (int)hist.size() - 1 << "\n");
    } else {
        set_status("Redo Move: nothing to redo");
        PCN_LOG("[PolychaseTracker] Redo Move: nothing to redo (pos="
                  << pos << ", size=" << hist.size() << ")\n");
    }
}

// Wipe the move-history stack without touching the current pose/pins/knob values —
// it only forgets the undo trail. We then immediately re-seed a fresh baseline from
// the CURRENT state and KEEP capture armed, so history continues accumulating right
// away: the next gizmo/pin move, colour, gradient or other tracked edit records on
// top of this baseline, and Undo Move walks back to exactly the post-clear state.
// (Disarming here was the bug — it silently paused recording of any pose change that
// doesn't go through a grab until the next grab re-armed.)
void PolychaseTracker::clear_move_history()
{
    MoveSnapshot base;
    double cur[7];
    if (read_offset_values(cur)) std::copy(cur, cur + 7, base.v);
    base.frame = editing_frame();
    base.pins  = serialize_pin_list(pins_);
    capture_tracked_extras(base);            // current colour/gradient/extras

    std::vector<MoveSnapshot> fresh;
    fresh.push_back(base);
    store_move_history(fresh, 0);            // single baseline; committed_ -> base
    move_hist_armed_   = true;               // keep recording from here on
    last_tracked_knob_.clear();

    set_status("Move history cleared");
    PCN_LOG("[PolychaseTracker] move history cleared (baseline re-seeded @f"
              << base.frame << ")\n");
    asapUpdate();
}

// Discard an uncommitted pose edit at the current frame so the overlay snaps back to
// the keyed animation, killing the between-keys flicker. The scenario: you start a
// gizmo/knob edit on a frame with no key (e.g. frame 40, between keys at 1 and 65),
// then change your mind. That edit lives in the seven offset knobs and is pinned as a
// live preview at this frame, so on playback the frame flickers off the animation to
// the abandoned pose. Zero the offset knobs (the edit) and drop the preview, then
// republish empty so every Op instance (the viewer overlay included) resyncs. With no
// edit, effective_model_matrix falls through to the keyed pose where keys exist (the
// animation), otherwise the geo's upstream/rest position. Keyframes live in the pose
// curves, not the offsets, so they are never touched; pins are left as-is.
void PolychaseTracker::refresh_overlay()
{
    {
        ScopedFlags guard(suppress_trans_callback_, suppress_rot_callback_);
        for (int i = 0; i < 7; ++i)
            if (DD::Image::Knob* k = knob(kOffsetKnobNames[i])) k->set_value(0.0);
    }

    live_scene_.reset();             // drop the (possibly stranded) live preview
    live_edit_frame_ = -1000000;     // nothing is being previewed at any frame now
    sync_blob_from_live_pose();      // live_scene_ null -> empty blob -> all instances clear

    const bool keyed = has_pose_keys();
    set_status(keyed ? "Overlay refreshed — following keyed animation"
                     : "Overlay refreshed — geo at rest pose");
    PCN_LOG("[PolychaseTracker] overlay refreshed, discarded uncommitted edit ("
              << (keyed ? "keyed animation" : "rest pose") << ")\n");
    asapUpdate();
}

// ---------------------------------------------------------------------------
// Undoable live gizmo pose.
//
// A gizmo translate/dolly drag only mutates live_scene_ in memory, which Nuke's
// undo cannot see. These helpers mirror live_scene_ into the hidden, writable
// live_pose_blob knob so the undo stack captures it: sync_blob_from_live_pose()
// serializes the edit frame + model_matrix on gizmo release (and writes the
// empty string on the reset paths); on_live_pose_blob_changed() rebuilds (or
// clears) live_scene_ from the blob on undo/redo and on .nk load. Same round-trip
// shape as pins_blob — a suppress flag + content cache stop our own write from
// recursively re-importing.
// ---------------------------------------------------------------------------
void PolychaseTracker::sync_blob_from_live_pose()
{
    std::string s;                       // empty == "no live pose"
    if (live_scene_) {
        std::ostringstream o;
        o << std::setprecision(9) << live_edit_frame_;
        const RowMajorMatrix4f& m = live_scene_->model_matrix;
        for (int i = 0; i < 16; ++i) o << ' ' << m.data()[i];
        s = o.str();
    }
    {
        ScopedFlags guard(suppress_live_pose_callback_);
        if (DD::Image::Knob* k = knob("live_pose_blob")) k->set_text(s.c_str());
        live_pose_blob_cache_ = s;
    }
    asapUpdate();
}

void PolychaseTracker::ensure_live_pose_loaded()
{
    // Poll: on_live_pose_blob_changed() early-outs when the blob still matches our
    // cache, so this is cheap every redraw and only rebuilds when an undo/redo (or
    // .nk load) has reverted the blob behind our back — the event doesn't fire
    // reliably for this INVISIBLE knob, so the draw pass is what catches it.
    on_live_pose_blob_changed();
}

void PolychaseTracker::on_live_pose_blob_changed()
{
    // Fires on undo/redo and on .nk load. Skip our own writes (suppress flag) and
    // our own echo (content cache) so only a genuine external change reloads.
    const char* blob = live_pose_blob_ ? live_pose_blob_ : "";
    if (suppress_live_pose_callback_ || live_pose_blob_cache_ == blob) return;
    live_pose_blob_cache_ = blob;

    if (blob[0] == '\0') {               // reverted to "no live pose"
        live_scene_.reset();
        rot_base_.reset();
        rot_base_had_live_ = false;
        trans_base_.reset();
        trans_base_had_live_ = false;
        // Zero the offset readout so the numbers match the (no-offset) state.
        {
            ScopedFlags guard(suppress_trans_callback_, suppress_rot_callback_);
            for (const char* nm : { "trans_x", "trans_y", "trans_z", "dolly",
                                    "rot_x", "rot_y", "rot_z" })
                if (DD::Image::Knob* k = knob(nm)) k->set_value(0.0);
        }
        asapUpdate();
        return;
    }

    std::istringstream in(blob);
    int frame = 0;
    in >> frame;
    RowMajorMatrix4f m;
    for (int i = 0; i < 16; ++i) in >> m.data()[i];
    if (in.fail()) return;               // malformed — leave state untouched

    SceneTransformations st{};
    if (live_scene_) st = *live_scene_;  // carry any existing view/intrinsics
    st.model_matrix  = m;
    live_scene_      = st;
    live_edit_frame_ = frame;
    // A restored gizmo pose invalidates any frozen rotation/translate base.
    rot_base_.reset();
    rot_base_had_live_ = false;
    trans_base_.reset();
    trans_base_had_live_ = false;
    // Revert the Translate/Dolly/Rotate readout to match the restored pose so the
    // numbers undo together with the wireframe (decompose against the upstream base).
    sync_offsets_from_pose(m);
    asapUpdate();
}

// ---------------------------------------------------------------------------
// Rotation-offset preview (replaces the old apply-and-key nudge).
//
// The three Rotate X/Y/Z knobs hold an absolute offset (degrees) layered on a
// FROZEN base (rot_base_, captured on the first nonzero edit = the current
// working pose without the offset). Editing one recomputes the preview into
// live_scene_ but does NOT key — committing is the explicit "Set Pose Key"
// button (which folds the offset back to 0). The overlay shows the preview
// because effective_model_matrix returns live_scene_ whenever it is set.
// rot_offset_pending() is used by the gizmo grab to bake a pending offset
// before a translate/dolly drag begins.
// ---------------------------------------------------------------------------
bool PolychaseTracker::rot_offset_pending() const
{
    auto v = [&](const char* nm) { Knob* k = knob(nm); return k ? k->get_value() : 0.0; };
    return v("rot_x") != 0.0 || v("rot_y") != 0.0 || v("rot_z") != 0.0;
}

void PolychaseTracker::reset_rot_offsets()
{
    // set_value() re-fires knob_changed for each field; the ScopedFlags guard
    // turns those re-entrant calls into no-ops so we don't recompute a preview
    // three times mid-reset. Also drop the frozen base so the next rotation
    // edit re-captures it from the (now committed/baked) working pose.
    {
        ScopedFlags guard(suppress_rot_callback_);
        for (const char* nm : {"rot_x", "rot_y", "rot_z"})
            if (Knob* k = knob(nm)) k->set_value(0.0);
    }
    rot_base_.reset();
    rot_base_had_live_ = false;
}

void PolychaseTracker::preview_center_rotation()
{
    // Rotation is now one input into the single unified offset->pose map (it
    // composes with translate/dolly onto the upstream entry-point base, rotating
    // about the current geo centroid). No separate frozen rot_base_ anymore.
    recompute_pose_from_offsets();
}

// ---------------------------------------------------------------------------
// Translate/dolly-offset preview — the exact analogue of preview_center_rotation
// for the trans_x/y/z + dolly knobs. trans_* are a WORLD-space translation
// offset; dolly is a signed distance along trans_base_dolly_dir_ (the camera->
// mesh viewing ray). Both layer on a FROZEN base (trans_base_) and recompute
// live_scene_ without keying. The gizmo grab pre-freezes trans_base_ so a drag
// and its undo replay reconstruct identically; a typed edit freezes it here on
// the first nonzero value. "Set Pose Key" bakes and reset_trans_offsets() folds
// the fields back to 0.
// ---------------------------------------------------------------------------
bool PolychaseTracker::trans_offset_pending() const
{
    auto v = [&](const char* nm) { Knob* k = knob(nm); return k ? k->get_value() : 0.0; };
    return v("trans_x") != 0.0 || v("trans_y") != 0.0 ||
           v("trans_z") != 0.0 || v("dolly")   != 0.0;
}

void PolychaseTracker::reset_trans_offsets()
{
    // set_value() re-fires knob_changed for each field; the ScopedFlags guard
    // turns those re-entrant calls into no-ops so we don't recompute the preview
    // four times mid-reset. Also drop the frozen base so the next translate/dolly
    // edit re-captures it from the (now committed/baked) working pose.
    {
        ScopedFlags guard(suppress_trans_callback_);
        for (const char* nm : {"trans_x", "trans_y", "trans_z", "dolly"})
            if (Knob* k = knob(nm)) k->set_value(0.0);
    }
    trans_base_.reset();
    trans_base_had_live_ = false;
}

void PolychaseTracker::preview_translate_offset()
{
    // Translate + dolly are now two inputs into the single unified offset->pose
    // map (see recompute_pose_from_offsets). Dolly is anchored to the CURRENT geo
    // centroid (upstream centroid + translate), so it tracks the gizmo pivot in
    // the viewer even after a sideways move.
    recompute_pose_from_offsets();
}


// ---------------------------------------------------------------------------
// recompute_pose_from_offsets — the single forward map. Reads all seven persistent
// offset knobs (trans_x/y/z, dolly, rot_x/y/z) and composes them onto the upstream
// entry-point pose via compose_offset_pose(), publishing the result as live_scene_
// at the editing frame. Base is ALWAYS the upstream geo (the "zero entry point"),
// never the keyed pose — that is what lets the offsets persist across Set Pose Key
// without double-counting. With all offsets zero, the live preview is dropped so
// the keyed/upstream pose shows through.
// ---------------------------------------------------------------------------
void PolychaseTracker::recompute_pose_from_offsets()
{
    Op* geo = input_geo_op();
    if (!geo) return;
    GeoMesh gm;
    if (!extract_mesh(geo, gm)) return;

    auto val = [&](const char* nm) { Knob* k = knob(nm); return k ? k->get_value() : 0.0; };
    double T[3]   = { val("trans_x"), val("trans_y"), val("trans_z") };
    double dolly  = val("dolly");
    double R[3]   = { val("rot_x"), val("rot_y"), val("rot_z") };

    if (T[0] == 0.0 && T[1] == 0.0 && T[2] == 0.0 && dolly == 0.0 &&
        R[0] == 0.0 && R[1] == 0.0 && R[2] == 0.0) {
        live_scene_.reset();   // no offset -> let keyed/upstream show through
        asapUpdate();
        return;
    }

    const RowMajorMatrix4f U = nuke_to_eigen_m4(gm.object_to_world);
    const V3 lc = mesh_local_center(gm);
    Eigen::Vector3f cam_pos(0.f, 0.f, 0.f);
    if (CameraOp* cam = input_cam()) {
        cam->validate(true);
        const Matrix4 c2w = cam->matrix();
        const Vector4 c   = c2w * Vector4(0.f, 0.f, 0.f, 1.f);
        cam_pos = Eigen::Vector3f(c.x, c.y, c.z);
    }

    const RowMajorMatrix4f working = compose_offset_pose(U, lc, cam_pos, T, dolly, R);
    SceneTransformations st{};
    if (live_scene_) st = *live_scene_;
    st.model_matrix  = working;
    live_scene_      = st;
    live_edit_frame_ = editing_frame();
    asapUpdate();
}


// ---------------------------------------------------------------------------
// sync_offsets_from_pose — the inverse map. Given an authoritative working pose
// (e.g. a pin solve), decompose it against the upstream base into the translate +
// rotate offset knobs (dolly folded into translate, reported 0) so the readout
// always reflects what's actually happening. Writes are suppressed so they don't
// recursively re-trigger recompute. The caller owns whether/when this is recorded
// for undo.
// ---------------------------------------------------------------------------
void PolychaseTracker::sync_offsets_from_pose(const RowMajorMatrix4f& working)
{
    Op* geo = input_geo_op();
    if (!geo) return;
    GeoMesh gm;
    if (!extract_mesh(geo, gm)) return;

    const RowMajorMatrix4f U = nuke_to_eigen_m4(gm.object_to_world);
    const V3 lc = mesh_local_center(gm);
    Eigen::Vector3f cam_pos(0.f, 0.f, 0.f);
    if (CameraOp* cam = input_cam()) {
        cam->validate(true);
        const Vector4 c = cam->matrix() * Vector4(0.f, 0.f, 0.f, 1.f);
        cam_pos = Eigen::Vector3f(c.x, c.y, c.z);
    }
    double T[3], R[3], dolly = 0.0;
    decompose_offset_pose(working, U, lc, cam_pos, T, dolly, R);

    {
        ScopedFlags guard(suppress_trans_callback_, suppress_rot_callback_);
        auto setk = [&](const char* nm, double v) { if (Knob* k = knob(nm)) k->set_value(v); };
        setk("trans_x", T[0]); setk("trans_y", T[1]); setk("trans_z", T[2]);
        setk("dolly", dolly);
        setk("rot_x", R[0]); setk("rot_y", R[1]); setk("rot_z", R[2]);
    }

    SceneTransformations st{};
    if (live_scene_) st = *live_scene_;
    st.model_matrix = working;
    live_scene_     = st;
    asapUpdate();
}

}  // namespace pcn