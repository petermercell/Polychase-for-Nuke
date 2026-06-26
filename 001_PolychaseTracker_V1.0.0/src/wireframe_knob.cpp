// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Peter Mercell
//
// Developed with assistance from Claude (Anthropic).

// wireframe_knob.cpp — part of the PolychaseTracker plugin (see polychase_tracker.h).
#include "polychase_tracker.h"

#include "pcn_convention.h"   // conv::apply_curve_focal (single a00/a11 focal rewrite)

#include <GL/gl.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

using namespace DD::Image;

namespace pcn {

namespace {

// -----------------------------------------------------------------------------
// draw_pin_hud_glyphs — self-contained GL_LINES "PIN" label.
//
// No dependency on any gl_text / font API (whose availability varies by Nuke
// build): each letter is hand-stroked from unit-square segments, scaled to a
// caller-supplied cell height and laid out left-to-right. Coordinates are in
// the same image-pixel space the wireframe draws in (Y-up), so the caller just
// passes a top-left origin. Call inside an active GL state block; this sets
// colour + line width itself.
// -----------------------------------------------------------------------------
inline void draw_pin_hud_glyphs(float x0, float ytop, float h)
{
    const float w  = h * 0.62f;          // glyph cell width
    const float g  = h * 0.35f;          // inter-glyph gap
    const float yb = ytop - h;           // cell bottom (Y-up: bottom < top)
    const float ym = ytop - h * 0.5f;    // cell midline

    glLineWidth(2.5f);
    glColor4f(1.0f, 0.78f, 0.10f, 1.0f); // bright amber — matches active pins
    glBegin(GL_LINES);

    auto seg = [&](float ax, float ay, float bx, float by) {
        glVertex2f(ax, ay); glVertex2f(bx, by);
    };

    float x = x0;
    // 'P' : left stem + upper bowl (top, right-upper, midline back to stem).
    seg(x, yb, x, ytop);
    seg(x, ytop, x + w, ytop);
    seg(x + w, ytop, x + w, ym);
    seg(x + w, ym, x, ym);
    x += w + g;

    // 'I' : centre stem with top & bottom serifs.
    {
        const float xc = x + w * 0.5f;
        seg(xc, yb, xc, ytop);
        seg(x, ytop, x + w, ytop);
        seg(x, yb, x + w, yb);
    }
    x += w + g;

    // 'N' : left stem, right stem, top-left → bottom-right diagonal.
    seg(x, yb, x, ytop);
    seg(x + w, yb, x + w, ytop);
    seg(x, ytop, x + w, yb);

    glEnd();
}


// -----------------------------------------------------------------------------
// draw_number_glyphs — stroke a non-negative integer as 7-segment digits, the
// same gl_text-free approach as draw_pin_hud_glyphs (font-API availability varies
// by Nuke build). Coordinates are image-pixel space, Y-up; (x0, ybottom) is the
// bottom-left of the first digit, h the cell height. The caller sets glColor
// BEFORE calling (so pin labels can be amber and track labels the marker colour);
// this sets line width + emits a single GL_LINES batch.
// -----------------------------------------------------------------------------
inline void draw_seven_seg_digit(int d, float x, float yb, float ym, float ytop,
                                 float w)
{
    // segment bits: a=top b=tr c=br d=bottom e=bl f=tl g=mid
    static const unsigned char SEG[10] = {
        0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F
    };
    if (d < 0 || d > 9) return;
    const unsigned char m = SEG[d];
    auto S = [&](float ax, float ay, float bx, float by) {
        glVertex2f(ax, ay); glVertex2f(bx, by);
    };
    if (m & 0x01) S(x,     ytop, x + w, ytop);   // a  top
    if (m & 0x02) S(x + w, ym,   x + w, ytop);   // b  top-right
    if (m & 0x04) S(x + w, yb,   x + w, ym);     // c  bottom-right
    if (m & 0x08) S(x,     yb,   x + w, yb);     // d  bottom
    if (m & 0x10) S(x,     yb,   x,     ym);     // e  bottom-left
    if (m & 0x20) S(x,     ym,   x,     ytop);   // f  top-left
    if (m & 0x40) S(x,     ym,   x + w, ym);     // g  middle
}

inline void draw_number_glyphs(int value, float x0, float ybottom, float h)
{
    if (value < 0) value = 0;
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%d", value);

    const float w    = h * 0.55f;       // digit cell width
    const float gap  = h * 0.30f;       // inter-digit gap
    const float yb   = ybottom;
    const float ytop = ybottom + h;
    const float ym   = ybottom + h * 0.5f;

    glLineWidth(1.5f);
    glBegin(GL_LINES);
    float x = x0;
    for (const char* p = buf; *p; ++p) {
        if (*p >= '0' && *p <= '9')
            draw_seven_seg_digit(*p - '0', x, yb, ym, ytop, w);
        x += w + gap;
    }
    glEnd();
}

}  // namespace


// -----------------------------------------------------------------------------
// PolychaseWireframeKnob::draw_world_overlay_3d — 3D-viewport overlay.
//
// In the 3D viewer Nuke has already loaded the navigation camera into GL, so we
// emit the solved proxy + pins in WORLD space (glVertex3f) and let GL transform
// them — no manual projection. World = effective_model_matrix * local, the same
// solved transform the 2D path uses, so the proxy sits at its tracked 3D pose.
// Draw-only for now (no picking); reuses extract_mesh, so it's geometry-agnostic.
// -----------------------------------------------------------------------------
void PolychaseWireframeKnob::draw_world_overlay_3d(DD::Image::ViewerContext* /*ctx*/)
{
    using namespace DD::Image;

    Op* geo_op = owner_->Op::input(2);
    if (!geo_op) return;
    geo_op->validate(true);

    GeoMesh gm;
    if (!extract_mesh(geo_op, gm) || !gm.valid) return;
    const Eigen::Index nverts = gm.local_vertices.rows();
    const Eigen::Index ntri   = gm.triangles.rows();
    if (nverts < 1 || ntri < 1) return;

    const double  vframe = (double)uiContext().frame();
    const Matrix4 o2w    = owner_->effective_model_matrix(gm.object_to_world, vframe);

    // local vertex index -> world position (perspective-divide guarded).
    auto W = [&](Eigen::Index vi, float& x, float& y, float& z) {
        const Vector4 w = o2w * Vector4(gm.local_vertices(vi, 0),
                                        gm.local_vertices(vi, 1),
                                        gm.local_vertices(vi, 2), 1.0f);
        const float iw = (w.w != 0.0f) ? 1.0f / w.w : 1.0f;
        x = w.x * iw; y = w.y * iw; z = w.z * iw;
    };

    // ---- wireframe (flat colour from the wire_color knob) ----
    float wc[4] = {0.2f, 1.0f, 0.55f, 1.0f};
    owner_->wire_color(wc);
    glColor4f(wc[0], wc[1], wc[2], 1.0f);
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    for (Eigen::Index ti = 0; ti < ntri; ++ti) {
        const Eigen::Index a = (Eigen::Index)gm.triangles(ti, 0);
        const Eigen::Index b = (Eigen::Index)gm.triangles(ti, 1);
        const Eigen::Index c = (Eigen::Index)gm.triangles(ti, 2);
        if (a >= nverts || b >= nverts || c >= nverts) continue;
        float ax, ay, az, bx, by, bz, cx, cy, cz;
        W(a, ax, ay, az); W(b, bx, by, bz); W(c, cx, cy, cz);
        glVertex3f(ax, ay, az); glVertex3f(bx, by, bz);
        glVertex3f(bx, by, bz); glVertex3f(cx, cy, cz);
        glVertex3f(cx, cy, cz); glVertex3f(ax, ay, az);
    }
    glEnd();

    // ---- pins (amber world-space points anchored to their mesh vertices) ----
    const std::vector<Pin>& pins = owner_->pins();
    if (!pins.empty()) {
        glColor4f(1.0f, 0.78f, 0.10f, 1.0f);   // bright amber — matches 2D pins
        glPointSize(7.0f);
        glBegin(GL_POINTS);
        for (const Pin& p : pins) {
            if ((Eigen::Index)p.vertex_idx >= nverts) continue;
            float x, y, z;
            W((Eigen::Index)p.vertex_idx, x, y, z);
            glVertex3f(x, y, z);
        }
        glEnd();
        glPointSize(1.0f);
    }
}


// -----------------------------------------------------------------------------
// PolychaseWireframeKnob::draw_handle — out-of-class definition.
//
// Has to be defined AFTER PolychaseTracker is fully declared because we call
// owner_->input() which requires the complete Op type.
//
// Per-frame on viewer redraw in 2D mode:
//   1. Resolve typed inputs (img on 0, camera on 1, geo on 2).
//   2. Pull format dimensions from the image (for NDC→pixel mapping).
//   3. Build view*projection = cam->projection() * cam->imatrix().
//   4. Walk geo: for each primitive, for each sub-face, emit each edge
//      as a projected line pair through view*projection.
//   5. Discard edges where either endpoint is behind the camera (w <= 0).
//
// Performance note: this runs every redraw. For modest meshes (cube = 6 quads
// → 24 edges, or torus = a few hundred edges) it's fine. For dense meshes
// we'd want to cache projected vertices. Polychase use case is low-poly
// proxies so we're not there yet.
// -----------------------------------------------------------------------------
void PolychaseWireframeKnob::draw_handle(DD::Image::ViewerContext* ctx)
{
    using namespace DD::Image;

    if (!owner_) return;

    const auto evt = ctx->event();

    // 3D viewport: in Nuke 17 legacy draw_handle still composites over the Hydra
    // viewer (verified — same mechanism Axis4/Camera4 use). The 2D path below hand-
    // projects through the TRACKED camera into plate pixels; in 3D the viewer has
    // already loaded its own navigation camera into GL, so we draw the proxy + pins
    // in WORLD space and let GL transform them. (3D mouse picking is a later step;
    // for now 3D is draw-only — interaction stays in the 2D viewer.)
    if (ctx->viewer_mode() != VIEWER_2D) {
        if (evt == DRAW_OPAQUE) draw_world_overlay_3d(ctx);
        return;
    }
    if (ctx->transform_mode() != 0) return;  // 2D viewer only beyond this point

    // ---- Mouse-event dispatch ----
    // PUSH/DRAG/RELEASE all use the GL state cached during the last
    // DRAW_OPAQUE (we need it to project vertex image-pixel coords through
    // the viewer's pan+zoom transform, and inversely to convert mouse
    // screen pixels back to image-pixel coords).
    //
    // The Op decides on PUSH whether the click hits an existing pin
    // (→ start drag) or hits empty space (→ try to place a new pin via
    // vertex snap).
    if (evt == PUSH) {
        owner_->on_mouse_push   (ctx, cached_mv_, cached_pj_, cached_vp_, cache_valid_);
        return;
    }
    if (evt == DRAG) {
        owner_->on_mouse_drag   (ctx, cached_mv_, cached_pj_, cached_vp_, cache_valid_);
        return;
    }
    if (evt == RELEASE) {
        owner_->on_mouse_release(ctx, cached_mv_, cached_pj_, cached_vp_, cache_valid_);
        return;
    }
    // A button-up mouse move. If a pin drag is still "active" here, its RELEASE was
    // missed (common for place-and-drag) — finish it now so the pin rests (amber)
    // instead of staying red, and so the completed move gets recorded for undo.
    if (evt == MOVE) {
        if (owner_->dragging_pin_slot() >= 0)
            owner_->on_mouse_release(ctx, cached_mv_, cached_pj_, cached_vp_, cache_valid_);
        return;
    }

    // ---- shift+P toggles pin-edit arming ----
    // Keydown only (no release/repeat handling) so a single tap flips the arm
    // state. We match uppercase 'P' only — i.e. Shift held — so a bare p (a common
    // Nuke viewer shortcut) doesn't collide. The Bool_knob "Pin Edit [shift+P]"
    // mirrors the same flag and is the guaranteed fallback if a given Nuke build
    // doesn't route KEY events to an overlay knob's draw_handle. toggle_pin_input()
    // is idempotent and writes the knob, so both entry points stay in sync.
    if (evt == KEY) {
        const int key = ctx->key();
        if (key == 'P') {
            owner_->toggle_pin_input();
        }
        return;
    }

    // ---- Drawing pass: only the opaque draw event ----
    if (evt != DRAW_OPAQUE) return;

    // Capture the viewer's current GL state — modelview + projection matrices
    // and viewport rect. PUSH events fire LATER and need this to convert
    // image-pixel vertex projections to viewer screen pixels for comparison
    // against mouse_x/mouse_y.
    glGetDoublev(GL_MODELVIEW_MATRIX,  cached_mv_);
    glGetDoublev(GL_PROJECTION_MATRIX, cached_pj_);
    glGetIntegerv(GL_VIEWPORT,         cached_vp_);
    cache_valid_ = true;

    // Cache the displayed frame alongside the GL state, for the same reason and
    // in the same way: PUSH/DRAG/RELEASE fire LATER and must read the frame the
    // user is looking at, not query viewer state live (unreliable during events).
    // uiContext() tracks the playhead exactly during the draw; a DRAW always
    // precedes a click on a given frame, so the cached value is correct at click
    // time. This is what lets place/grab/solve/key happen on the displayed frame
    // without toggling Pin Edit off/on.
    owner_->set_editing_frame((int)std::floor((double)uiContext().frame() + 0.5));

    // Catch undo/redo of a gizmo/pin gesture: the live_pose_blob is reverted by
    // Nuke behind our back, but its knob_changed doesn't fire reliably (INVISIBLE
    // knob), so poll it here every redraw — rebuilds live_scene_ + the offset
    // readout when it differs from our cache (cheap no-op otherwise).
    owner_->ensure_live_pose_loaded();

    // Track the shared offset numbers for our own Undo/Redo Move history: appends a
    // snapshot once a gizmo/pin move settles (no-op while dragging or unchanged).
    owner_->ensure_move_history();

    // Restore the saved 3D mask on .nk load. knob_changed("mask_blob") commonly
    // does NOT fire during script load, so poll the blob here every redraw (content-
    // diff, cheap no-op once loaded) — same approach as ensure_live_pose_loaded.
    // Without this the mask silently vanishes across a Nuke restart.
    owner_->ensure_mask_loaded();

    // -------- Resolve typed inputs --------
    Iop*      img = dynamic_cast<Iop*>     (owner_->Op::input(0));
    CameraOp* cam = dynamic_cast<CameraOp*>(owner_->Op::input(1));
    GeoOp*    geo = dynamic_cast<GeoOp*>   (owner_->Op::input(2));
    // Raw geo Op* — non-null for BOTH classic GeoOp and new-system GeomOp
    // (GeoCube). `geo` above is null for a GeomOp, which is how the new path is
    // detected below; we still need the Op* to validate + extract from it.
    Op*       geo_op = owner_->Op::input(2);

    // Camera + geo required; image is optional (we use it for format only,
    // fall back to a sensible default if absent).
    if (!cam || !geo_op) return;

    // -------- Force validation so matrices and geo are current --------
    cam->validate(true);
    geo_op->validate(true);
    if (img) img->validate(true);

    // The frame the viewer/UI is at. ViewerContext exposes no frame accessor,
    // and the op's / inputs' outputContext can lag the playhead while scrubbing
    // (served from the viewer cache without re-validating, which froze the
    // overlay at the last-keyed frame). This knob's uiContext() tracks the
    // playhead exactly — the same context the properties panel evaluates at — so
    // the keyframed pose is always evaluated at the displayed frame.
    const double vframe = (double)uiContext().frame();

    // -------- Image format (NDC → pixel mapping uses this) --------
    float fmt_w = 2048.0f, fmt_h = 1080.0f;  // fallback
    if (img) {
        const Format& fmt = img->info().format();
        fmt_w = (float)fmt.width();
        fmt_h = (float)fmt.height();
    }

    // -------- World-to-clip (camera+projection only) --------
    // imatrix(): world-space → camera-space (inverse of camera's world xform).
    // projection(): camera-space → clip-space.
    // We compose with each GeoInfo's local-to-world matrix below.
    //
    // FOCAL FROM THE ANIMATION CURVE (not the cooked projection). cam->projection()
    // does NOT reliably reflect an ANIMATED focal here: while scrubbing, the camera
    // is served from the viewer cache without re-validating at the displayed frame,
    // so the cooked projection carries a STALE focal that flickers between adjacent
    // frames' values on alternate redraws — drawing the wireframe at the wrong scale
    // every other frame even though the POSE (sampled at vframe via uiContext) is
    // correct. Same root cause fixed in the solve (track_via_pins). Read the focal /
    // haperture straight off the camera's knobs at vframe with get_value_at() and
    // rewrite the diagonal projection terms (a00 = a11 = 2*focal_mm/haperture in this
    // uniform-(w/2)-scale convention; sign preserved) via conv::apply_curve_focal.
    //
    // The camera's OWN focal is the single source of truth: a solved zoom is put on
    // the camera with 'Copy Solved Focal -> Camera', after which the wireframe shows
    // the zoom here automatically — the same curve the track reads. Static lens =>
    // identical value, so a non-zoom overlay is unchanged; apply_curve_focal no-ops on
    // a non-finite / non-positive focal or aperture, leaving the cooked projection.
    Matrix4 cam_proj = cam->projection();
    {
        double hap = 0.0;
        if (Knob* hk = cam->knob("haperture")) hap = hk->get_value_at(vframe);

        double foc_mm = 0.0;
        if (Knob* fk = cam->knob("focal")) foc_mm = fk->get_value_at(vframe);

        conv::apply_curve_focal(cam_proj, foc_mm, hap);
    }
    const Matrix4 view_proj = cam_proj * cam->imatrix();

    // -------- NDC → pixel mapping (aspect-correct) --------
    // First revision used (ndc+1)*0.5*fmt_{w,h} for X/Y independently. That
    // worked for position but stretched the wireframe horizontally by
    // fmt_w/fmt_h (≈1.9× for 2K_DCP); a cube viewed straight-on rendered
    // as a landscape rectangle instead of the square the ScanlineRender
    // ground truth produced.
    //
    // Root cause: Nuke's projection() matrix produces NDC where X and Y
    // share the same per-camera-unit scale (it does NOT pre-multiply by
    // image aspect the way gluPerspective does). Using different X/Y pixel
    // scales (fmt_w vs fmt_h) then introduces the stretch.
    //
    // Fix: UNIFORM pixel-per-NDC scale for both axes, centered on the image
    // midpoint. A square object in camera space now projects to a square
    // region in pixels. We use fmt_w*0.5 as the scale (horizontal fit);
    // could equally use fmt_h*0.5 with a corresponding letterbox change
    // but width-based is the standard convention.
    const float scale = fmt_w * 0.5f;
    const float cx    = fmt_w * 0.5f;
    const float cy    = fmt_h * 0.5f;

    // -------- Pull mesh data --------
    Scene scene;
    GeometryList geos;
#ifdef PCN_NEW_3D
    // New-system geometry (GeomOp / GeoCube) makes the GeoOp dynamic_cast fail,
    // so `geo` is null while `geo_op` is the GeomOp. Read it through the usg path
    // into a GeoMesh; the classic get_geometry below would return an empty list
    // for it (and the whole classic draw path then no-ops, drawing nothing).
    GeoMesh new_gm;
    const bool is_new_geo = (geo == nullptr);
    if (is_new_geo) {
        extract_mesh(geo_op, new_gm);   // usg -> GeoMesh (LOCAL verts + fan tris + obj_to_world)
    } else {
        geo->get_geometry(scene, geos);
    }
#else
    geo->get_geometry(scene, geos);
#endif

    // -------- GL state --------
    glPushAttrib(GL_LINE_BIT | GL_CURRENT_BIT | GL_ENABLE_BIT
                 | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_DEPTH_TEST);

    // -------- Masked-triangle tint (object 0) --------
    // Fill each masked triangle with the mask colour so the artist sees what the
    // brush / 3D-select excluded from the solve. The triangle order here MUST
    // mirror extract_first_object_mesh exactly — per primitive (skip null), per
    // face (skip nv<3), fan from vertex 0 — so the running counter `t` equals the
    // bit index in mask_bits_ (== RayHit::primitive_id). Only object 0 carries a
    // mask (the mesh extract/solve use). `continue` still runs the loop increment,
    // so `t` stays in lockstep even on unmasked triangles. Skipped wholesale when
    // nothing is masked. Drawn under the wireframe lines so edges stay crisp.
    if (owner_->has_mask()) {
        float mc[4];
        owner_->mask_color(mc);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        // Cull back-facing masked triangles. The overlay runs with depth test
        // OFF (it sits on top of the plate), so without this the masked
        // triangles on the FAR side of the mesh tint too and bleed through the
        // front — a marquee vertex-select easily grabs near+far verts, so the
        // masked band wraps the object and reads as an oversized, offset blob.
        // Culling leaves only the camera-facing (visible) masked faces. The fan
        // preserves each primitive's winding; if the tint ends up on the hidden
        // side instead, flip GL_CCW <-> GL_CW (single word).
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        glFrontFace(GL_CCW);
        glColor4f(mc[0], mc[1], mc[2], mc[3]);

        // -------- Masked-triangle tint (object 0) --------
        // Fill each masked triangle with the mask colour so the artist sees what
        // the brush / 3D-select excluded from the solve. The triangle order MUST
        // mirror the extract (classic fan / extract_mesh) exactly so the running
        // counter == the bit index in mask_bits_ (== RayHit::primitive_id).
        if (geos.objects() > 0) {                          // classic geometry
            const GeoInfo& info0 = geos[0];
            const PointList* pts0 = info0.point_list();
            if (pts0) {
                const Matrix4 m_obj_to_world =
                    owner_->effective_model_matrix(info0.matrix, vframe);
                const Matrix4 m_mvp = view_proj * m_obj_to_world;

                auto project0 = [&](const Vector3& v, float& px, float& py) -> bool {
                    const Vector4 clip = m_mvp * Vector4(v.x, v.y, v.z, 1.0f);
                    if (clip.w <= 0.001f) return false;
                    px = (clip.x / clip.w) * scale + cx;
                    py = (clip.y / clip.w) * scale + cy;
                    return true;
                };

                glBegin(GL_TRIANGLES);
                uint32_t t = 0;                          // global fan-triangle index
                const unsigned nprim = info0.primitives();
                for (unsigned p = 0; p < nprim; ++p) {
                    const Primitive* prim = info0.primitive(p);
                    if (!prim) continue;
                    const unsigned nf = prim->faces();
                    for (unsigned f = 0; f < nf; ++f) {
                        const unsigned nv = prim->face_vertices((int)f);
                        if (nv < 3) continue;
                        unsigned sbuf[32];
                        std::vector<unsigned> hbuf;
                        unsigned* fidx = sbuf;
                        if (nv > 32) { hbuf.resize(nv); fidx = hbuf.data(); }
                        prim->get_face_vertices((int)f, fidx);

                        const unsigned g0 = prim->vertex(fidx[0]);
                        for (unsigned i = 1; i + 1 < nv; ++i, ++t) {
                            if (!owner_->is_triangle_masked(t)) continue;
                            const unsigned g1 = prim->vertex(fidx[i]);
                            const unsigned g2 = prim->vertex(fidx[i + 1]);
                            float ax, ay, bx, by, cx2, cy2;
                            if (project0((*pts0)[g0], ax, ay) &&
                                project0((*pts0)[g1], bx, by) &&
                                project0((*pts0)[g2], cx2, cy2)) {
                                glVertex2f(ax, ay);
                                glVertex2f(bx, by);
                                glVertex2f(cx2, cy2);
                            }
                        }
                    }
                }
                glEnd();
            }
        }
#ifdef PCN_NEW_3D
        else if (is_new_geo && new_gm.valid) {             // new-system (usg/USD) geometry
            // Same as the wireframe edges below: project new_gm's LOCAL verts
            // through the solved object_to_world. new_gm.triangles is the SAME fan
            // the solver/mask use, so row ti == the mask bit ti directly.
            const Matrix4 m_obj_to_world =
                owner_->effective_model_matrix(new_gm.object_to_world, vframe);
            const Matrix4 m_mvp = view_proj * m_obj_to_world;

            auto projN = [&](Eigen::Index vi, float& px, float& py) -> bool {
                const Vector4 clip = m_mvp * Vector4(new_gm.local_vertices(vi, 0),
                                                     new_gm.local_vertices(vi, 1),
                                                     new_gm.local_vertices(vi, 2), 1.0f);
                if (clip.w <= 0.001f) return false;
                px = (clip.x / clip.w) * scale + cx;
                py = (clip.y / clip.w) * scale + cy;
                return true;
            };

            const Eigen::Index ntri   = new_gm.triangles.rows();
            const Eigen::Index nverts = new_gm.local_vertices.rows();
            glBegin(GL_TRIANGLES);
            for (Eigen::Index ti = 0; ti < ntri; ++ti) {
                if (!owner_->is_triangle_masked((uint32_t)ti)) continue;
                const Eigen::Index a = (Eigen::Index)new_gm.triangles(ti, 0);
                const Eigen::Index b = (Eigen::Index)new_gm.triangles(ti, 1);
                const Eigen::Index c = (Eigen::Index)new_gm.triangles(ti, 2);
                if (a >= nverts || b >= nverts || c >= nverts) continue;
                float ax, ay, bx, by, cx2, cy2;
                if (projN(a, ax, ay) && projN(b, bx, by) && projN(c, cx2, cy2)) {
                    glVertex2f(ax, ay);
                    glVertex2f(bx, by);
                    glVertex2f(cx2, cy2);
                }
            }
            glEnd();
        }
#endif

        glDisable(GL_CULL_FACE);
        glDisable(GL_BLEND);
    }

    // -------- 2D occlusion-mask overlay ("Show Masked") --------
    // Tint the IMAGE-SPACE pixels the mask plate excludes from Track/Refine, so the
    // artist sees exactly what the solve drops — both a setup aid and proof the 2D
    // mask is being applied. The bitset is Y-up (Nuke), the same image-pixel space
    // the wireframe draws in, so it maps straight to glVertex2f with no flip. Drawn
    // downsampled (one quad per stride×stride cell) so it stays cheap on big plates.
    if (owner_->mask2d_show()) {
        int mw = 0, mh = 0;
        const std::vector<uint32_t>* mbits =
            owner_->mask2d_overlay_bits((int)std::floor(vframe + 0.5), mw, mh);
        if (mbits && mw > 0 && mh > 0) {
            float mc2[4];
            owner_->mask2d_color(mc2);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glColor4f(mc2[0], mc2[1], mc2[2], mc2[3]);

            const int wpr    = (mw + 31) / 32;
            const int stride = std::max(1, (int)std::lround((double)std::max(mw, mh) / 320.0));
            glBegin(GL_QUADS);
            for (int y = 0; y < mh; y += stride) {
                const uint32_t* rb = &(*mbits)[(size_t)y * wpr];
                for (int x = 0; x < mw; x += stride) {
                    if ((rb[x >> 5] >> (x & 31)) & 1u) {
                        const float x0 = (float)x;
                        const float y0 = (float)y;
                        const float x1 = (float)std::min(x + stride, mw);
                        const float y1 = (float)std::min(y + stride, mh);
                        glVertex2f(x0, y0); glVertex2f(x1, y0);
                        glVertex2f(x1, y1); glVertex2f(x0, y1);
                    }
                }
            }
            glEnd();
            glDisable(GL_BLEND);
        }
    }

    // Wireframe colour comes from the "wire_color" knob (defaults to the
    // original bright green-cyan: stands out against most plates and distinct
    // from Nuke's white scene wireframe). Read every redraw so a colour change
    // repaints live; alpha stays fully opaque.
    //
    // When the "wire_gradient" toggle is on we instead colour each vertex by its
    // LOCAL position about the mesh origin (X→red, Y→green, Z→blue), so the flat
    // colour is skipped and glColor is set per-vertex inside the edge loop below.
    const bool gradient = owner_->wire_gradient();
    float wc[3] = {0.20f, 1.00f, 0.55f};
    owner_->wire_color(wc);
    if (!gradient) glColor4f(wc[0], wc[1], wc[2], 1.00f);
    glLineWidth(1.5f);

    glBegin(GL_LINES);

    // -------- Walk all objects -> primitives -> sub-faces -> edges --------
    //
    // Each GeoInfo carries its own local-to-world transform (info.matrix).
    // Point positions in info.point_list() are in LOCAL space — they only
    // become world coordinates after multiplying by info.matrix. Failing to
    // do this would:
    //   - misplace any geo that's been transformed (e.g. a USD with an Xform
    //     translate/rotate, or an upstream TransformGeo node)
    //   - make rotation invisible: rotating the geo changes info.matrix but
    //     not the local points, so the wireframe wouldn't move
    // Reading info.matrix every draw means upstream knob changes propagate
    // automatically — Nuke's validate(true) refresh already gives us the
    // current matrix value.
    const unsigned num_objects = geos.objects();
    for (unsigned o = 0; o < num_objects; ++o) {
        const GeoInfo& info = geos[o];
        const PointList* pts = info.point_list();
        if (!pts) continue;

        // Compose: local-point → world → camera → clip in one matrix.
        // Pre-multiplying once per object avoids 24 redundant 4×4 multiplies
        // on a 6-quad cube (each vertex would otherwise be transformed
        // edge_count × 2 times during the line walk).
        //
        // When a pin solve is live, owner_->effective_model_matrix
        // returns the solved object pose instead of info.matrix — so the
        // wireframe overlay shifts to match the pins as you drag, even though
        // the upstream GeoOp hasn't moved yet (DAG mutation comes in 1C.4).
        // Only the first object gets the solve override; subsequent objects
        // use their upstream matrix as-is. Matches single-object pin model.
        const Matrix4 obj_to_world = (o == 0)
            ? owner_->effective_model_matrix(info.matrix, vframe)
            : info.matrix;
        const Matrix4 mvp = view_proj * obj_to_world;

        // Per-object projection lambda — captures `mvp`, `scale`, `cx`, `cy`.
        auto project = [&](const Vector3& v_local,
                           float& px, float& py) -> bool
        {
            const Vector4 v(v_local.x, v_local.y, v_local.z, 1.0f);
            const Vector4 clip = mvp * v;
            // w > 0 means in front of the camera in OpenGL/Nuke convention
            if (clip.w <= 0.001f) return false;
            const float ndc_x = clip.x / clip.w;
            const float ndc_y = clip.y / clip.w;
            px = ndc_x * scale + cx;
            py = ndc_y * scale + cy;
            return true;
        };

        // Axis-gradient setup. Half-extent of the LOCAL points per axis about the
        // mesh origin (0,0,0): the farthest |coord| along each axis. We map a
        // vertex's local coord into [0,1] centred on the origin (origin → 0.5),
        // so +X is red-bright / -X red-dark, etc. Because it's computed from LOCAL
        // coords, the colouring is locked to the object and rotates with it — the
        // orientation cue. A per-axis floor avoids divide-by-zero and stops a flat
        // axis from amplifying noise into full-range colour.
        float hx = 1.0f, hy = 1.0f, hz = 1.0f;
        if (gradient) {
            float ex = 0.0f, ey = 0.0f, ez = 0.0f;
            const unsigned np = pts->size();
            for (unsigned i = 0; i < np; ++i) {
                const Vector3& v = (*pts)[i];
                ex = std::max(ex, std::fabs(v.x));
                ey = std::max(ey, std::fabs(v.y));
                ez = std::max(ez, std::fabs(v.z));
            }
            if (ex > 1e-6f) hx = ex;
            if (ey > 1e-6f) hy = ey;
            if (ez > 1e-6f) hz = ez;
        }
        auto grad_color = [&](const Vector3& v) {
            const float r = 0.5f + 0.5f * std::max(-1.0f, std::min(1.0f, v.x / hx));
            const float g = 0.5f + 0.5f * std::max(-1.0f, std::min(1.0f, v.y / hy));
            const float b = 0.5f + 0.5f * std::max(-1.0f, std::min(1.0f, v.z / hz));
            glColor4f(r, g, b, 1.0f);
        };

        const unsigned num_prims = info.primitives();
        for (unsigned p = 0; p < num_prims; ++p) {
            const Primitive* prim = info.primitive(p);
            if (!prim) continue;

            const unsigned num_faces = prim->faces();
            for (unsigned f = 0; f < num_faces; ++f) {
                const unsigned nv = prim->face_vertices((int)f);
                if (nv < 2) continue;

                // Fill local vertex indices for this sub-face. Stack-buffer
                // for small faces (cube quads = 4 verts) to avoid heap thrash.
                unsigned small_buf[32];
                std::vector<unsigned> heap_buf;
                unsigned* face_idx = small_buf;
                if (nv > 32) {
                    heap_buf.resize(nv);
                    face_idx = heap_buf.data();
                }
                prim->get_face_vertices((int)f, face_idx);

                // Emit each edge of the face as a line pair, closing back
                // to vertex 0 to draw a closed polygon outline.
                for (unsigned i = 0; i < nv; ++i) {
                    const unsigned local_a = face_idx[i];
                    const unsigned local_b = face_idx[(i + 1) % nv];

                    // Local face index → primitive's vertex array → global
                    // point index → Vector3 from the point list.
                    const unsigned global_a = prim->vertex(local_a);
                    const unsigned global_b = prim->vertex(local_b);

                    const Vector3& pa = (*pts)[global_a];
                    const Vector3& pb = (*pts)[global_b];

                    float ax, ay, bx, by;
                    if (project(pa, ax, ay) && project(pb, bx, by)) {
                        // Flat colour was set once before glBegin; in gradient mode
                        // each endpoint carries its own local-position colour.
                        if (gradient) { grad_color(pa); glVertex2f(ax, ay);
                                        grad_color(pb); glVertex2f(bx, by); }
                        else          { glVertex2f(ax, ay); glVertex2f(bx, by); }
                    }
                }
            }
        }
    }

#ifdef PCN_NEW_3D
    // -------- New-system (GeomOp / GeoCube) wireframe --------
    // The classic per-object loop above iterated an empty GeometryList for new
    // geo and drew nothing, so draw the extracted GeoMesh here, in the same
    // GL_LINES batch. GeoMesh stores fan TRIANGLES, so each triangle emits its 3
    // edges — a quad face therefore shows its diagonal (busier than the classic
    // polygon outline, expected for a triangulated proxy). object_to_world goes
    // through effective_model_matrix so a live solved pose moves the wireframe,
    // exactly like classic object 0.
    if (is_new_geo && new_gm.valid) {
        const Matrix4 obj_to_world =
            owner_->effective_model_matrix(new_gm.object_to_world, vframe);
        const Matrix4 mvp = view_proj * obj_to_world;

        auto projN = [&](float lx, float ly, float lz, float& px, float& py) -> bool {
            const Vector4 clip = mvp * Vector4(lx, ly, lz, 1.0f);
            if (clip.w <= 0.001f) return false;
            px = (clip.x / clip.w) * scale + cx;
            py = (clip.y / clip.w) * scale + cy;
            return true;
        };

        // Gradient half-extents from LOCAL verts (orientation cue, rotates with mesh).
        float hx = 1.0f, hy = 1.0f, hz = 1.0f;
        if (gradient) {
            float ex = 0.0f, ey = 0.0f, ez = 0.0f;
            for (Eigen::Index i = 0; i < new_gm.local_vertices.rows(); ++i) {
                ex = std::max(ex, std::fabs(new_gm.local_vertices(i, 0)));
                ey = std::max(ey, std::fabs(new_gm.local_vertices(i, 1)));
                ez = std::max(ez, std::fabs(new_gm.local_vertices(i, 2)));
            }
            if (ex > 1e-6f) hx = ex;
            if (ey > 1e-6f) hy = ey;
            if (ez > 1e-6f) hz = ez;
        }
        auto gcolN = [&](float x, float y, float z) {
            glColor4f(0.5f + 0.5f * std::max(-1.0f, std::min(1.0f, x / hx)),
                      0.5f + 0.5f * std::max(-1.0f, std::min(1.0f, y / hy)),
                      0.5f + 0.5f * std::max(-1.0f, std::min(1.0f, z / hz)), 1.0f);
        };
        // Flat colour was already set once before glBegin (when !gradient).

        const Eigen::Index ntri = new_gm.triangles.rows();
        for (Eigen::Index ti = 0; ti < ntri; ++ti) {
            const unsigned ia = new_gm.triangles(ti, 0);
            const unsigned ib = new_gm.triangles(ti, 1);
            const unsigned ic = new_gm.triangles(ti, 2);
            const float ax_l = new_gm.local_vertices(ia, 0), ay_l = new_gm.local_vertices(ia, 1), az_l = new_gm.local_vertices(ia, 2);
            const float bx_l = new_gm.local_vertices(ib, 0), by_l = new_gm.local_vertices(ib, 1), bz_l = new_gm.local_vertices(ib, 2);
            const float cx_l = new_gm.local_vertices(ic, 0), cy_l = new_gm.local_vertices(ic, 1), cz_l = new_gm.local_vertices(ic, 2);

            float pax, pay, pbx, pby, pcx, pcy;
            const bool oa = projN(ax_l, ay_l, az_l, pax, pay);
            const bool ob = projN(bx_l, by_l, bz_l, pbx, pby);
            const bool oc = projN(cx_l, cy_l, cz_l, pcx, pcy);

            if (oa && ob) {
                if (gradient) gcolN(ax_l, ay_l, az_l); glVertex2f(pax, pay);
                if (gradient) gcolN(bx_l, by_l, bz_l); glVertex2f(pbx, pby);
            }
            if (ob && oc) {
                if (gradient) gcolN(bx_l, by_l, bz_l); glVertex2f(pbx, pby);
                if (gradient) gcolN(cx_l, cy_l, cz_l); glVertex2f(pcx, pcy);
            }
            if (oc && oa) {
                if (gradient) gcolN(cx_l, cy_l, cz_l); glVertex2f(pcx, pcy);
                if (gradient) gcolN(ax_l, ay_l, az_l); glVertex2f(pax, pay);
            }
        }
    }
#endif

    glEnd();

    // ---- Pin drawing pass ----
    // Pins are anchored to mesh vertices (vertex_idx). EVERY pin is drawn at
    // the projection of its vertex through the effective (solved) pose — the
    // same matrix the wireframe uses — so the dot rides its vertex and stays
    // glued to the wireframe as the geo moves under a drag. Rotation,
    // animation, and per-object transforms propagate for free.
    //
    // The dragged pin's stored target (target_x_px/py) is NOT drawn: it's only
    // a transient input to the solve (update.pos). Drawing it directly is what
    // used to strand the dot in space when the geo didn't fully reach it. The
    // pin currently being dragged is coloured red, the rest amber.
    //
    // Image-pixel coords go straight to glVertex2f since the wireframe knob
    // is already drawing in that coord space (the viewer's MV/PJ apply the
    // pan+zoom for us).
    if (owner_->manipulator_mode() != 0 && !owner_->pins().empty()) {
        GeoMesh pin_gm;
        const bool have_pin_geo = extract_mesh(geo_op, pin_gm);
        Matrix4 pin_mvp;
        Eigen::Index nverts = 0;
        if (have_pin_geo) {
            // Pin anchors project through the SOLVED model
            // matrix when a solve is live — so amber dots follow the solve
            // along with the wireframe.
            const Matrix4 effective_obj_to_world =
                owner_->effective_model_matrix(pin_gm.object_to_world, vframe);
            pin_mvp = view_proj * effective_obj_to_world;
            nverts  = pin_gm.local_vertices.rows();
        }

        // When pin-edit is armed, pins are the active editor —
        // draw them larger and brighter (and the gizmo draws dimmed) so the
        // viewer reads at a glance which tool the mouse drives.
        const bool armed = owner_->pin_input_active();

        const std::vector<Pin>& pin_list = owner_->pins();
        const int active_slot = owner_->dragging_pin_slot();
        const int cur_frame   = (int)std::floor(vframe + 0.5);

        // ---- Pin dots (only). One per vertex, riding the wireframe. ----
        glEnable(GL_POINT_SMOOTH);
        glPointSize(armed ? 14.0f : 10.0f);

        // Collect (screen pos, label) so the index numbers can be stroked AFTER
        // the GL_POINTS batch (GL_LINES can't be nested inside GL_POINTS).
        struct PinLbl { float x, y; int n; };
        std::vector<PinLbl> pin_labels;

        glBegin(GL_POINTS);
        for (size_t pslot = 0; pslot < pin_list.size(); ++pslot) {
            const Pin& p = pin_list[pslot];
            if (!have_pin_geo) continue;
            if ((Eigen::Index)p.vertex_idx >= nverts) continue;
            // Drawn only when the pin is ACTIVE on this frame: a manual KEY here, or a
            // LINKED track seen here (resolve_pin_2d with interpolation off). The dot
            // rides the vertex projection through the SOLVED pose, so it stays glued to
            // the wireframe and follows the body as you drag. Free Pin Move ignores the
            // link, so an un-keyed linked pin is HIDDEN on this frame (like a hand-keyed
            // pin), and only the pins you've actually placed here are shown.
            if (!owner_->resolve_pin_2d(p, cur_frame, fmt_h, /*allow_interp=*/false)) continue;
            const bool linked_here = (p.linked_track >= 0);

            const float lx = pin_gm.local_vertices((Eigen::Index)p.vertex_idx, 0);
            const float ly = pin_gm.local_vertices((Eigen::Index)p.vertex_idx, 1);
            const float lz = pin_gm.local_vertices((Eigen::Index)p.vertex_idx, 2);
            const Vector4 clip = pin_mvp * Vector4(lx, ly, lz, 1.0f);
            if (clip.w <= 0.001f) continue;
            const float px_pin = (clip.x / clip.w) * scale + cx;
            const float py_pin = (clip.y / clip.w) * scale + cy;

            if ((int)pslot == active_slot && owner_->pin_drag_is_moving())
                glColor4f(1.0f, 0.0f, 0.0f, 1.0f);   // red = pin actively being moved
            else if (linked_here)
                glColor4f(0.20f, 0.80f, 1.0f, 1.0f); // cyan = linked to a track
            else if (armed)
                glColor4f(1.0f, 0.85f, 0.12f, 1.0f); // yellow = manual, armed/active
            else
                glColor4f(0.85f, 0.62f, 0.0f, 1.0f); // muted amber = manual, gizmo active

            glVertex2f(px_pin, py_pin);
            // 1-based label from the pin's STABLE id (survives delete gaps; this is
            // the id a pin<->track link would reference).
            pin_labels.push_back({px_pin, py_pin, (int)p.id + 1});
        }
        glEnd();

        // Pin index numbers, offset up-right of each dot, in amber.
        if (owner_->show_numbers() && !pin_labels.empty()) {
            const float lh  = std::max(9.0f, fmt_h * 0.013f);
            const float off = lh * 0.5f;
            glColor4f(1.0f, 0.85f, 0.15f, 1.0f);
            for (const PinLbl& L : pin_labels)
                draw_number_glyphs(L.n, L.x + off, L.y + off, lh);
        }
    }

    // ---- "PIN" HUD label when armed ----------------------------------------
    // Drawn independently of whether any pins exist yet, so the user gets clear
    // confirmation the mode is armed even before placing the first pin. Image-px
    // space, Y-up, anchored near the top-left of the format.
    if (owner_->pin_input_active()) {
        const float hud_h    = std::max(18.0f, fmt_h * 0.03f);
        const float hud_x    = fmt_w * 0.03f;
        const float hud_ytop = fmt_h * 0.95f;
        glDisable(GL_DEPTH_TEST);
        draw_pin_hud_glyphs(hud_x, hud_ytop, hud_h);
    }
    // ---- User-track overlay ("Show User Tracks") ---------------------------
    // For each anchored user track: a CROSS at its measured 2D position on this
    // frame, and a DOT where its mesh anchor projects through the solved pose. They
    // line up when the track sits on the object and the solve is good — the visual
    // proof. The anchor projects exactly like a wireframe vertex; the measured
    // position is the solver's y-down px flipped back to image-pixel Y-up (h - y).
    if (owner_->user_tracks_show()) {
        const UserTracks& uts = owner_->user_tracks_for_solve();
        if (!uts.empty()) {
            float uc[4];
            owner_->user_tracks_color(uc);

            GeoMesh ut_gm;
            Matrix4 ut_mvp;
            bool have_ut_geo = false;
            if (extract_mesh(geo_op, ut_gm)) {
                ut_mvp = view_proj *
                         owner_->effective_model_matrix(ut_gm.object_to_world, vframe);
                have_ut_geo = true;
            }

            // Anchor dots (where each track's mesh point projects right now).
            if (have_ut_geo) {
                glColor4f(uc[0], uc[1], uc[2], uc[3]);
                glPointSize(8.0f);
                glEnable(GL_POINT_SMOOTH);

                // Collect (screen pos, 1-based load-order index) for the numbers,
                // stroked after the GL_POINTS batch.
                struct UtLbl { float x, y; int n; };
                std::vector<UtLbl> ut_labels;

                glBegin(GL_POINTS);
                int tnum = 0;
                for (const UserTrack& t : uts) {
                    ++tnum;   // 1-based index in the anchored (load-order) list
                    const Vector4 clip = ut_mvp * Vector4(t.object_point.x(),
                                                          t.object_point.y(),
                                                          t.object_point.z(), 1.0f);
                    if (clip.w <= 0.001f) continue;
                    const float upx = (clip.x / clip.w) * scale + cx;
                    const float upy = (clip.y / clip.w) * scale + cy;
                    glVertex2f(upx, upy);
                    ut_labels.push_back({upx, upy, tnum});
                }
                glEnd();
                glPointSize(1.0f);

                // Track index numbers, offset up-right of each anchor dot.
                if (owner_->show_numbers() && !ut_labels.empty()) {
                    const float lh  = std::max(9.0f, fmt_h * 0.013f);
                    const float off = lh * 0.5f;
                    glColor4f(uc[0], uc[1], uc[2], uc[3]);
                    for (const UtLbl& L : ut_labels)
                        draw_number_glyphs(L.n, L.x + off, L.y + off, lh);
                }
            }

            // Measured-position crosses on the current frame.
            const int cur_ut_frame = (int)std::floor(vframe + 0.5);
            const float r = std::max(5.0f, fmt_h * 0.006f);
            glColor4f(uc[0], uc[1], uc[2], uc[3]);
            glLineWidth(1.5f);
            glBegin(GL_LINES);
            for (const UserTrack& t : uts) {
                for (const UserTrackObservation& o : t.observations) {
                    if (o.frame_id != cur_ut_frame) continue;
                    const float px = o.image_point.x();
                    const float py = fmt_h - o.image_point.y();   // y-down -> image Y-up
                    glVertex2f(px - r, py); glVertex2f(px + r, py);
                    glVertex2f(px, py - r); glVertex2f(px, py + r);
                    break;   // one observation per track per frame
                }
            }
            glEnd();
        }
    }

    // Anchored at the proxy origin, drawn after the wireframe/pin passes.
    // draw_gizmo owns its GL attrib stack and no-ops in "Pins" mode.
    {
        GeoMesh giz_gm;
        if (extract_mesh(geo_op, giz_gm)) {
            const Matrix4 giz_obj_to_world =
                owner_->effective_model_matrix(giz_gm.object_to_world, vframe);
            // Pivot at the mesh bounding-box centre (local space).
            Vector3 pivot_local(0.0f, 0.0f, 0.0f);
            const Eigen::Index nv = giz_gm.local_vertices.rows();
            if (nv > 0) {
                float mnx=1e30f,mny=1e30f,mnz=1e30f,mxx=-1e30f,mxy=-1e30f,mxz=-1e30f;
                for (Eigen::Index i=0;i<nv;++i) {
                    const float x=giz_gm.local_vertices(i,0), y=giz_gm.local_vertices(i,1), z=giz_gm.local_vertices(i,2);
                    mnx=std::min(mnx,x); mny=std::min(mny,y); mnz=std::min(mnz,z);
                    mxx=std::max(mxx,x); mxy=std::max(mxy,y); mxz=std::max(mxz,z);
                }
                pivot_local = Vector3(0.5f*(mnx+mxx), 0.5f*(mny+mxy), 0.5f*(mnz+mxz));
            }
            owner_->draw_gizmo(view_proj, giz_obj_to_world, cam, scale, cx, cy, pivot_local);
        }
    }

    glPopAttrib();
}

} // namespace pcn