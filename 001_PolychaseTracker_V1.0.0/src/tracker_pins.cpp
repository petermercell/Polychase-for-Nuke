// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Peter Mercell
//
// Developed with assistance from Claude (Anthropic).

// tracker_pins.cpp — part of the PolychaseTracker plugin (see polychase_tracker.h).
#include "polychase_tracker.h"

#include "pcn_convention.h"   // conv::apply_curve_focal (stale-focal workaround)
#include "pnp/solvers.h"   // PnPResult, PnPOptions, SolvePnPIterative (weighted PnP)

#include <GL/gl.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>

using namespace DD::Image;

namespace pcn {

// -----------------------------------------------------------------------------
// Per-frame pins (frame scoping).
//
// Each pin belongs to the frame it was created on (Pin::created_frame). Only the
// pins for the CURRENT frame are drawn, hit-tested, and fed to the solve — so a
// pin you placed at frame 1 (target = a frame-1 pixel) no longer fights the
// frame-50 solve where the cube has moved. This matches the keyframe-anchor
// workflow: pin-align each anchor frame independently, then TrackIt between.
//
// frame_key() rounds a Nuke frame (double) to the int used for comparison and
// storage, so display (uiContext) and solve (outputContext) bucket identically.
namespace {
inline int frame_key(double f) { return (int)std::floor(f + 0.5); }
}


// editing_frame — the frame the user is actually editing on. Prefer the UI/
// playhead frame the overlay knob pushed in (uiContext().frame(), which tracks
// the playhead exactly), and fall back to the rounded Op context when no viewer
// has drawn yet. Every place that used frame_key(outputContext().frame()) for a
// mouse/key-driven action now routes through here, fixing the "scrub to a new
// frame and you can't place a pin until you toggle Pin Edit" bug.
int PolychaseTracker::editing_frame() const
{
    return has_editing_frame_ ? editing_frame_
                              : frame_key(outputContext().frame());
}


// -----------------------------------------------------------------------------
// Mouse event handlers.
//
// PUSH    → on_mouse_push:    if click is on an existing pin, start dragging
//                              that pin. Otherwise try to place a new pin via
//                              vertex-snap.
// DRAG    → on_mouse_drag:    if dragging, update the pin's target_x/y to
//                              follow the cursor. Mutates pins_ in place and
//                              triggers a viewer redraw, but does NOT call
//                              sync_blob_from_pins (that would create dozens
//                              of undo entries per drag).
// RELEASE → on_mouse_release: if we were dragging, sync_blob_from_pins once
//                              to capture a single undo entry for the whole
//                              drag.
//
// Why vertex-snap for placement? See the kept comment block below.
// Why hit-test on draw position (target_x/y when set, else projection)?
// Because the user sees the pin at that position, so that's what they're
// clicking on. Hit-testing the anchor projection would feel wrong after
// a drag has moved the dot away from the underlying vertex.
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// resolve_pin_2d — the single source of a pin's 2D screen position at a frame.
// Order: manual key wins, else the linked track, else interpolation between the
// pin's manual keys, else none. Result is Nuke Y-UP pixels.
// -----------------------------------------------------------------------------
std::optional<Eigen::Vector2f>
PolychaseTracker::resolve_pin_2d(const Pin& pin, int frame, float fmt_h, bool allow_interp)
{
    // 1. Manual key on this exact frame — always wins.
    const auto k = pin.keys.find(frame);
    if (k != pin.keys.end()) return k->second;

    // 2. Linked user track seen at this frame (track store is Y-DOWN -> Y-UP).
    if (pin.linked_track >= 0) {
        const UserTracks& uts = user_tracks_for_solve();
        if ((size_t)pin.linked_track < uts.size()) {
            for (const UserTrackObservation& ob : uts[(size_t)pin.linked_track].observations)
                if (ob.frame_id == frame)
                    return Eigen::Vector2f(ob.image_point.x(), fmt_h - ob.image_point.y());
        }
    }

    // 3. Interpolate between the pin's bracketing manual keys (pure hand-track).
    if (allow_interp && !pin.keys.empty()) {
        auto hi = pin.keys.lower_bound(frame);
        if (hi == pin.keys.begin()) return hi->second;            // before first key
        if (hi == pin.keys.end())   return std::prev(hi)->second; // after last key
        const auto lo = std::prev(hi);
        const float span = (float)(hi->first - lo->first);
        const float t    = span > 0.0f ? (float)(frame - lo->first) / span : 0.0f;
        return lo->second + t * (hi->second - lo->second);
    }

    // 4. No 2D here.
    return std::nullopt;
}


void PolychaseTracker::on_mouse_push(DD::Image::ViewerContext* ctx,
                                     const double mv[16], const double pj[16],
                                     const int vp[4], bool gl_state_valid)
{
    if (!gl_state_valid) {
        PCN_LOG("[PolychaseTracker] PUSH: GL state not yet captured "
                     "(viewer must draw at least once before clicking)\n");
        return;
    }

    ensure_pins_loaded();

    // Pin-refine routing. The arm flag decides who
    // gets the mouse — there is no gizmo-handle-vs-pin-dot arbitration.
    //
    //   disarmed → the gizmo gets the click (coarse placement).
    //   armed    → the pins get the click: drag an existing pin, else place a
    //              new one on a vertex and drag it in the same motion.
    if (!pin_input_active()) {
        on_gizmo_push(ctx, mv, pj, vp);   // gizmo_hit_test no-ops while armed anyway
        return;
    }

    // ---- Armed: pins ----
    PCN_LOG("[push] DIAG armed: editing_frame=" << editing_frame()
              << " pins=" << pins_.size() << " next_id=" << next_pin_id_
              << " live_scene=" << (live_scene_ ? "yes" : "no") << "\n");
    // A PUSH is always the start of a NEW gesture. If a previous drag's RELEASE
    // was ever dropped (focus change, a mid-drag P toggle, or a place-and-drag
    // that never saw its release), dragging_pin_idx_ / drag_anchor_ could still
    // be set. Clear them here so a stranded gesture can't wedge the next drag or
    // leave run_pin_solve warm-started from a stale frozen anchor.
    if (dragging_pin_idx_ >= 0) {
        // A previous drag never saw its RELEASE (and no MOVE caught it either).
        // Commit it now — keys the pose, syncs offsets, persists pins — so its move
        // is recorded, before this new gesture begins. on_mouse_release clears
        // dragging_pin_idx_ / drag_anchor_ for us.
        on_mouse_release(ctx, mv, pj, vp, gl_state_valid);
    }

    // Move-history: capture the pre-operation state (offsets + current pin set)
    // BEFORE we place/grab a pin, so Undo can step back to "before this pin".
    arm_and_seed_move_baseline();
    pin_drag_moved_ = false;   // a fresh gesture hasn't moved anything yet

    const int hit = find_pin_under_cursor(ctx->mouse_x(), ctx->mouse_y(),
                                          mv, pj, vp);
    if (hit >= 0) {
        dragging_pin_idx_ = hit;          // existing pin → drag it (the mover)
        PCN_LOG("[PolychaseTracker] picked up pin #"
                  << pins_[hit].id << " (slot " << hit << ")\n");
        asapUpdate();
        return;
    }

    // Empty vertex → place a new pin. Already-pinned vertex that isn't keyed on
    // this frame → key it here and grab it (re-activate the same vertex on a new
    // frame). Either way try_place_new_pin returns the pin index to drag, or -1.
    const int placed = try_place_new_pin(ctx, mv, pj, vp);
    if (placed >= 0) {
        // Commit the just-placed/re-keyed pin as its own undo step BEFORE the
        // place-and-drag motion moves it, so undoing the move leaves the pin in
        // place (undo again removes it).
        record_move_snapshot("pin placed");
        dragging_pin_idx_ = placed;
    }
}


void PolychaseTracker::on_mouse_drag(DD::Image::ViewerContext* ctx,
                                     const double mv[16], const double pj[16],
                                     const int vp[4], bool gl_state_valid)
{
    // Discriminate on which drag is actually in progress rather than on the arm
    // flag: a pin drag can only have started while armed (on_mouse_push), and a
    // gizmo drag only while disarmed, so this also survives a mid-drag P toggle.
    if (dragging_pin_idx_ >= 0) {
        if (!gl_state_valid) return;
        if (dragging_pin_idx_ >= (int)pins_.size()) {
            dragging_pin_idx_ = -1;
            return;
        }

        // Convert mouse (Nuke screen coords, Y-top-down) → image-pixel coords
        double img_x = 0.0, img_y = 0.0;
        if (!mouse_to_image_pixel(mv, pj, vp,
                                  ctx->mouse_x(), ctx->mouse_y(),
                                  img_x, img_y)) {
            return;
        }

        Pin& pin = pins_[dragging_pin_idx_];
        pin.target_x_px        = (float)img_x;   // mover target for run_pin_solve
        pin.target_y_px        = (float)img_y;
        pin.is_target_user_set = true;
        // New model: dragging keys THIS pin's 2D on the current frame...
        pin.keys[editing_frame()] = Eigen::Vector2f((float)img_x, (float)img_y);
        // ...and then re-solves the rigid body LIVE: the dragged pin leads toward the
        // cursor. In NORMAL mode every OTHER keyed pin holds at kAnchorPinWeight, so
        // the body fits all pins together. In FREE PIN MOVE the other pins' weights
        // are dropped to zero inside run_pin_solve, so ONLY the dragged pin leads —
        // the cube follows that one handle and the rest ride along without pulling
        // back (the synthetic prior keeps the remaining DOFs stable). Either way the
        // dot rides the vertex projection through the freshly solved pose.
        run_pin_solve(dragging_pin_idx_);

        pin_drag_moved_ = true;   // the gesture has moved a pin → release re-fits
        // Do NOT sync_blob_from_pins() here — one undo entry per drag, on RELEASE.
        last_pin_drag_tp_ = std::chrono::steady_clock::now();   // for the red/amber motion cue
        asapUpdate();
        return;
    }

    // Otherwise: a gizmo drag (disarmed). on_gizmo_drag also early-outs if armed.
    if (gizmo_dragging()) {
        if (gl_state_valid) on_gizmo_drag(ctx, mv, pj, vp);
        return;
    }
}

// A pin counts as "moving" only while a drag tick landed very recently. This lets
// the viewer draw the pin red while it is actually being dragged and amber the
// moment it comes to rest (a held-still pause or a finished drag), independent of
// whether dragging_pin_idx_ has been cleared yet.
bool PolychaseTracker::pin_drag_is_moving() const {
    if (dragging_pin_idx_ < 0) return false;
    return (std::chrono::steady_clock::now() - last_pin_drag_tp_)
           < std::chrono::milliseconds(250);
}


void PolychaseTracker::on_mouse_release(DD::Image::ViewerContext* /*ctx*/,
                                        const double /*mv*/[16],
                                        const double /*pj*/[16],
                                        const int    /*vp*/[4],
                                        bool /*gl_state_valid*/)
{
    // A pin drag (started while armed) always completes here, regardless of the
    // current arm state, so a mid-drag P toggle can't strand it. The commit below is
    // made undoable by journaling the pose into live_pose_blob and the pins into
    // pins_blob — both writable String_knobs, whose set_text() Nuke 17 DOES capture
    // for undo from a viewer event (unlike the DO_NOT_WRITE offset knobs).
    if (dragging_pin_idx_ >= 0) {
        if (dragging_pin_idx_ < (int)pins_.size()) {
            const Pin& pin = pins_[dragging_pin_idx_];
            PCN_LOG("[PolychaseTracker] dropped pin #" << pin.id
                      << " at image-pixel (" << pin.target_x_px
                      << ", " << pin.target_y_px << ")\n");
        }
        dragging_pin_idx_ = -1;
        drag_anchor_.reset();   // unfreeze; next grab re-anchors to the live pose

        // Only re-fit/key if the gesture actually moved a pin. A bare click just
        // grabs/releases and must not key a pose or persist anything.
        if (!pin_drag_moved_) { asapUpdate(); return; }

        // The drag already solved the body LIVE (run_pin_solve) into live_scene_;
        // just key THAT pose at this frame. Old-code behaviour: what you dragged is
        // exactly what gets keyed, no extra re-balancing on release.
        key_pose_at_current_frame();
        // Reflect the pin-solved pose in the Translate/Dolly/Rotate knobs (same
        // inverse map the gizmo uses), so the readout follows the pins.
        if (live_scene_) sync_offsets_from_pose(live_scene_->model_matrix);
        // Make the drag UNDOABLE: journal live_scene_ into the writable live_pose_blob
        // (set_text is captured by Nuke 17 undo from a viewer event — the same way
        // pins_blob is). Both String_knob writes land in this one mouse-up action, so a
        // single Ctrl+Z reverts them together: on_live_pose_blob_changed() rebuilds
        // live_scene_ (unsticks the wireframe) AND the offset readout, while
        // on_pins_blob_changed() restores the pin positions.
        sync_blob_from_live_pose();
        // Persist the pin set.
        sync_blob_from_pins();
        return;
    }

    // Otherwise: finish a gizmo drag (keys the pose).
    if (gizmo_dragging()) { on_gizmo_release(); return; }
}


// -----------------------------------------------------------------------------
// Hit-test: find the pin nearest the cursor in screen pixels, return -1 if
// none within HIT_RADIUS_PX. Uses each pin's drawn position (target if user-
// set, anchor projection otherwise) so what the user clicks matches what
// they see.
// -----------------------------------------------------------------------------
int PolychaseTracker::find_pin_under_cursor(int mouse_x_nuke,
                                            int mouse_y_nuke_topdown,
                                            const double mv[16],
                                            const double pj[16],
                                            const int vp[4])
{
    using namespace DD::Image;

    CameraOp* cam = input_cam();
    Op*       geo = input_geo_op();
    Iop*      img = input_img();
    if (!cam || !geo) return -1;
    if (pins_.empty())  return -1;

    cam->validate(true);
    geo->validate(true);
    if (img) img->validate(true);

    // Image format → NDC→pixel scale (must match wireframe draw)
    float fmt_w = 2048.0f, fmt_h = 1080.0f;
    if (img) {
        const Format& fmt = img->info().format();
        fmt_w = (float)fmt.width();
        fmt_h = (float)fmt.height();
    }
    const float scale = fmt_w * 0.5f;
    const float cx    = fmt_w * 0.5f;
    const float cy    = fmt_h * 0.5f;

    // Mouse → GL viewport convention (Y up, 0 at bottom)
    const float mx_screen = (float)mouse_x_nuke;
    const float my_screen = (float)(vp[3] - mouse_y_nuke_topdown);

    // Need geometry to project anchor positions for non-user-set pins
    GeoMesh gm;
    const bool have_geo = extract_mesh(geo, gm);
    Matrix4 mvp;
    if (have_geo) {
        // Hit-test against where pins are DRAWN, which uses the solved
        // pose when one exists. Anchor dots should be clickable at their
        // visible position.
        const Matrix4 effective_obj = effective_model_matrix(gm.object_to_world);
        mvp = cam->projection() * cam->imatrix() * effective_obj;
    }

    constexpr float HIT_RADIUS_PX = 14.0f;  // grab tolerance — be generous
    int   best_idx = -1;
    float best_d2  = HIT_RADIUS_PX * HIT_RADIUS_PX;
    const int cur_frame = editing_frame();

    for (size_t i = 0; i < pins_.size(); ++i) {
        const Pin& pin = pins_[i];
        if (!have_geo) continue;
        const Eigen::Index nverts = gm.local_vertices.rows();
        if ((Eigen::Index)pin.vertex_idx >= nverts) continue;
        // Only pins active on this frame are drawn, so only they are grabbable
        // (matches the overlay): a manual key here, or a linked track seen here.
        // Free Pin Move ignores the link, so an un-keyed linked pin is inert here
        // too — you re-activate it by clicking its vertex (try_place_new_pin keys
        // and grabs it), exactly like a hand-keyed pin.
        if (!resolve_pin_2d(pin, cur_frame, fmt_h, /*allow_interp=*/false)) continue;

        const Vector4 clip = mvp * Vector4(gm.local_vertices((Eigen::Index)pin.vertex_idx, 0),
                                           gm.local_vertices((Eigen::Index)pin.vertex_idx, 1),
                                           gm.local_vertices((Eigen::Index)pin.vertex_idx, 2),
                                           1.0f);
        if (clip.w <= 0.001f) continue;
        const double image_px = (double)((clip.x / clip.w) * scale + cx);
        const double image_py = (double)((clip.y / clip.w) * scale + cy);

        // image-pixel → viewer screen pixel
        double sx = 0.0, sy = 0.0;
        if (!project_image_to_screen_gl(mv, pj, vp, image_px, image_py, sx, sy)) continue;

        const float dx = (float)sx - mx_screen;
        const float dy = (float)sy - my_screen;
        const float d2 = dx*dx + dy*dy;
        if (d2 < best_d2) {
            best_d2  = d2;
            best_idx = (int)i;
        }
    }
    return best_idx;
}


// -----------------------------------------------------------------------------
// try_place_new_pin: vertex-snap pin placement.
//
// Vertex-snap mode (per Peter's UX choice):
//   - Click anywhere in the 2D viewer
//   - Find the projected vertex closest (in screen pixels) to the click
//   - If within a snap threshold, place a pin anchored to that vertex
//
// Why screen-space distance? Because mouse_x()/mouse_y() are in viewer
// widget pixels, not image pixels. The viewer applies its own pan+zoom
// transform via the GL_MODELVIEW + GL_PROJECTION matrices, so a fixed image
// pixel maps to a different screen pixel depending on zoom. We capture that
// state at the end of each DRAW_OPAQUE pass and apply it here so the
// comparison is apples-to-apples.
//
// Why vertex-snap instead of ray-cast? The original ctx->x()/ctx->y()
// approach was returning (0, 0) because those accessors only carry data
// for POSITION-type handles registered via begin_handle(). Switching to
// mouse_x/mouse_y gives us real coords, but they're in screen space, so
// snapping is the simplest way to handle that.
//
// Returns true if a pin was placed.
// -----------------------------------------------------------------------------
int PolychaseTracker::try_place_new_pin(DD::Image::ViewerContext* ctx,
                                         const double mv[16],
                                         const double pj[16],
                                         const int    vp[4])
{
    using namespace DD::Image;

    // Pin cap is GLOBAL now (one pin per vertex, persistent across all frames).
    const int cur_frame = editing_frame();
    if (pins_.size() >= (size_t)kMaxPinSlots) {
        PCN_LOG("[PolychaseTracker] place_pin: pin limit reached ("
                  << kMaxPinSlots << " max). Delete a pin to place another.\n");
        return -1;
    }

    CameraOp* cam = input_cam();
    Op*       geo = input_geo_op();
    Iop*      img = input_img();
    if (!cam || !geo) return -1;

    cam->validate(true);
    geo->validate(true);
    if (img) img->validate(true);

    // Image format → NDC→pixel scale (matches wireframe draw)
    float fmt_w = 2048.0f, fmt_h = 1080.0f;
    if (img) {
        const Format& fmt = img->info().format();
        fmt_w = (float)fmt.width();
        fmt_h = (float)fmt.height();
    }
    const float scale = fmt_w * 0.5f;
    const float cx    = fmt_w * 0.5f;
    const float cy    = fmt_h * 0.5f;

    // Mouse → GL viewport Y-up convention
    const float mx_screen = (float)ctx->mouse_x();
    const float my_screen = (float)(vp[3] - ctx->mouse_y());

    GeoMesh gm;
    if (!extract_mesh(geo, gm)) {
        PCN_LOG("[PolychaseTracker] place_pin: no geometry to snap to\n");
        return -1;
    }

    // Snap against where the wireframe is DRAWN — under live
    // solve that's effective_model_matrix, not gm.object_to_world. Without
    // this, clicking on a visible vertex (post-solve) would miss because the
    // snap target is the UN-solved position.
    const Matrix4 effective_obj = effective_model_matrix(gm.object_to_world);
    const Matrix4 mvp = cam->projection() * cam->imatrix() * effective_obj;

    int    nearest_idx     = -1;
    float  best_dist_sq_px = 1e10f;
    double nearest_image_px = 0.0;
    double nearest_image_py = 0.0;
    const Eigen::Index nverts = gm.local_vertices.rows();

    for (Eigen::Index i = 0; i < nverts; ++i) {
        const Vector4 clip = mvp * Vector4(gm.local_vertices(i, 0),
                                           gm.local_vertices(i, 1),
                                           gm.local_vertices(i, 2),
                                           1.0f);
        if (clip.w <= 0.001f) continue;
        const float ndc_x = clip.x / clip.w;
        const float ndc_y = clip.y / clip.w;
        const double image_px = (double)(ndc_x * scale + cx);
        const double image_py = (double)(ndc_y * scale + cy);

        double sx = 0.0, sy = 0.0;
        if (!project_image_to_screen_gl(mv, pj, vp, image_px, image_py, sx, sy)) continue;

        const float dx = (float)sx - mx_screen;
        const float dy = (float)sy - my_screen;
        const float d_sq = dx*dx + dy*dy;
        if (d_sq < best_dist_sq_px) {
            best_dist_sq_px = d_sq;
            nearest_idx     = (int)i;
            nearest_image_px = image_px;
            nearest_image_py = image_py;
        }
    }

    const float SNAP_RADIUS_PX = 40.0f;

    if (nearest_idx < 0) {
        PCN_LOG("[PolychaseTracker] place_pin: no visible vertices\n");
        return -1;
    }
    const float best_dist_px = std::sqrt(best_dist_sq_px);
    if (best_dist_px > SNAP_RADIUS_PX) {
        PCN_LOG("[PolychaseTracker] place_pin: nearest vertex (#"
                  << nearest_idx << ") is " << best_dist_px
                  << " px away — outside " << SNAP_RADIUS_PX << " px snap radius\n");
        return -1;
    }

    // One pin per VERTEX — the pin IS the vertex (new model). Clicking a vertex
    // that already has a pin is a no-op (drag that pin to key it on this frame, or
    // it just gets a 2D key when you move it). The per-frame duplication of the
    // old model is gone: a single pin carries the vertex across all frames via its
    // key curve / linked track.
    for (size_t ei = 0; ei < pins_.size(); ++ei) {
        Pin& existing = pins_[ei];
        if (existing.vertex_idx == (unsigned)nearest_idx) {
            // The vertex already has a pin. If it isn't keyed on THIS frame yet,
            // key it here (at its current projection) so it becomes an active
            // this-frame pin — the new-model way to "place" the same vertex on a
            // new frame (the old model made a separate per-frame pin). Then grab it
            // so the same click can place-and-drag. If it's already keyed here,
            // find_pin_under_cursor would have grabbed it; we still return it.
            if (existing.keys.find(cur_frame) == existing.keys.end()) {
                existing.keys[cur_frame] =
                    Eigen::Vector2f((float)nearest_image_px, (float)nearest_image_py);
                existing.target_x_px        = (float)nearest_image_px;
                existing.target_y_px        = (float)nearest_image_py;
                existing.is_target_user_set = false;
                sync_blob_from_pins();
                PCN_LOG("[PolychaseTracker] vertex " << nearest_idx
                          << " pin#" << existing.id << " keyed on frame " << cur_frame << "\n");
            }
            return (int)ei;
        }
    }

    Pin p;
    p.id                 = next_pin_id_++;
    p.vertex_idx         = (unsigned)nearest_idx;
    p.linked_track       = -1;            // unlinked until Connect/auto-link
    // Key the pin on THIS frame at its vertex projection. That makes it an active
    // this-frame pin straight away (drawn, grabbable, and a solve anchor), exactly
    // like the old model's created_frame==cur_frame pin — but carried by the key so
    // the same pin persists across frames. A place-and-drag immediately overwrites
    // this key with the cursor; an untouched placement just resists drift here.
    p.keys[cur_frame]    = Eigen::Vector2f((float)nearest_image_px, (float)nearest_image_py);
    p.created_frame      = cur_frame;     // legacy: seeds the transitional solve
    p.target_x_px        = (float)nearest_image_px;
    p.target_y_px        = (float)nearest_image_py;
    p.is_target_user_set = false;
    pins_.push_back(p);

    PCN_LOG("[PolychaseTracker] pin #" << p.id
              << " on vertex " << nearest_idx
              << " (" << best_dist_px << " px from click)"
              << "  [pins=" << pins_.size()
              << " next_id=" << next_pin_id_ << "]\n");

    sync_blob_from_pins();
    return (int)pins_.size() - 1;
}


// -----------------------------------------------------------------------------
// Live pin-mode solve.
//
// ONE rigid path for every pin count — no 1/2/3+ dispatch, no lurch guard, no
// similarity fallback. The flow:
//
//   1. Resolve typed inputs and validate.
//   2. Extract first-object mesh; each placed pin → one object_point (local).
//   3. Build CameraIntrinsics from cam->projection() — same projection the
//      wireframe uses, so overlay and solve stay in lock-step.
//   4. Freeze `initial` on the first tick of a drag (drag_anchor_): it defines
//      the model_view the un-dragged pins project through AND their "stay"
//      targets. `current` is the warm-start (latest solved pose). view_matrix
//      and intrinsics always come from the live camera at this frame.
//   5. Targets + weights: un-dragged pins → projection through `initial` at
//      kAnchorPinWeight (resist but yield); the dragged pin → cursor pixel at
//      kDraggedPinWeight (leads). The finite mover:anchor ratio is the
//      stickiness knob — it never hard-locks, even with 3+ well-spread anchors.
//   6. Add a low-weight synthetic prior (>=3 well-spread mesh verts projected
//      through `current`) so the system is always well-posed: satisfies
//      SolvePnPIterative's CHECK_GE(rows,3), holds the DOFs 1-2 real pins miss,
//      and breaks the collinear-pin roll degeneracy.
//   7. solve_pins_weighted → SolvePnPIterative (loss TRIVIAL — every pin
//      trusted). Result is the new rigid object pose (R + t; no scale — the
//      pose's scale is owned by the gizmo/keyed curve).
//   8. Store result into live_scene_; viewer redraw picks it up via
//      effective_model_matrix() in WireframeKnob::draw_handle. On release the
//      pose is keyed (see on_mouse_release).
//
// Solver failure modes: the solve is wrapped in try/catch so degenerate inputs
// (e.g. a pin behind the camera) don't crash the plugin; on failure we leave
// the previous live_scene_ untouched.
// -----------------------------------------------------------------------------

namespace {

// Sample a camera's world->camera (imatrix) at an ARBITRARY frame, restoring its
// OutputContext afterwards. The focal sweep needs the camera basis per frame (the
// camera is animated), and unlike projection()'s focal, imatrix() DOES update with
// setOutputContext + validate(true) (known-focal tracking already locks on this
// moving camera, which would be impossible if imatrix were stale). Mirror of
// tracker_intrinsics.cpp::icp_cam_imatrix_at — kept file-local here so the sweep
// can call it without exposing the intrinsics-file helper.
DD::Image::Matrix4 icp_pin_cam_imatrix_at(DD::Image::CameraOp* cam, double frame)
{
    using namespace DD::Image;
    OutputContext orig = cam->outputContext();
    OutputContext c    = orig;
    c.setFrame(frame);
    cam->setOutputContext(c);
    cam->validate(true);
    const Matrix4 m = cam->imatrix();
    cam->setOutputContext(orig);
    cam->validate(true);
    return m;
}

// Permanent low-weight synthetic prior used by the rigid pin solve. Small enough
// that one real pin (weight kDraggedPinWeight) dominates the DOFs it observes,
// large enough to win where no real pin constrains (real-pin gradient ~0). Tune
// so 3+ well-spread real pins drive the prior's cost contribution under ~1%.
constexpr float kSynthAnchorWeight = 0.05f;

// Real-pin weights for the live drag.
//
// The dragged pin (the one under the cursor) must be able to MOVE the rigid pose
// against the other placed pins. The original 1000:1 anchor:mover split made the
// non-dragged pins HARD constraints — and three non-collinear hard points fully
// determine a 6-DOF rigid pose, so the 4th pin had zero leverage and the drag
// locked solid. Cause was geometric, not the synthetic prior.
//
// Fix: a finite mover:anchor ratio. The dragged pin leads at kDraggedPinWeight;
// every other placed pin resists at kAnchorPinWeight but yields. The system is a
// well-posed weighted least-squares fit at ANY pin count — more pins just add
// inertia (collective anchor weight grows), they never hard-lock the pose.
//   - raise kDraggedPinWeight  -> grabbed pin tracks the cursor more tightly
//   - raise kAnchorPinWeight   -> previously-placed pins hold more stubbornly
//     (but push the ratio too far and you reintroduce the >=3-pin lock)
// At 3:1 the grabbed pin leads but doesn't violently swing the cube; combined
// with persistent placements (un-dragged user-set pins hold their stored pixel),
// the fit settles instead of sliding around, so alignment converges. Raise this
// for snappier cursor tracking, lower it toward 1:1 for an even gentler, more
// democratic fit. Tune to taste.
constexpr float kDraggedPinWeight = 3.0f;
constexpr float kAnchorPinWeight  = 1.0f;


// Project a LOCAL mesh vertex through a SceneTransformations to image-pixel
// coords (matches the wireframe / intrinsics convention: i = P * (MV * local),
// divide by w). Used to build the "stay" targets for un-dragged real pins.
inline Eigen::Vector2f project_local_to_image(const Eigen::Vector3f& local,
                                              const SceneTransformations& st)
{
    const RowMajorMatrix4f mv  = st.view_matrix * st.model_matrix;
    const Eigen::Vector3f  cam = mv.block<3, 3>(0, 0) * local + mv.block<3, 1>(0, 3);
    const RowMajorMatrix3f P   = st.intrinsics.To3x3ProjectionMatrix();
    const Eigen::Vector3f  i   = P * cam;
    const float inv = (std::fabs(i.z()) > 1e-9f) ? 1.0f / i.z() : 0.0f;
    return Eigen::Vector2f(i.x() * inv, i.y() * inv);
}

// Pick >= 3 well-spread, non-collinear mesh vertices for the synthetic prior.
// Approximate-diameter walk (O(n) per pass): farthest-from-v0 → A, farthest-from-A
// → B, farthest-from-line-AB → C. Gives a large-area, non-degenerate triangle so
// the prior constrains rotation, not just translation. Returns vertex indices.
std::vector<unsigned> pick_spread_anchor_verts(const RowMajorArrayX3f& V)
{
    std::vector<unsigned> out;
    const Eigen::Index n = V.rows();
    if (n < 3) return out;   // caller guarantees >= 3 verts before solving

    auto pos = [&](Eigen::Index i) {
        return Eigen::Vector3f(V(i, 0), V(i, 1), V(i, 2));
    };

    // farthest from vertex 0
    Eigen::Index a = 0;
    { float bd = -1.0f; const Eigen::Vector3f p0 = pos(0);
      for (Eigen::Index i = 0; i < n; ++i) {
          const float d = (pos(i) - p0).squaredNorm();
          if (d > bd) { bd = d; a = i; } } }

    // farthest from A
    Eigen::Index b = a;
    { float bd = -1.0f; const Eigen::Vector3f pa = pos(a);
      for (Eigen::Index i = 0; i < n; ++i) {
          const float d = (pos(i) - pa).squaredNorm();
          if (d > bd) { bd = d; b = i; } } }

    // farthest from line AB
    Eigen::Index c = a;
    { float bd = -1.0f; const Eigen::Vector3f pa = pos(a), pb = pos(b);
      const Eigen::Vector3f ab = pb - pa; const float abl2 = ab.squaredNorm();
      for (Eigen::Index i = 0; i < n; ++i) {
          const Eigen::Vector3f ap = pos(i) - pa;
          const float t = (abl2 > 1e-12f) ? ap.dot(ab) / abl2 : 0.0f;
          const float d = (ap - t * ab).squaredNorm();
          if (d > bd) { bd = d; c = i; } } }

    out = { (unsigned)a, (unsigned)b, (unsigned)c };
    return out;
}

// -----------------------------------------------------------------------------
// The ONE rigid pin solve. Anchored weighted PnP:
//   - real pins  : caller-supplied image targets + weights. The dragged pin is
//                  the mover (kDraggedPinWeight); every other real pin resists
//                  at kAnchorPinWeight, held at its drag-start projection. For
//                  the no-mover re-solve (delete) all real pins are equal.
//   - synthetic  : >= 3 well-spread mesh verts projected through `current` (the
//                  per-tick warm start) at a low weight. They satisfy
//                  SolvePnPIterative's CHECK_GE(rows,3), hold under-constrained
//                  DOFs, and break the collinear-pin roll degeneracy — which is
//                  why the old 35° lurch guard and the similarity fallback are
//                  retired. Their residual is ~0 at the warm start (they say
//                  "stay where we are this tick"), so they add no pose bias.
//
// object points (real and synthetic) live in `initial` camera space; the PnP
// solves a delta pose relative to `initial`, warm-started from `current`.
// -----------------------------------------------------------------------------
SceneTransformations solve_pins_weighted(
    const RowMajorMatrixX3f&    object_points,   // (N,3) real pin LOCAL verts
    const RowMajorMatrixX2f&    real_targets,    // (N,2) real pin image targets
    const Eigen::ArrayXf&       real_weights,    // (N)   real pin weights
    const RowMajorMatrixX3f&    synth_points,    // (S,3) synthetic LOCAL verts (S>=3)
    float                       synth_weight,
    const SceneTransformations& initial,
    const SceneTransformations& current)
{
    const RowMajorMatrix4f model_view = initial.view_matrix * initial.model_matrix;
    const RowMajorMatrix3f mvR = model_view.block<3, 3>(0, 0);
    const Eigen::Vector3f  mvt = model_view.block<3, 1>(0, 3);

    const Eigen::Index N = object_points.rows();
    const Eigen::Index S = synth_points.rows();
    const Eigen::Index R = N + S;

    // Object points in INITIAL camera space (what the PnP solves a delta on).
    RowMajorMatrixX3f obj_cam(R, 3);
    if (N > 0)
        obj_cam.topRows(N)    = (object_points * mvR.transpose()).rowwise() + mvt.transpose();
    if (S > 0)
        obj_cam.bottomRows(S) = (synth_points  * mvR.transpose()).rowwise() + mvt.transpose();

    // Image targets: real pins from the caller; synthetic projected through
    // CURRENT so their residual is ~0 at the warm start.
    RowMajorMatrixX2f image_points(R, 2);
    if (N > 0) image_points.topRows(N) = real_targets;
    if (S > 0) {
        const RowMajorMatrix4f cur_mv  = current.view_matrix * current.model_matrix;
        const RowMajorMatrix3f cur_mvR = cur_mv.block<3, 3>(0, 0);
        const Eigen::Vector3f  cur_mvt = cur_mv.block<3, 1>(0, 3);
        const RowMajorMatrix3f Pc_T =
            current.intrinsics.To3x3ProjectionMatrix().transpose();
        const RowMajorMatrixX3f scam =
            (synth_points * cur_mvR.transpose()).rowwise() + cur_mvt.transpose();
        const RowMajorMatrixX3f si3 = scam * Pc_T;       // (x, y, w) per row
        for (Eigen::Index r = 0; r < S; ++r) {
            const float w   = si3(r, 2);
            const float inv = (std::fabs(w) > 1e-9f) ? 1.0f / w : 0.0f;
            image_points(N + r, 0) = si3(r, 0) * inv;
            image_points(N + r, 1) = si3(r, 1) * inv;
        }
    }

    // Weights: real from the caller, synthetic at the low prior weight.
    Eigen::ArrayXf weights(R);
    if (N > 0) weights.head(N) = real_weights;
    if (S > 0) weights.tail(S).setConstant(synth_weight);

    // Warm-start the delta pose from `current` (identity when current==initial).
    const RowMajorMatrix4f initial_pose =
        (current.view_matrix * current.model_matrix) * model_view.inverse();

    PnPResult result;
    result.camera.intrinsics = current.intrinsics;
    result.camera.pose       = Pose::FromRt(initial_pose);

    PnPOptions opts;
    opts.bundle_opts.loss_type    = BundleOptions::LossType::TRIVIAL;  // every pin trusted
    opts.max_inlier_error         = 0.0f;   // interactive: no inlier rejection
    opts.optimize_focal_length    = false;
    opts.optimize_principal_point = false;

    SolvePnPIterative(obj_cam, image_points, weights, opts, result);

    const RowMajorMatrix3f result_R = result.camera.pose.R();
    const Eigen::Vector3f  result_t = result.camera.pose.t;

    RowMajorMatrix4f new_model_view = RowMajorMatrix4f::Identity();
    new_model_view.block<3, 3>(0, 0) = result_R * mvR;
    new_model_view.block<3, 1>(0, 3) = result_R * mvt + result_t;

    SceneTransformations out;
    out.model_matrix = initial.view_matrix.inverse() * new_model_view;
    out.view_matrix  = current.view_matrix;
    out.intrinsics   = result.camera.intrinsics;
    return out;
}

}  // namespace

void PolychaseTracker::run_pin_solve(int pin_idx)
{
    using namespace DD::Image;

    if (pin_idx < 0 || (size_t)pin_idx >= pins_.size()) return;
    const Pin& pin = pins_[pin_idx];

    CameraOp* cam = input_cam();
    Op*       geo = input_geo_op();
    Iop*      img = input_img();
    if (!cam || !geo) {
        PCN_LOG("[solve] missing inputs\n");
        return;
    }
    cam->validate(true);
    geo->validate(true);
    if (img) img->validate(true);

    // Format dimensions — must match what we use elsewhere for pixel coords
    float fmt_w = 2048.0f, fmt_h = 1080.0f;
    if (img) {
        const Format& fmt = img->info().format();
        fmt_w = (float)fmt.width();
        fmt_h = (float)fmt.height();
    }

    // Mesh in local coords. The Embree-backed AcceleratedMesh stores vertices
    // as RowMajorArrayX3f, polychase's FindTransformation wants a
    // RowMajorMatrixX3f — convert by .matrix() (a no-op view typically).
    GeoMesh gm;
    if (!extract_mesh(geo, gm)) {
        PCN_LOG("[solve] no geometry\n");
        return;
    }
    const Eigen::Index nverts = gm.local_vertices.rows();
    if (nverts < 3) {
        PCN_LOG("[solve] need >=3 vertices, got " << nverts << "\n");
        return;
    }

    // -------- Constraint set: this frame's pins only (per-frame) --------
    // Only pins whose created_frame matches the current frame participate. A pin
    // from another frame carries a 2D target valid at THAT frame, so including it
    // here would drag this frame's pose toward a stale position. We gather the
    // current-frame pins into `rows` (indices into pins_) and solve over that
    // subset. The dragged pin was placed/grabbed on this frame, so it's in the
    // set; we find its position within the subset as `dragged_row`.
    //
    // All counts route through the single weighted rigid solve below (no count
    // dispatch). The dragged pin moves to its target at kDraggedPinWeight; the
    // other current-frame pins resist at kAnchorPinWeight (stored pixel if the
    // user has positioned them, else their frozen `initial` projection). A
    // low-weight synthetic prior (>=3 spread mesh verts) keeps 1-2 pin solves
    // well-posed (SolvePnPIterative needs >=3 rows) and breaks roll degeneracy.
    const int cur_frame = editing_frame();

    std::vector<size_t> rows;
    rows.reserve(pins_.size());
    int dragged_row = -1;
    for (size_t i = 0; i < pins_.size(); ++i) {
        // A pin participates on this frame iff it has a 2D here — a manual KEY or a
        // LINKED track seen on this frame (resolve_pin_2d with interpolation off is
        // exactly that test). The dragged pin always qualifies (on_mouse_drag keys
        // it here first). Off-frame keys/tracks are inert, so the solve stays as
        // lightly constrained as the old per-frame solve.
        if ((Eigen::Index)pins_[i].vertex_idx >= nverts) continue;
        if (!resolve_pin_2d(pins_[i], cur_frame, fmt_h, /*allow_interp=*/false)) continue;
        if ((int)i == pin_idx) dragged_row = (int)rows.size();
        rows.push_back(i);
    }
    const size_t npins = rows.size();
    if (npins < 1 || dragged_row < 0) {
        PCN_LOG("[solve] DIAG skip: dragged pin not found (npins=" << npins
                << " dragged_row=" << dragged_row << ")\n");
        return;
    }

    RowMajorMatrixX3f object_points((Eigen::Index)npins, 3);
    for (size_t r = 0; r < npins; ++r) {
        const unsigned vidx = pins_[rows[r]].vertex_idx;
        object_points((Eigen::Index)r, 0) = gm.local_vertices((Eigen::Index)vidx, 0);
        object_points((Eigen::Index)r, 1) = gm.local_vertices((Eigen::Index)vidx, 1);
        object_points((Eigen::Index)r, 2) = gm.local_vertices((Eigen::Index)vidx, 2);
    }

    // -------- initial (anchor) vs current (warm-start) --------
    // initial defines BOTH the model_view the pins are projected through AND
    // the "stay" targets for the un-dragged pins. It must stay FROZEN for the
    // whole drag — if it were re-read from live_scene_ every tick it would
    // chase `current`, the stay constraints would collapse to "stay where you
    // are now", and the solve would drift. We capture it once on the first
    // tick of a drag (drag_anchor_) and reuse it until release. `current` is
    // only the LM warm-start, so it tracks the latest solved pose.
    //
    // view_matrix and intrinsics ALWAYS come from the live camera at this
    // frame — never a stored snapshot. They're frame-dependent, and this is
    // also what lets a keyed pose (which carries only model_matrix) solve
    // correctly: we just refresh the camera parts here.
    const RowMajorMatrix4f cam_view = nuke_to_eigen_m4(cam->imatrix());
    const CameraIntrinsics cam_intr =
        build_intrinsics_from_projection(cam->projection(), fmt_w, fmt_h);

    // Starting model for this drag: the live in-progress pose if we have one,
    // else the keyframed pose at the CURRENT frame (so per-frame adjustment
    // starts from that frame's keyed pose), else the upstream geo transform.
    const RowMajorMatrix4f start_model =
        live_scene_   ? live_scene_->model_matrix
      : has_pose_keys() ? nuke_to_eigen_m4(pose_matrix_to_nuke((double)editing_frame()))
                        : nuke_to_eigen_m4(gm.object_to_world);

    SceneTransformations initial;
    if (drag_anchor_) {
        initial = *drag_anchor_;
    } else {
        initial.model_matrix = start_model;
        initial.view_matrix  = cam_view;
        initial.intrinsics   = cam_intr;
        drag_anchor_ = initial;   // freeze for the rest of this drag
    }

    SceneTransformations current = initial;
    current.model_matrix = start_model;   // warm-start from the current pose
    current.view_matrix  = cam_view;
    current.intrinsics   = cam_intr;

    // -------- Real-pin targets + weights --------
    // Targets, per pin:
    //   - the dragged pin  : the cursor pixel (the mover), at kDraggedPinWeight.
    //   - an un-dragged pin the user has ALREADY positioned (is_target_user_set):
    //     its STORED target pixel. This is the key to convergence — a placement
    //     you made earlier keeps insisting on its spot through every later drag,
    //     instead of being forgotten and re-anchored to "wherever it projects
    //     now". Without this, each drag erases your previous placements and the
    //     cube can never be aligned (whack-a-mole).
    //   - an un-dragged pin you've never positioned: its projection through the
    //     FROZEN `initial` pose, i.e. "hold where you were when this drag began"
    //     (it has no meaningful 2D target yet, so it just resists drift).
    // All un-dragged pins hold at kAnchorPinWeight; the dragged one leads at the
    // higher kDraggedPinWeight. Finite ratio (not the old 1000:1 hard lock), so
    // 3+ pins never freeze the pose. NOTE: with a single rigid pose, dragging
    // one pin still moves the others on screen (rigid coupling — unavoidable
    // without deforming the mesh); persistent targets make them settle BACK to
    // their placed spots as you go, so the alignment actually converges.
    RowMajorMatrixX2f real_targets((Eigen::Index)npins, 2);
    Eigen::ArrayXf    real_weights((Eigen::Index)npins);
    // FREE PIN MOVE: ONLY the dragged pin constrains the solve. Every other pin —
    // even one with a manual key on this frame — gets ZERO weight, so it does not
    // pull the cube back. The diagnostic proved the "fight" comes from pins KEYED on
    // this frame (you aligned them here earlier), not from the track link, so we
    // can't fix it by gathering fewer pins; we have to drop their pull to zero. The
    // dragged pin leads and the low-weight synthetic prior (below) holds the rest of
    // the DOFs so the cube follows that one handle smoothly. NORMAL mode keeps every
    // pin at kAnchorPinWeight so the body fits them all together.
    const float other_weight = kAnchorPinWeight;
    real_weights.setConstant(other_weight);
    for (size_t r = 0; r < npins; ++r) {
        if ((int)r == dragged_row) continue;          // mover handled below
        const Pin& pr = pins_[rows[r]];
        // Every participating pin has a 2D here (the gather guarantees it): a manual
        // key, or its linked track at this frame. That resolved pixel IS its target.
        // (In free mode its weight is zero, so the target only matters for the row
        // shape — it won't pull.)
        const Eigen::Vector2f tgt = *resolve_pin_2d(pr, cur_frame, fmt_h,
                                                    /*allow_interp=*/false);
        real_targets((Eigen::Index)r, 0) = tgt.x();
        real_targets((Eigen::Index)r, 1) = tgt.y();
    }
    real_targets((Eigen::Index)dragged_row, 0) = pin.target_x_px;   // mover → cursor
    real_targets((Eigen::Index)dragged_row, 1) = pin.target_y_px;
    real_weights((Eigen::Index)dragged_row)    = kDraggedPinWeight;

    // -------- Synthetic prior: >= 3 well-spread verts through `current` --------
    // Satisfies CHECK_GE(rows,3), holds DOFs 1-2 real pins miss, breaks the
    // collinear-pin roll degeneracy. Cached; recomputed only when the geo's
    // vertex count changes.
    ensure_synth_anchors(gm);
    RowMajorMatrixX3f synth_points((Eigen::Index)synth_anchor_verts_.size(), 3);
    for (size_t i = 0; i < synth_anchor_verts_.size(); ++i) {
        const unsigned v = synth_anchor_verts_[i];
        synth_points((Eigen::Index)i, 0) = gm.local_vertices((Eigen::Index)v, 0);
        synth_points((Eigen::Index)i, 1) = gm.local_vertices((Eigen::Index)v, 1);
        synth_points((Eigen::Index)i, 2) = gm.local_vertices((Eigen::Index)v, 2);
    }

    // -------- The ONE rigid solve. All pin counts route here (no count
    // dispatch, no lurch guard, no similarity fallback). 1 pin nudges mostly
    // translation; 2 add a little rotation; 3+ well-spread = crisp full PnP. --
    SceneTransformations result;
    try {
        result = solve_pins_weighted(object_points, real_targets, real_weights,
                                     synth_points, kSynthAnchorWeight,
                                     initial, current);
    } catch (const std::exception& e) {
        PCN_LOG("[solve] rigid PnP threw: " << e.what() << "\n");
        return;
    } catch (...) {
        PCN_LOG("[solve] rigid PnP threw unknown exception\n");
        return;
    }

    // Reject a non-finite (NaN/Inf) solve. A pathological drag — cursor flung
    // far, near-collinear pins, a bad warm-start — can make the LM/PnP diverge
    // and return garbage. If we accepted it, effective_model_matrix() would feed
    // that pose to every projection: the wireframe culls to nothing, vertex-snap
    // reports "no visible vertices", hit-test grabs nothing — pin mode looks
    // frozen, and (because release keys it onto the curve) a Nuke restart seemed
    // like the only way out. Keeping the previous good live_scene_ instead means
    // a bad tick is simply ignored; the next coherent drag position re-solves.
    if (!result.model_matrix.allFinite()) {
        PCN_LOG("[solve] rejected non-finite pose (kept previous) — "
                     "drag was likely degenerate this tick\n");
        return;
    }

    live_scene_ = result;
    live_edit_frame_ = editing_frame();
}


// =============================================================================
// on_solve_focal_sweep — from-scratch, per-frame GLOBAL focal search.
//
// The alternating Solve Zoom is trapped in the constant-focal local minimum:
// fitting focal to a pose that was tracked under the wrong focal merely confirms
// it (the object's zoom-in gets explained as moving closer). This solver avoids
// that by never trusting a pre-tracked pose. At each frame it sweeps the focal
// across the plausible range, RE-SOLVES the pose fresh (PnP from the pins/tracks)
// at each candidate, measures reprojection error, and keeps the focal whose error
// is lowest. With real parallax the rigid object's foreshortening makes the
// error-vs-focal curve dip sharply at the TRUE focal, so a varying lens is
// recovered with no anchors and no hand alignment. Writes only solved_focal.
//
// Correspondences and conventions mirror run_pin_solve exactly (Nuke Y-up pin
// targets via resolve_pin_2d, GL intrinsics via build_intrinsics_from_projection,
// the same solve_pins_weighted PnP), so no new sign conventions are introduced.
// =============================================================================
void PolychaseTracker::on_solve_focal_sweep()
{
    using namespace DD::Image;
    std::ostringstream oss;
    oss << "[" << timestamp() << "] Solve Focal (Sweep) — per-frame global focal search\n";

    CameraOp* cam = input_cam();
    Op*       geo = input_geo_op();
    Iop*      img = input_img();
    if (!cam || !geo) { oss << "  [FAIL] camera/geo input not connected."; set_status(oss.str()); return; }
    if (last_frame_ <= first_frame_) { oss << "  [FAIL] Last Frame must be greater than First Frame."; set_status(oss.str()); return; }

    cam->validate(true);
    geo->validate(true);
    if (img) img->validate(true);

    float fmt_w = 2048.0f, fmt_h = 1080.0f;
    if (img) { const Format& fmt = img->info().format(); fmt_w = (float)fmt.width(); fmt_h = (float)fmt.height(); }

    double haperture = 24.576;
    if (Knob* hk = cam->knob("haperture")) haperture = hk->get_value();

    GeoMesh gm;
    if (!extract_mesh(geo, gm)) { oss << "  [FAIL] could not extract a mesh."; set_status(oss.str()); return; }
    const Eigen::Index nverts = gm.local_vertices.rows();
    if (nverts < 3) { oss << "  [FAIL] mesh has < 3 vertices."; set_status(oss.str()); return; }

    ensure_pins_loaded();

    // Synthetic prior verts (>=3, well spread) — keep each per-frame PnP well-posed
    // exactly as run_pin_solve does (SolvePnPIterative needs >=3 rows / breaks roll).
    ensure_synth_anchors(gm);
    const Eigen::Index S = (Eigen::Index)synth_anchor_verts_.size();
    RowMajorMatrixX3f synth_points(S, 3);
    for (Eigen::Index s = 0; s < S; ++s) {
        const unsigned v = synth_anchor_verts_[(size_t)s];
        synth_points(s, 0) = gm.local_vertices((Eigen::Index)v, 0);
        synth_points(s, 1) = gm.local_vertices((Eigen::Index)v, 1);
        synth_points(s, 2) = gm.local_vertices((Eigen::Index)v, 2);
    }

    // Base projection — supplies principal point / aspect; the focal diagonal is
    // overridden per candidate. cam->projection() is fine here (we never read its
    // focal; apply_curve_focal rewrites a00/a11 from the candidate, sign-preserving).
    const DD::Image::Matrix4 base_proj = cam->projection();

    // Focal sweep range, derived from the Min/Max FOV bounds (same controls the
    // other solvers clamp to). fov = 2*atan(haperture / (2 f)) -> f = haperture / (2 tan(fov/2)).
    constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
    const double f_min_mm = haperture / (2.0 * std::tan(0.5 * max_fov_deg_ * kDeg2Rad)); // widest FOV -> shortest focal
    const double f_max_mm = haperture / (2.0 * std::tan(0.5 * min_fov_deg_ * kDeg2Rad)); // narrowest FOV -> longest focal
    if (!(f_max_mm > f_min_mm) || !std::isfinite(f_min_mm) || !std::isfinite(f_max_mm)) {
        oss << "  [FAIL] bad focal range from FOV bounds (min_fov=" << min_fov_deg_
            << " max_fov=" << max_fov_deg_ << ").";
        set_status(oss.str()); return;
    }
    constexpr int    kCoarse = 48;   // coarse sweep steps across the range
    constexpr int    kRefine = 12;   // refine steps in the bracket around the coarse min

    // Reproject obj_pts through (view, model, intr) and return RMS pixel error vs targets.
    auto reproj_rms = [&](const RowMajorMatrixX3f& obj_pts,
                          const RowMajorMatrixX2f& targets,
                          const RowMajorMatrix4f&  view,
                          const RowMajorMatrix4f&  model,
                          const CameraIntrinsics&  intr) -> double {
        const RowMajorMatrix4f mv = view * model;
        const RowMajorMatrix3f mvR = mv.block<3,3>(0,0);
        const Eigen::Vector3f  mvt = mv.block<3,1>(0,3);
        const RowMajorMatrix3f Pc_T = intr.To3x3ProjectionMatrix().transpose();
        const Eigen::Index N = obj_pts.rows();
        double sse = 0.0; int cnt = 0;
        for (Eigen::Index r = 0; r < N; ++r) {
            const Eigen::Vector3f X = obj_pts.row(r).transpose();
            const Eigen::Vector3f Xc = mvR * X + mvt;
            const Eigen::Vector3f pix = (Xc.transpose() * Pc_T).transpose();   // (x,y,w)
            if (std::fabs(pix.z()) < 1e-9f) continue;
            const double px = (double)pix.x() / (double)pix.z();
            const double py = (double)pix.y() / (double)pix.z();
            const double du = px - (double)targets(r, 0);
            const double dv = py - (double)targets(r, 1);
            sse += du*du + dv*dv; ++cnt;
        }
        return (cnt > 0) ? std::sqrt(sse / (double)cnt) : 1e30;
    };

    // One PnP at a fixed focal -> solved pose + its reprojection RMS.
    auto solve_at_focal = [&](double f_mm,
                              const RowMajorMatrixX3f& obj_pts,
                              const RowMajorMatrixX2f& targets,
                              const Eigen::ArrayXf&    weights,
                              const RowMajorMatrix4f&  cam_view,
                              const RowMajorMatrix4f&  start_model,
                              RowMajorMatrix4f&        out_model) -> double {
        DD::Image::Matrix4 proj_f = base_proj;
        conv::apply_curve_focal(proj_f, f_mm, haperture);
        const CameraIntrinsics intr_f = build_intrinsics_from_projection(proj_f, fmt_w, fmt_h);

        SceneTransformations initial;
        initial.model_matrix = start_model;
        initial.view_matrix  = cam_view;
        initial.intrinsics   = intr_f;
        const SceneTransformations out =
            solve_pins_weighted(obj_pts, targets, weights, synth_points, 0.01f, initial, initial);
        if (!out.model_matrix.allFinite()) { out_model = start_model; return 1e30; }
        out_model = out.model_matrix;
        return reproj_rms(obj_pts, targets, cam_view, out.model_matrix, intr_f);
    };

    int    frames_solved = 0, frames_skipped = 0;
    double sum_best_rms = 0.0;
    bool             have_prev = false;
    RowMajorMatrix4f prev_model = RowMajorMatrix4f::Identity();

    // Correspondence SOURCE. User tracks carry per-frame 2D for the WHOLE shot (the
    // clean, full-length correspondences the focal solve needs); pins only carry 2D
    // where you hand-keyed them. So prefer user tracks whenever there are enough —
    // this is what makes the sweep solve every frame instead of just the keyed ones.
    // NOTE: the user-track store is OpenCV Y-DOWN; solve_pins_weighted wants Nuke
    // Y-UP (same as resolve_pin_2d), so track pixels are flipped to fmt_h - y below.
    UserTracks  solve_tracks = build_solve_tracks();
    const bool  use_tracks   = (solve_tracks.size() >= 4);
    PCN_LOG("[fsweep] source = " << (use_tracks ? "USER TRACKS" : "PINS")
            << " (" << solve_tracks.size() << " track(s), " << pins_.size() << " pin(s))\n");

    // CAMERA VIEW — sampled ONCE at the seed frame, held CONSTANT for every frame.
    // This is the tracker's convention (see view_at in tracker_intrinsics.cpp): the
    // relative camera<->object motion is carried by the per-frame OBJECT POSE that
    // PnP solves below, NOT by re-sampling the camera each frame. Re-sampling the
    // camera per frame returns a STALE basis (frozen at the panel frame, same as the
    // projection()'s focal staleness we already fixed), which froze the view at
    // frame 1 and made the focal collapse toward the short end to compensate. With a
    // fixed seed view, the object pose absorbs the parallax and the focal stays put.
    const int seed_frame = first_frame_;
    const RowMajorMatrix4f cam_view =
        nuke_to_eigen_m4(icp_pin_cam_imatrix_at(cam, (double)seed_frame));

    for (int t = first_frame_; t <= last_frame_; ++t) {
        // Build (object LOCAL 3D, target Nuke-Yup 2D) correspondences at frame t,
        // from user tracks (every frame) or, as a fallback, from pins (keyed frames).
        RowMajorMatrixX3f obj_pts;
        RowMajorMatrixX2f targets;

        if (use_tracks) {
            std::vector<Eigen::Vector3f> O;
            std::vector<Eigen::Vector2f> T;
            O.reserve(solve_tracks.size());
            T.reserve(solve_tracks.size());
            for (const UserTrack& ut : solve_tracks) {
                const Eigen::Vector2f* o = nullptr;
                for (const UserTrackObservation& ob : ut.observations)
                    if (ob.frame_id == t) { o = &ob.image_point; break; }
                if (!o) continue;
                O.push_back(ut.object_point);                       // LOCAL vertex
                T.emplace_back(o->x(), fmt_h - o->y());             // y-down store -> Nuke Y-up
            }
            if (O.size() < 4) {
                ++frames_skipped;
                PCN_LOG("[fsweep] f=" << t << " SKIP (only " << O.size()
                        << " track obs; need >=4)\n");
                continue;
            }
            const Eigen::Index Nt = (Eigen::Index)O.size();
            obj_pts.resize(Nt, 3);
            targets.resize(Nt, 2);
            for (Eigen::Index r = 0; r < Nt; ++r) {
                obj_pts(r, 0) = O[(size_t)r].x(); obj_pts(r, 1) = O[(size_t)r].y(); obj_pts(r, 2) = O[(size_t)r].z();
                targets(r, 0) = T[(size_t)r].x(); targets(r, 1) = T[(size_t)r].y();
            }
        } else {
            std::vector<size_t> rows;
            rows.reserve(pins_.size());
            for (size_t i = 0; i < pins_.size(); ++i) {
                if ((Eigen::Index)pins_[i].vertex_idx >= nverts) continue;
                if (!resolve_pin_2d(pins_[i], t, fmt_h, /*allow_interp=*/false)) continue;
                rows.push_back(i);
            }
            if (rows.size() < 4) {
                ++frames_skipped;
                PCN_LOG("[fsweep] f=" << t << " SKIP (only " << rows.size() << " pin(s) with 2D; need >=4)\n");
                continue;
            }
            const Eigen::Index Np = (Eigen::Index)rows.size();
            obj_pts.resize(Np, 3);
            targets.resize(Np, 2);
            for (Eigen::Index r = 0; r < Np; ++r) {
                const Pin& p = pins_[rows[(size_t)r]];
                obj_pts(r, 0) = gm.local_vertices((Eigen::Index)p.vertex_idx, 0);
                obj_pts(r, 1) = gm.local_vertices((Eigen::Index)p.vertex_idx, 1);
                obj_pts(r, 2) = gm.local_vertices((Eigen::Index)p.vertex_idx, 2);
                const Eigen::Vector2f tgt = *resolve_pin_2d(p, t, fmt_h, /*allow_interp=*/false);
                targets(r, 0) = tgt.x();
                targets(r, 1) = tgt.y();
            }
        }

        const Eigen::Index Np = obj_pts.rows();
        Eigen::ArrayXf weights(Np);
        weights.setConstant(1.0f);

        const RowMajorMatrix4f seed_model =
            have_prev          ? prev_model
          : has_pose_keys()    ? nuke_to_eigen_m4(pose_matrix_to_nuke((double)t))
                               : nuke_to_eigen_m4(gm.object_to_world);

        // ---- coarse global sweep ----
        double best_rms = 1e30, best_f = f_min_mm;
        RowMajorMatrix4f best_model = seed_model;
        double rms_lo = 1e30, rms_hi = 1e30;   // neighbours of the best (dip sharpness)
        for (int k = 0; k <= kCoarse; ++k) {
            const double f_mm = f_min_mm + (f_max_mm - f_min_mm) * (double)k / (double)kCoarse;
            RowMajorMatrix4f m;
            const double rms = solve_at_focal(f_mm, obj_pts, targets, weights, cam_view, seed_model, m);
            if (rms < best_rms) { rms_lo = best_rms; best_rms = rms; best_f = f_mm; best_model = m; }
            else if (rms < rms_hi) { rms_hi = rms; }
        }

        // ---- local refine in the bracket [best +/- one coarse step] ----
        const double step = (f_max_mm - f_min_mm) / (double)kCoarse;
        const double lo = std::max(f_min_mm, best_f - step);
        const double hi = std::min(f_max_mm, best_f + step);
        for (int k = 0; k <= kRefine; ++k) {
            const double f_mm = lo + (hi - lo) * (double)k / (double)kRefine;
            RowMajorMatrix4f m;
            const double rms = solve_at_focal(f_mm, obj_pts, targets, weights, cam_view, seed_model, m);
            if (rms < best_rms) { best_rms = rms; best_f = f_mm; best_model = m; }
        }

        key_focal_at((double)t, best_f);
        prev_model = best_model; have_prev = true;
        sum_best_rms += best_rms; ++frames_solved;

        // Dip sharpness: how much worse the best's neighbours were. A large gap =
        // a confident frame (parallax pins the focal); ~0 = degenerate/ambiguous.
        const double neighbour = std::min(rms_lo, rms_hi);
        const double sharpness = (neighbour < 1e29 && best_rms > 1e-9)
                               ? (neighbour - best_rms) : 0.0;
        PCN_LOG("[fsweep] f=" << t << " best_focal=" << best_f << "mm  rms="
                << best_rms << "px  dip=" << sharpness << "px  pins=" << Np << "\n");
    }

    if (frames_solved == 0) {
        if (use_tracks)
            oss << "  [FAIL] User tracks were found but none had >=4 observations on any "
                   "frame. Re-anchor / extend the tracks across the shot.";
        else
            oss << "  [FAIL] No user tracks, and no frame had >=4 pins with a 2D. Create "
                   "User Tracks (or Connect pins to tracks), or hand-key >=4 corners.";
        set_status(oss.str());
        return;
    }

    // A run is only trustworthy when it covered most of the range from full-length
    // tracks. If it fell back to pins (sparse hand keys), say so LOUDLY instead of a
    // bare PASS — three keyed frames is not a focal curve.
    const int    span     = last_frame_ - first_frame_ + 1;
    const double coverage = (span > 0) ? (100.0 * frames_solved / span) : 0.0;
    const bool   sparse   = (!use_tracks) || (coverage < 50.0);

    if (sparse) {
        oss << "  [WARN] Solved only " << frames_solved << " of " << span << " frame(s) ("
            << (int)coverage << "%) from "
            << (use_tracks ? "USER TRACKS" : "PINS — no full-length tracks were available") << ".\n"
            << "  A real focal curve needs 2D on EVERY frame. To get that:\n"
            << "    1. Go to the Reference Frame (User Tracks tab) and 'Load Tracks' so the\n"
            << "       log prints '[utracks] anchored N/N' — that is the per-frame 2D source.\n"
            << "    2. (optional) 'Connect' there to pin the tracks to exact mesh vertices.\n"
            << "    3. Run Solve Focal (Sweep) again — the first log line should read\n"
            << "       '[fsweep] source = USER TRACKS' and ~every frame should solve.\n"
            << "  The " << frames_solved << " frame(s) above were written, but treat them as a smoke\n"
            << "  test, not a result.";
        set_status(oss.str());
        asapUpdate();
        return;
    }

    oss << "  [PASS] Focal sweep baked over " << first_frame_ << ".." << last_frame_
        << " from USER TRACKS.\n"
        << "    frames solved   = " << frames_solved << " (skipped " << frames_skipped
        << ")  coverage " << (int)coverage << "%\n"
        << "    mean best RMS   = " << (sum_best_rms / (double)frames_solved) << " px\n"
        << "  Only the Solved Focal curve was written (pose untouched). Check the\n"
        << "  curve in the Curve Editor; the per-frame [fsweep] log shows each\n"
        << "  frame's dip sharpness — a near-zero dip means that frame was too\n"
        << "  low-parallax to trust. 'Smooth Solved Focal' to clean it, then tick\n"
        << "  'Export Solved Focal' to bake it onto the camera.";
    set_status(oss.str());
    asapUpdate();
}


// -----------------------------------------------------------------------------
// resolve_pins_no_mover — re-fit the rigid pose from this frame's pins as equal-
// weight anchors (no distinguished mover), warm-started from the current pose,
// then key the result. Used by delete_pin_at: a removed pin is a removed
// constraint, not a reason to discard the gizmo+pin pose. Per-frame: if no pins
// remain ON THE CURRENT FRAME, it no-ops and leaves the keyed pose intact.
//
// Targets: user-set pins keep their pixel (so the pose can relax toward the
// remaining constraints once the deleted one is gone); never-dragged pins are
// refreshed to their current projection so they don't pull on a stale target.
// Caller must have already removed the deleted pin from pins_ and ensured at
// least one pin remains.
// -----------------------------------------------------------------------------
void PolychaseTracker::resolve_pins_no_mover()
{
    using namespace DD::Image;

    ensure_pins_loaded();
    if (pins_.empty()) return;

    CameraOp* cam = input_cam();
    Op*       geo = input_geo_op();
    Iop*      img = input_img();
    if (!cam || !geo) return;
    cam->validate(true);
    geo->validate(true);
    if (img) img->validate(true);

    float fmt_w = 2048.0f, fmt_h = 1080.0f;
    if (img) {
        const Format& fmt = img->info().format();
        fmt_w = (float)fmt.width();
        fmt_h = (float)fmt.height();
    }

    GeoMesh gm;
    if (!extract_mesh(geo, gm)) return;
    const Eigen::Index nverts = gm.local_vertices.rows();
    if (nverts < 3) return;

    // Only pins keyed on this frame take part (same gate as run_pin_solve). They
    // all carry equal weight here — no distinguished mover — so the body settles to
    // the least-squares best fit of every placed pin on the frame.
    const int cur_frame = editing_frame();
    std::vector<size_t> rows;
    rows.reserve(pins_.size());
    for (size_t i = 0; i < pins_.size(); ++i) {
        if ((Eigen::Index)pins_[i].vertex_idx >= nverts) continue;
        if (!resolve_pin_2d(pins_[i], cur_frame, fmt_h, /*allow_interp=*/false)) continue;
        rows.push_back(i);
    }
    const size_t npins = rows.size();
    if (npins == 0) return;   // no pins keyed here → leave the keyed pose

    // initial == current == the current pose (keyed at this frame, or the live
    // working pose, or upstream). No drag is in progress, so we build it fresh
    // and do NOT touch drag_anchor_.
    const RowMajorMatrix4f cam_view = nuke_to_eigen_m4(cam->imatrix());
    const CameraIntrinsics cam_intr =
        build_intrinsics_from_projection(cam->projection(), fmt_w, fmt_h);
    const RowMajorMatrix4f cur_model =
        live_scene_   ? live_scene_->model_matrix
      : has_pose_keys() ? nuke_to_eigen_m4(pose_matrix_to_nuke((double)editing_frame()))
                        : nuke_to_eigen_m4(gm.object_to_world);

    SceneTransformations cur;
    cur.model_matrix = cur_model;
    cur.view_matrix  = cam_view;
    cur.intrinsics   = cam_intr;

    RowMajorMatrixX3f object_points((Eigen::Index)npins, 3);
    RowMajorMatrixX2f real_targets((Eigen::Index)npins, 2);
    Eigen::ArrayXf    real_weights((Eigen::Index)npins);
    real_weights.setConstant(kAnchorPinWeight);        // equal anchors, no mover
    for (size_t r = 0; r < npins; ++r) {
        const Pin& pr = pins_[rows[r]];
        const unsigned vidx = pr.vertex_idx;
        const Eigen::Vector3f lv(gm.local_vertices((Eigen::Index)vidx, 0),
                                 gm.local_vertices((Eigen::Index)vidx, 1),
                                 gm.local_vertices((Eigen::Index)vidx, 2));
        object_points((Eigen::Index)r, 0) = lv.x();
        object_points((Eigen::Index)r, 1) = lv.y();
        object_points((Eigen::Index)r, 2) = lv.z();
        // Active here → its target is the resolved key/linked-track pixel (manual
        // key only when Free Pin Move ignores the link).
        const Eigen::Vector2f tgt = *resolve_pin_2d(pr, cur_frame, fmt_h,
                                                    /*allow_interp=*/false);
        real_targets((Eigen::Index)r, 0) = tgt.x();
        real_targets((Eigen::Index)r, 1) = tgt.y();
    }

    ensure_synth_anchors(gm);
    RowMajorMatrixX3f synth_points((Eigen::Index)synth_anchor_verts_.size(), 3);
    for (size_t i = 0; i < synth_anchor_verts_.size(); ++i) {
        const unsigned v = synth_anchor_verts_[i];
        synth_points((Eigen::Index)i, 0) = gm.local_vertices((Eigen::Index)v, 0);
        synth_points((Eigen::Index)i, 1) = gm.local_vertices((Eigen::Index)v, 1);
        synth_points((Eigen::Index)i, 2) = gm.local_vertices((Eigen::Index)v, 2);
    }

    SceneTransformations result;
    try {
        // No distinguished mover here: every real pin carries kAnchorPinWeight,
        // so this is the equal-weight no-mover fit.
        result = solve_pins_weighted(object_points, real_targets, real_weights,
                                     synth_points, kSynthAnchorWeight,
                                     cur, cur);
    } catch (const std::exception& e) {
        PCN_LOG("[solve] no-mover re-solve threw: " << e.what() << "\n");
        return;
    } catch (...) {
        PCN_LOG("[solve] no-mover re-solve threw unknown exception\n");
        return;
    }

    // Same non-finite guard as run_pin_solve: never bake a NaN/Inf pose into a
    // key. That would poison the curve permanently — every later frame reads it
    // back through effective_model_matrix and culls the overlay.
    if (!result.model_matrix.allFinite()) {
        PCN_LOG("[solve] no-mover re-solve produced a non-finite pose — "
                     "keeping the existing keyed pose\n");
        return;
    }

    // Stage the re-fit pose, then bake it to a key (mirrors the drag-release
    // commit). key_pose_at_current_frame() also clears live_scene_ + drag state.
    live_scene_      = result;
    live_edit_frame_ = editing_frame();
    key_pose_at_current_frame();
}


// -----------------------------------------------------------------------------
// ensure_synth_anchors — (re)fill the cached well-spread synthetic-anchor
// vertex set. Recomputes only when the geo's vertex count changes (a cheap
// proxy for "the geometry changed").
// -----------------------------------------------------------------------------
void PolychaseTracker::ensure_synth_anchors(const GeoMesh& gm)
{
    const Eigen::Index n = gm.local_vertices.rows();
    if (n == synth_geo_vert_count_ && !synth_anchor_verts_.empty()) return;
    synth_anchor_verts_   = pick_spread_anchor_verts(gm.local_vertices);
    synth_geo_vert_count_ = n;
}


// -----------------------------------------------------------------------------
// reset_live_solve — wipe the solver state and trigger a viewer redraw.
//
// Called when:
//   - "Clear Solve" button pressed
//   - pins_blob mutated externally (undo / .nk load) — different constraint
//     set, so the previous solve is no longer valid
//   - all pins cleared
//   - a pin is deleted
// -----------------------------------------------------------------------------
void PolychaseTracker::reset_live_solve()
{
    if (live_scene_) {
        live_scene_.reset();
        PCN_LOG("[solve] live state reset\n");
    }
    drag_anchor_.reset();
    // Also clear any in-progress rotation offset + its frozen base, so the
    // working pose is fully discarded (not just the translate/dolly part).
    reset_rot_offsets();
    reset_trans_offsets();   // and the translate/dolly offset + its frozen base
    // Keep the undoable gizmo-pose mirror in step with the now-empty live_scene_,
    // so a stale gizmo pose can't be re-restored by a later undo and the overlay
    // truly falls back to the keyed curve / upstream geo.
    sync_blob_from_live_pose();
    asapUpdate();
}


// -----------------------------------------------------------------------------
// Pin persistence + UI sync.
// -----------------------------------------------------------------------------

void PolychaseTracker::sync_blob_from_pins()
{
    // pins_ has just been modified by a click / delete / clear. Serialize and
    // push to the hidden pins_blob String_knob. Nuke's undo stack captures
    // this knob change automatically, so Ctrl+Z reverts to the prior state.
    //
    // The set_text() call will trigger knob_changed("pins_blob") synchronously,
    // which calls on_pins_blob_changed(). To avoid that recursive path re-
    // deserializing pins_ (which we just authoritatively set), we guard with
    // suppress_pin_blob_callback_.
    const std::string s = serialize_pin_list(pins_);
    {
        ScopedFlags guard(suppress_pin_blob_callback_);
        if (DD::Image::Knob* k = knob("pins_blob")) {
            k->set_text(s.c_str());
        }
        pins_blob_cache_ = s;   // keep the load-cache in sync with what we wrote
        pins_in_memory_authoritative_ = true;   // memory is now the source of truth
    }

    refresh_pin_ui();
    asapUpdate();
}


void PolychaseTracker::restore_pins_from_blob(const std::string& s)
{
    // Move-history undo/redo: set the pin set to a recorded snapshot. We own the
    // string (it came from our own history), so deserialize it straight into pins_
    // and push back through the normal sync path — that updates the blob, the
    // load-cache, the authoritative latch, the panel list, and triggers a redraw.
    pins_ = deserialize_pin_list(s.c_str());
    next_pin_id_ = compute_next_pin_id(pins_);
    sync_blob_from_pins();
}


void PolychaseTracker::on_pins_blob_changed()
{
    // Fires when the pins_blob String_knob value changes:
    //   - on .nk load (Nuke restores the saved value)
    //   - on Ctrl+Z / Ctrl+Y (Nuke walks the undo stack)
    //   - rarely, if a user edits the hidden knob via Python
    // Skip our own self-triggered updates (sync_blob_from_pins set the flag).
    // set_text/knob_changed are ASYNC in this Nuke build, so the time-boxed
    // suppress flag we used before was already false by the time this deferred
    // callback ran (DIAG showed "NOT our own write" firing for our OWN placement,
    // then a stale reload dropping the pin). Detect our own echo by CONTENT: if
    // the blob equals what we last serialized, pins_ is already authoritative —
    // ignore it. Only a genuinely different value (real undo / .nk load) reloads.
    const char* blob = pins_blob_ ? pins_blob_ : "";
    if (suppress_pin_blob_callback_ || pins_blob_cache_ == blob) return;

    pins_ = deserialize_pin_list(blob);
    next_pin_id_ = compute_next_pin_id(pins_);
    PCN_LOG("[pins] DIAG on_pins_blob_changed RELOAD: " << pins_.size()
              << " pins, next_id=" << next_pin_id_
              << " (undo/redo/.nk/external — NOT our own write)\n");
    pins_blob_cache_ = blob;
    pins_loaded_from_blob_ = true;
    pins_in_memory_authoritative_ = true;   // this blob is the new truth

    // NOTE: we deliberately do NOT reset the live solve here. This callback can
    // fire during .nk load when Nuke restores pins_blob, and resetting would
    // wipe the just-restored solved pose. Genuine pin edits (place/delete/clear)
    // reset the solve explicitly in their own handlers.

    refresh_pin_ui();
    asapUpdate();
}


void PolychaseTracker::ensure_pins_loaded()
{
    // Reload whenever the serialized blob differs from what we last parsed.
    // This is what makes .nk load reliable: ensure_pins_loaded() may run once
    // early (e.g. a first draw) BEFORE Nuke has restored the saved pins_blob
    // value, which would cache an empty list. Latching on a one-shot bool then
    // left pins empty forever. By comparing content, the next access after the
    // value is restored sees the difference and reloads.
    // In-session, pins_ is authoritative — do NOT re-read the knob. This is the
    // direct fix for the DIAG failure: a stale async blob echo was reloaded here,
    // dropping a just-placed pin. Genuine undo/.nk changes arrive via the
    // knob_changed("pins_blob") event (on_pins_blob_changed), not this poller.
    if (pins_in_memory_authoritative_) return;

    const char* blob = pins_blob_ ? pins_blob_ : "";
    if (pins_loaded_from_blob_ && pins_blob_cache_ == blob) return;
    pins_ = deserialize_pin_list(blob);
    next_pin_id_ = compute_next_pin_id(pins_);
    PCN_LOG("[pins] DIAG ensure_pins_loaded RELOAD (live blob != cache): "
              << pins_.size() << " pins, next_id=" << next_pin_id_ << "\n");
    // Bind restored linked_track_name -> current index (no-op until tracks built;
    // the track rebuild also calls this, so either load order converges).
    reresolve_pin_links();
    pins_blob_cache_ = blob;
    pins_loaded_from_blob_ = true;
    // Got a real (non-empty) list from the knob → trust memory from here on.
    if (!pins_.empty()) pins_in_memory_authoritative_ = true;
}


void PolychaseTracker::refresh_pin_ui()
{
    // -- Multiline display --
    // Shows 1-indexed pin numbers so the user can plug them straight into
    // the "Pin index" Int_knob for selective delete.
    std::ostringstream oss;
    if (pins_.empty()) {
        oss << "No pins. Click on a mesh vertex in the 2D viewer to place one.";
    } else {
        oss << pins_.size() << " pin" << (pins_.size() == 1 ? "" : "s") << ":\n";
        for (size_t i = 0; i < pins_.size(); ++i) {
            const Pin& p = pins_[i];
            oss << "  " << (i + 1) << ". pin#" << p.id
                << "  vertex " << p.vertex_idx;
            // link status: show the STABLE name. If a name is stored but no longer
            // resolves to a current track (index -1), flag it so a stale link is
            // visible rather than silent.
            if (!p.linked_track_name.empty()) {
                oss << "  link '" << p.linked_track_name << "'";
                if (p.linked_track < 0) oss << " (missing)";
            } else if (p.linked_track >= 0) {
                oss << "  link t" << (p.linked_track + 1);
            }
            // key count + frames
            if (p.keys.empty()) {
                oss << "  (no keys)";
            } else {
                oss << "  " << p.keys.size() << " key"
                    << (p.keys.size() == 1 ? "" : "s") << " @ ";
                bool first = true;
                for (const auto& kv : p.keys) {
                    if (!first) oss << ',';
                    oss << kv.first;
                    first = false;
                }
            }
            if (i + 1 < pins_.size()) oss << '\n';
        }
    }
    const std::string display = oss.str();
    if (DD::Image::Knob* k = knob("pin_list_display")) {
        k->set_text(display.c_str());
    }
}


void PolychaseTracker::clear_pins()
{
    ensure_pins_loaded();
    if (pins_.empty()) return;
    pins_.clear();
    next_pin_id_ = 0;
    dragging_pin_idx_ = -1;
    drag_anchor_.reset();
    // Deliberately do NOT reset_live_solve(): pins are an editor for the rigid
    // pose, not the pose itself. Removing the pins should leave whatever pose
    // was last keyed exactly where it is — the user can still gizmo it, re-pin
    // it, or clear the pose keys explicitly. Wiping the solve here would yank
    // the object back to the upstream default and silently discard work.
    PCN_LOG("[PolychaseTracker] all pins cleared (keyed pose left intact)\n");
    sync_blob_from_pins();
}


void PolychaseTracker::delete_pin_at(unsigned slot)
{
    ensure_pins_loaded();
    if (slot >= pins_.size()) return;
    const unsigned removed_id = pins_[slot].id;
    pins_.erase(pins_.begin() + (long)slot);
    dragging_pin_idx_ = -1;
    drag_anchor_.reset();
    PCN_LOG("[PolychaseTracker] deleted pin #" << removed_id
              << " (slot " << (slot + 1) << ")\n");

    // A removed pin is a removed constraint. If pins remain, re-fit the rigid
    // pose from them as equal anchors (no mover) and re-key — the pose relaxes
    // toward the surviving constraints instead of staying frozen on the old
    // solve. If NO pins remain, leave the keyed pose intact (same rationale as
    // clear_pins): deletion edits constraints, it doesn't reset the object.
    if (!pins_.empty()) {
        resolve_pins_no_mover();
    }
    sync_blob_from_pins();
}


// Remove a single 2D key from a pin at `frame` (the per-frame hand correction).
// Returns true if a key was actually removed. The pin itself stays; without a key
// at this frame it falls back to its linked track / interpolation / vertex.
bool PolychaseTracker::clear_pin_key_at(unsigned slot, int frame)
{
    ensure_pins_loaded();
    if (slot >= pins_.size()) return false;
    Pin& p = pins_[slot];
    const auto it = p.keys.find(frame);
    if (it == p.keys.end()) return false;
    p.keys.erase(it);
    // Keep the transitional legacy flag honest: if nothing is keyed any more, the
    // pin is back to a plain anchor for the old solve.
    if (p.keys.empty()) p.is_target_user_set = false;
    PCN_LOG("[PolychaseTracker] cleared key @ frame " << frame
            << " from pin #" << p.id << " (slot " << (slot + 1) << ")\n");
    sync_blob_from_pins();
    return true;
}
// (is_target_user_set) and the recorded bindings first, so a previous Connect's
// stale targets on now-unmatched pins can't fight the fresh rigid solve, then
// runs Connect normally (which re-pairs, re-targets, solves, keys, re-records).
// -----------------------------------------------------------------------------
void PolychaseTracker::reconnect_pins_to_tracks()
{
    for (Pin& p : pins_) { p.is_target_user_set = false; p.linked_track = -1; p.linked_track_name.clear(); }
    sync_blob_from_pins();          // persist the cleared targets + links (undoable)

    PCN_LOG("[reconnect] cleared pin targets + links; running fresh Connect\n");
    connect_pins_to_tracks();       // re-pair, re-target, solve, key, re-record
}


// -----------------------------------------------------------------------------
// connect_pins_to_tracks — Pin <-> Track linking, v1 (auto-nearest).
//
// On the Reference Frame, pair each anchored user track to its nearest pin in
// 2D SCREEN pixels (1:1 — a pin taken by one track can't be grabbed by another;
// a track whose nearest free pin is beyond connect_max_dist_ is skipped). Each
// matched pin's target is set to its track's ref-frame pixel, then the existing
// rigid pin solve runs once and the pose is keyed (Set Pose Key). This is the
// "line the proxy up to the tracks before tracking" step.
//
// CONVENTION (load-bearing): the pin solve works in Nuke Y-UP image pixels
// (build_intrinsics_from_projection + un-flipped imatrix; pin target_x/y_px are
// viewer pixels). User-track observations were stored Y-DOWN (rebuild_user_tracks
// did image_point = (x, h - y) with the OpenCV intrinsics). So a track's pixel in
// pin space is  (image_point.x, fmt_h - image_point.y)  — undo the y-flip. Both
// the pairing distance and the applied target use that Y-up pixel.
//
// Requires the viewer to be parked on the Reference Frame: run_pin_solve and
// key_pose_at_current_frame both act on editing_frame(), and the camera basis is
// read at the current context — so editing_frame() must equal the ref frame for
// the solve and the key to land correctly. (Frame-independent Connect — sampling
// the ref camera and keying at ref regardless of the UI frame — is a later step.)
// -----------------------------------------------------------------------------
void PolychaseTracker::connect_pins_to_tracks()
{
    using namespace DD::Image;

    const int ref = user_track_ref_;
    if (editing_frame() != ref) {
        set_status("[" + timestamp() + "] Connect: go to the Reference Frame ("
                   + std::to_string(ref) + ") first, then press Connect. The solve "
                   "and the pose key are written on the frame you're parked on.");
        return;
    }

    CameraOp* cam = input_cam();
    Op*       geo = input_geo_op();
    Iop*      img = input_img();
    if (!cam || !geo) {
        set_status("[" + timestamp() + "] Connect: needs a camera and geo input.");
        return;
    }
    cam->validate(true);
    geo->validate(true);
    if (img) img->validate(true);

    float fmt_w = 2048.0f, fmt_h = 1080.0f;
    if (img) {
        const Format& fmt = img->info().format();
        fmt_w = (float)fmt.width();
        fmt_h = (float)fmt.height();
    }

    GeoMesh gm;
    if (!extract_mesh(geo, gm) || gm.local_vertices.rows() < 3) {
        set_status("[" + timestamp() + "] Connect: could not extract a mesh (>=3 verts).");
        return;
    }
    const Eigen::Index nverts = gm.local_vertices.rows();

    const UserTracks& uts = user_tracks_for_solve();
    if (uts.empty()) {
        set_status("[" + timestamp() + "] Connect: no anchored user tracks. Load Tracks "
                   "(and check the count anchored on the object) first.");
        return;
    }

    // Pin projection scene at the ref (== current) frame — identical convention
    // to run_pin_solve so the projected dots match what the viewer shows.
    SceneTransformations st{};
    st.model_matrix = live_scene_
                      ? live_scene_->model_matrix
                      : has_pose_keys()
                          ? nuke_to_eigen_m4(pose_matrix_to_nuke((double)ref))
                          : nuke_to_eigen_m4(gm.object_to_world);
    st.view_matrix  = nuke_to_eigen_m4(cam->imatrix());
    st.intrinsics   = build_intrinsics_from_projection(cam->projection(), fmt_w, fmt_h);

    // This frame's pins, each projected to a Nuke Y-up image pixel.
    struct PinProj { size_t slot; Eigen::Vector2f px; };
    std::vector<PinProj> pps;
    for (size_t i = 0; i < pins_.size(); ++i) {
        // One pin per vertex — every pin is a pairing candidate (the old per-frame
        // created_frame gate is gone). Pair by the vertex's projection at the ref.
        if ((Eigen::Index)pins_[i].vertex_idx >= nverts) continue;
        const Eigen::Vector3f lv(
            gm.local_vertices((Eigen::Index)pins_[i].vertex_idx, 0),
            gm.local_vertices((Eigen::Index)pins_[i].vertex_idx, 1),
            gm.local_vertices((Eigen::Index)pins_[i].vertex_idx, 2));
        pps.push_back({ i, project_local_to_image(lv, st) });
    }
    if (pps.empty()) {
        set_status("[" + timestamp() + "] Connect: no pins on the Reference Frame ("
                   + std::to_string(ref) + "). Place pins on the proxy first.");
        return;
    }

    // Track ref-frame pixels, converted from the stored Y-down image_point back
    // to Nuke Y-up (pin space). Skip a track with no observation on the ref frame.
    struct TrkPx { int num; Eigen::Vector2f px; };
    std::vector<TrkPx> tps;
    int tnum = 0;
    for (const UserTrack& t : uts) {
        ++tnum;   // 1-based, matches the viewer overlay label
        const Eigen::Vector2f* obs = nullptr;
        for (const UserTrackObservation& o : t.observations)
            if (o.frame_id == ref) { obs = &o.image_point; break; }
        if (!obs) continue;
        tps.push_back({ tnum, Eigen::Vector2f(obs->x(), fmt_h - obs->y()) });
    }
    if (tps.empty()) {
        set_status("[" + timestamp() + "] Connect: no user-track observations on the "
                   "Reference Frame (" + std::to_string(ref) + ").");
        return;
    }

    // Greedy nearest 1:1 within the pixel threshold (each track grabs its nearest
    // still-free pin; a pin can be grabbed once).
    const float maxd  = (float)connect_max_dist_;
    const float maxd2 = maxd * maxd;
    std::vector<bool> pin_used(pps.size(), false);
    struct Pair { size_t pin_slot; int track_num; Eigen::Vector2f target; float dist; };
    std::vector<Pair> pairs;
    for (const TrkPx& tp : tps) {
        int   best   = -1;
        float bestd2 = maxd2;
        for (size_t k = 0; k < pps.size(); ++k) {
            if (pin_used[k]) continue;
            const float d2 = (pps[k].px - tp.px).squaredNorm();
            if (d2 < bestd2) { bestd2 = d2; best = (int)k; }
        }
        if (best < 0) continue;   // nearest free pin is beyond the threshold
        pin_used[best] = true;
        pairs.push_back({ pps[best].slot, tp.num, tp.px, std::sqrt(bestd2) });
    }
    if (pairs.empty()) {
        std::ostringstream o;
        o << "[" << timestamp() << "] Connect: no pin within " << (int)maxd
          << "px of any track. Raise Connect Max Dist, or align the proxy closer "
             "to the tracks first.";
        set_status(o.str());
        return;
    }

    // Apply the matched targets (Nuke Y-up px) and record the per-pin links, then
    // solve + key. Connect REPLACES each matched pin's link with the current pairing.
    for (const Pair& pr : pairs) {
        Pin& pin = pins_[pr.pin_slot];
        const int tidx = pr.track_num - 1;            // 0-based into the anchored set
        pin.linked_track       = tidx;                // current index (volatile)
        // Stable id: the track's name at this anchored index. This is what persists
        // and what reresolve_pin_links() re-binds after any track rebuild.
        pin.linked_track_name  = ((size_t)tidx < user_track_names_.size())
                                 ? user_track_names_[(size_t)tidx] : std::string();
        pin.target_x_px        = pr.target.x();
        pin.target_y_px        = pr.target.y();
        pin.is_target_user_set = true;
        // Seed a key on the ref frame too, so the pin is immediately active there for
        // the solve below even if its track had no observation exactly on ref (the
        // pairing already used the ref pixel). The link drives every other frame.
        pin.keys[ref]          = pr.target;
        PCN_LOG("[connect] pin#" << pin.id << " (slot " << pr.pin_slot
                << ") <- track " << pr.track_num << " '" << pin.linked_track_name
                << "'  dist=" << pr.dist << "px\n");
    }

    // Clean (non-drag) rigid solve, mover = first matched pin, then Set Pose Key.
    dragging_pin_idx_ = -1;
    drag_anchor_.reset();
    run_pin_solve((int)pairs.front().pin_slot);

    if (live_scene_) {
        key_pose_at_current_frame();   // keys @ editing_frame() == ref, adds anchor
    } else {
        set_status("[" + timestamp() + "] Connect: the pin solve did not converge; "
                   "nothing keyed. Check the pin/track pairing in the viewer.");
        return;
    }

    // Discard the live preview now that the pose is keyed. Otherwise live_scene_
    // (the raw solve matrix) stays alive AND is persisted in live_pose_blob, so on
    // .nk reload it can win at the ref frame instead of the keyed curve — and its
    // restore timing varies, which is why the ref pose looked "different" after a
    // restart. Clearing it (+ empty blob) makes the deterministic keyed pose the
    // single source of truth everywhere. Keyframes are untouched.
    live_scene_.reset();
    live_edit_frame_ = -1000000;
    sync_blob_from_live_pose();

    std::ostringstream o;
    o << "[" << timestamp() << "] Connect: paired " << pairs.size() << " of "
      << tps.size() << " track(s) to pins; solved + keyed @ frame " << ref << ".";
    if ((int)pairs.size() < (int)tps.size())
        o << " " << (tps.size() - pairs.size()) << " skipped (no free pin within "
          << (int)maxd << "px).";
    set_status(o.str());

    sync_blob_from_pins();   // persist the new pin targets (undoable)
    invalidate();
    asapUpdate();
}


// -----------------------------------------------------------------------------
// track_via_pins — the "Use Only User Tracks" track mode. Exact per-frame PnP
// from pin<->track bindings, bypassing the optical-flow DB entirely.
//
// WHY this is exact where "user tracks only" (flow-domination) is not: the user-
// track solver anchors each track by RAYCASTING its ref pixel onto the mesh — an
// approximate, pose-derived surface point. A PIN instead names an EXACT mesh
// vertex (a real cube corner). When the pins sit on the same vertices the tracks
// were reconciled from, (vertex_3d, track_2d@frame) is a ground-truth 3D<->2D
// correspondence, so the per-frame PnP recovers the true pose with no drift.
//
// Flow: pair pins to tracks ONCE at the ref frame (auto-nearest 2D, same rule as
// Connect, within connect_max_dist_), then bake outward from ref in the requested
// direction. Each frame: sample the camera, gather the bound (vertex, track-pixel)
// pairs visible that frame, solve the rigid PnP (>=3 pairs), and key. Camera basis
// and intrinsics use the PIN convention (un-flipped imatrix + Nuke Y-up pixels),
// matching project_local_to_image / run_pin_solve.
// -----------------------------------------------------------------------------
bool PolychaseTracker::track_via_pins(bool forward, bool seed_solved_intrinsics)
{
    using namespace DD::Image;

    const int ref = user_track_ref_;

    CameraOp* cam = input_cam();
    Op*       geo = input_geo_op();
    Iop*      img = input_img();
    if (!cam || !geo) {
        set_status("[" + timestamp() + "] Track (via pins): needs a camera and geo input.");
        return false;
    }
    cam->validate(true);
    geo->validate(true);
    if (img) img->validate(true);

    float fmt_w = 2048.0f, fmt_h = 1080.0f;
    if (img) {
        const Format& fmt = img->info().format();
        fmt_w = (float)fmt.width();
        fmt_h = (float)fmt.height();
    }

    GeoMesh gm;
    if (!extract_mesh(geo, gm) || gm.local_vertices.rows() < 3) {
        set_status("[" + timestamp() + "] Track (via pins): could not extract a mesh.");
        return false;
    }

    // Eligible pins drive the bake: any pin with at least one manual key OR a track
    // link has a resolvable 2D across the range (keys interpolate/clamp; a link
    // follows its track where observed). Need >=3 for a rigid PnP.
    int eligible = 0;
    for (const Pin& p : pins_)
        if (!p.keys.empty() || p.linked_track >= 0) ++eligible;
    if (eligible < 3) {
        std::ostringstream o;
        o << "[" << timestamp() << "] Track (via pins): only " << eligible
          << " pin(s) with a key or track link; need >=3 for a pose. Key a few "
             "corners by hand, or Connect pins to tracks.";
        set_status(o.str());
        return false;
    }

    // Horizontal aperture (mm) for the focal pixel<->mm map (fx_px = focal_mm *
    // w / haperture). Read once; aperture is frame-constant. Consulted every frame
    // now: both for the camera's own per-frame focal (below) and for the optional
    // solved-focal override (seed_solved_intrinsics, used by the focal solve's
    // internal re-track before the focal is baked onto the camera).
    double haperture = 24.576;
    if (Knob* hk = cam->knob("haperture")) haperture = hk->get_value();
    const bool seed_focal     = seed_solved_intrinsics && has_solved_focal();
    const bool seed_principal = seed_solved_intrinsics && has_solved_principal();

    // Sample the camera basis at one frame (OutputContext swap, restore after).
    auto cam_basis_at = [&](int frame, DD::Image::Matrix4& proj, DD::Image::Matrix4& imat) {
        OutputContext oc0 = cam->outputContext();
        OutputContext oc  = oc0;
        oc.setFrame((double)frame);
        cam->setOutputContext(oc);
        cam->validate(true);
        proj = cam->projection();
        imat = cam->imatrix();
        cam->setOutputContext(oc0);
        cam->validate(true);
    };

    // Warm-start seed = the ref-frame pose.
    const RowMajorMatrix4f ref_model = has_pose_keys()
        ? nuke_to_eigen_m4(pose_matrix_to_nuke((double)ref))
        : nuke_to_eigen_m4(gm.object_to_world);

    // Synthetic prior (spread verts) — conditions the solve / holds DOFs when a
    // frame momentarily drops below 3 visible pairs. Negligible at the exact pose.
    std::vector<unsigned> synthv = pick_spread_anchor_verts(gm.local_vertices);
    RowMajorMatrixX3f synth_pts((Eigen::Index)synthv.size(), 3);
    for (size_t i = 0; i < synthv.size(); ++i) {
        synth_pts((Eigen::Index)i, 0) = gm.local_vertices((Eigen::Index)synthv[i], 0);
        synth_pts((Eigen::Index)i, 1) = gm.local_vertices((Eigen::Index)synthv[i], 1);
        synth_pts((Eigen::Index)i, 2) = gm.local_vertices((Eigen::Index)synthv[i], 2);
    }

    // ---- Bake outward from ref in the requested direction --------------------
    const int end  = forward ? last_frame_ : first_frame_;
    const int step = forward ? +1 : -1;

    RowMajorMatrix4f prev_model = ref_model;
    int keyed = 0, skipped = 0;

    for (int f = ref; forward ? (f <= end) : (f >= end); f += step) {
        DD::Image::Matrix4 proj_f, imat_f;
        cam_basis_at(f, proj_f, imat_f);

        // cam->projection() carries a FROZEN focal even under the forced
        // OutputContext above (the cooked-projection staleness conv::apply_curve_focal
        // exists for), so proj_f's diagonal is a single panel-frame value, NOT the
        // per-frame lens. Rewrite it from the camera's focal CURVE at this frame so an
        // ANIMATED camera focal actually drives the track per frame. This is the path
        // that makes the workflow "Solve Focal -> Copy Solved Focal to Camera -> Track"
        // honour the zoom with no extra toggle: once the zoom lives on the camera, the
        // track reads it here. No-op for a static lens (same value the cook produced).
        {
            double cam_focal_mm = 0.0;
            if (Knob* fk = cam->knob("focal")) cam_focal_mm = fk->get_value_at((double)f);
            conv::apply_curve_focal(proj_f, cam_focal_mm, haperture);
        }

        // Each pin's resolved 2D on this frame: a manual key (interpolated between
        // keys / clamped past the ends) OR its linked track where observed. That's
        // the (vertex_3d, target_2d) correspondence for the PnP. resolve_pin_2d
        // already returns Nuke Y-up pixels, matching the pin solve convention.
        std::vector<Eigen::Vector3f> objs;
        std::vector<Eigen::Vector2f> tgts;
        objs.reserve(pins_.size());
        tgts.reserve(pins_.size());
        for (const Pin& p : pins_) {
            if ((Eigen::Index)p.vertex_idx >= gm.local_vertices.rows()) continue;
            const std::optional<Eigen::Vector2f> r2 =
                resolve_pin_2d(p, f, fmt_h, /*allow_interp=*/true);
            if (!r2) continue;
            objs.push_back(Eigen::Vector3f(
                gm.local_vertices((Eigen::Index)p.vertex_idx, 0),
                gm.local_vertices((Eigen::Index)p.vertex_idx, 1),
                gm.local_vertices((Eigen::Index)p.vertex_idx, 2)));
            tgts.push_back(*r2);   // already Nuke y-up
        }
        if ((int)objs.size() < 3) { ++skipped; continue; }

        RowMajorMatrixX3f obj_pts((Eigen::Index)objs.size(), 3);
        RowMajorMatrixX2f real_t((Eigen::Index)tgts.size(), 2);
        Eigen::ArrayXf    real_w((Eigen::Index)objs.size());
        real_w.setConstant(1.0f);
        for (size_t i = 0; i < objs.size(); ++i) {
            obj_pts((Eigen::Index)i, 0) = objs[i].x();
            obj_pts((Eigen::Index)i, 1) = objs[i].y();
            obj_pts((Eigen::Index)i, 2) = objs[i].z();
            real_t((Eigen::Index)i, 0)  = tgts[i].x();
            real_t((Eigen::Index)i, 1)  = tgts[i].y();
        }

        SceneTransformations initial{}, current{};
        initial.model_matrix = prev_model;                                  // warm start
        initial.view_matrix  = nuke_to_eigen_m4(imat_f);
        initial.intrinsics   = build_intrinsics_from_projection(proj_f, fmt_w, fmt_h);

        // EXPLICIT solved-curve override (seed_solved_intrinsics). The base focal
        // above already follows the camera's per-frame lens, so this is only used by
        // the focal solve's INTERNAL re-track (tracker_intrinsics.cpp), which must
        // re-track under a just-fit solved focal BEFORE it has been baked onto the
        // camera with 'Copy Solved Focal -> Camera'. For a normal user track the flag
        // is off and this is skipped — the camera focal (above) governs.
        // build_intrinsics_from_projection is the OpenGL/overlay convention (Y-UP,
        // NEGATIVE fx/fy), so preserve that sign and the fx/fy aspect when overriding
        // the magnitude, and flip the principal Y (cy_gl = h - cy_cv; solved_cy is
        // OpenCV / y-down). No-op when nothing was solved.
        if (seed_focal) {
            if (Knob* sf = knob("solved_focal")) {
                const double focal_mm = sf->get_value_at((double)f);
                if (std::isfinite(focal_mm) && focal_mm > 1e-6 &&
                    fmt_w > 0.0f && haperture > 1e-6) {
                    const double fx_px  = focal_mm * (double)fmt_w / haperture;
                    const double aspect = (std::abs(initial.intrinsics.aspect_ratio) > 1e-6f)
                                              ? (double)initial.intrinsics.aspect_ratio : 1.0;
                    initial.intrinsics.fx = -(float)fx_px;                 // GL: negative
                    initial.intrinsics.fy = (float)(initial.intrinsics.fx / aspect);
                }
            }
        }
        if (seed_principal) {
            if (Knob* kx = knob("solved_cx"))
                initial.intrinsics.cx = (float)kx->get_value_at((double)f);
            if (Knob* ky = knob("solved_cy"))
                initial.intrinsics.cy = (float)((double)fmt_h - ky->get_value_at((double)f));
        }

        current = initial;

        const SceneTransformations out = solve_pins_weighted(
            obj_pts, real_t, real_w, synth_pts, 0.01f, initial, current);

        if (!out.model_matrix.allFinite()) { ++skipped; continue; }
        key_pose_matrix_at((double)f, out.model_matrix);
        prev_model = out.model_matrix;
        ++keyed;
    }

    // The ref frame is the trusted anchor; clear any live preview so the overlay
    // reads the freshly keyed curve. Also clear the persisted live_pose_blob so a
    // stale live pose can't restore at the ref frame on .nk reload.
    live_scene_.reset();
    live_edit_frame_ = -1000000;
    sync_blob_from_live_pose();
    add_refine_anchor(ref);

    std::ostringstream o;
    o << "[" << timestamp() << "] Track (via pins): keyed " << keyed
      << " frame(s) " << ref << (forward ? " -> " : " <- ") << end
      << " from " << eligible << " pin(s) (keys + track links).";
    if (skipped) o << "  " << skipped << " frame(s) skipped (<3 pins resolvable).";
    set_status(o.str());
    PCN_LOG("[track-pins] keyed=" << keyed << " skipped=" << skipped
            << " eligible=" << eligible << " dir=" << (forward ? "fwd" : "bwd") << "\n");
    asapUpdate();
    return keyed > 0;
}

} // namespace pcn