// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Peter Mercell
//
// Developed with assistance from Claude (Anthropic).

// =============================================================================
// polychase_tracker.h  —  class declarations for the PolychaseTracker plugin.
// The two classes (overlay knob + Op) are declared here so their method
// definitions can be split across translation units. namespace `pcn`.
// =============================================================================
#ifndef POLYCHASE_TRACKER_H
#define POLYCHASE_TRACKER_H

// DDImage's NoIop.h transitively includes GL/glew.h, which hard-errors if a
// plain GL/gl.h was seen first. So the DDImage headers must come BEFORE
// polychase_util.h (whose ViewerContext.h can pull in gl.h).
#include "DDImage/NoIop.h"
#include "DDImage/Iop.h"
#include "DDImage/Knobs.h"
#include "DDImage/Knob.h"
#include "DDImage/Row.h"
#include "DDImage/Channel.h"

#include "polychase_util.h"

#include <chrono>
#include <ctime>
#include <list>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <utility>          // std::pair
#include <unordered_map>
#include <vector>

namespace pcn {

using namespace DD::Image;

// -----------------------------------------------------------------------------
// Pin-refine interaction model. The old gizmo-only hard-lock
// is gone: the node now always sits in "Both" — the gizmo AND the pins are
// drawn at all times. A sticky `shift+P` toggle (pin_input_active_) decides who
// receives the mouse:
//   - armed   (shift+P on)  → pins get the mouse; the gizmo is drawn but inert/dimmed
//   - disarmed(shift+P off) → the gizmo gets the mouse; pins are inert (still drawn)
// There is therefore no gizmo-handle-vs-pin-dot hit-test arbitration anywhere.
// The Manipulator selector stays hidden; shift+P (or the "Pin Edit" checkbox) is the
// only control. The gizmo and pins both read and write the SAME rigid pose
// (model_matrix), so editing with one continues from where the other left off.
// -----------------------------------------------------------------------------

// Forward declaration so the knob can hold a typed pointer back to its owning Op.
class PolychaseTracker;

// -----------------------------------------------------------------------------
// PolychaseWireframeKnob — owns 2D-viewer wireframe drawing.
//
// Pulls geo from input 2, camera from input 1, image from input 0, projects
// every triangle edge through view*projection into image-pixel coordinates,
// emits GL_LINES. Runs on every viewer redraw in 2D mode (transform_mode 0).
//
// Coordinate-system contract for 2D viewer (validated 1C.1c):
//   - GL coords are image pixels, Y up, origin at bottom-left of format
//   - We do NOT touch GL_PROJECTION / GL_MODELVIEW; viewer's pan+zoom is
//     applied externally to whatever we emit
//
// Projection math (Nuke's CameraOp convention, OpenGL-style column-major):
//   mvp     = projection() * imatrix() * info.matrix    (per-object compose)
//   v_clip  = mvp * v_local
//   v_ndc   = v_clip.xyz / v_clip.w
//   v_pixel = v_ndc.xy * (fmt_w/2) + (fmt_w/2, fmt_h/2)  (uniform scale)
//   discard if v_clip.w <= 0 (behind camera)
//
// Per-object matrix composition is what gives us live transform updates:
// changing a TransformGeo knob upstream changes info.matrix, and we
// re-read it every draw, so the wireframe follows the geo immediately.
// -----------------------------------------------------------------------------
class PolychaseWireframeKnob : public DD::Image::Knob
{
public:
    // CustomKnob1 ctor signature: (Knob_Closure*, void* pointer, const char* name).
    // The void* is the `this` we passed at registration time — the owning Op.
    PolychaseWireframeKnob(DD::Image::Knob_Closure* kc,
                           void* pointer,
                           const char* name)
        : DD::Image::Knob(kc, name)
        , owner_(static_cast<PolychaseTracker*>(pointer))
    {}

    const char* Class() const override { return "PolychaseWireframe"; }

    // Participate in BOTH 2D and 3D viewer drawing. In Nuke 17 legacy draw_handle
    // still composites over the Hydra 3D viewport (same path Axis4/Camera4 use for
    // their gnomons), so returning true here makes draw_handle fire in 3D too;
    // draw_handle then branches on viewer_mode() — manual plate projection in 2D,
    // world-space GL in 3D.
    bool build_handle(DD::Image::ViewerContext* /*ctx*/) override
    {
        return true;
    }

    void draw_handle(DD::Image::ViewerContext* ctx) override;

private:
    // 3D-viewport overlay: draw the solved proxy wireframe + pins in WORLD space
    // (the viewer has already loaded its camera into GL), no manual projection.
    void draw_world_overlay_3d(DD::Image::ViewerContext* ctx);

    PolychaseTracker* owner_;

    // ---- GL state captured during DRAW_OPAQUE -----------------
    // mouse_x()/mouse_y() return viewer-widget screen pixels; to compare
    // them against image-pixel vertex projections we need the viewer's
    // current MV+P+VP. These are captured at the end of each DRAW_OPAQUE
    // and reused when a PUSH arrives.
    mutable double cached_mv_[16] = {0};
    mutable double cached_pj_[16] = {0};
    mutable int    cached_vp_[4]  = {0};
    mutable bool   cache_valid_   = false;
};

// -----------------------------------------------------------------------------
// PolychaseTracker — NoIop subclass with three typed inputs.
//
// Why NoIop (not plain Op):
//   NoIop is "Iop that passes input 0 through unchanged with optional info
//   modifications." That gives us 2D-viewer passthrough of the plate — the
//   artist sees the footage when they wire Viewer to this node. Inputs 1 (cam)
//   and 2 (geo) are non-image typed inputs that don't participate in the
//   image pipeline.
//
//   The hazard with NoIop + minimum_inputs=0 is its default _validate() calls
//   copy_info() on input0() which NULL-derefs when nothing is connected. We
//   override _validate to guard against that.

// -----------------------------------------------------------------------------
class PolychaseTracker : public NoIop
#ifdef PCN_NEW_3D
                       , public DD::Image::GeometryProviderI
#endif
{
public:
    explicit PolychaseTracker(Node* node);

    const char* Class()     const override { return description.name; }
    const char* node_help() const override { return kNodeHelp; }

    // -------- Input wiring --------
    int  minimum_inputs() const override { return 0; }   // all optional
    int  maximum_inputs() const override { return 4; }   // img, cam, geo, mask

    // Type-gate each pin. Nuke greys out wires that don't match.
    bool test_input(int idx, Op* op) const override
    {
        if (!op) return true;                  // disconnecting is always fine
        switch (idx) {
            case kInputImg: return dynamic_cast<Iop*>(op)      != nullptr;
            case kInputCam: return dynamic_cast<CameraOp*>(op) != nullptr;  // Camera4 too (CameraSceneOp:CameraOp)
#ifdef PCN_NEW_3D
            // Accept new-system geometry (GeoCube / any GeomOp) as well as classic
            // GeoOp. geomOp() matches ONLY GeomOp geometry — unlike geometryProvider(),
            // it does NOT also match Camera4/Axis4/lights (they implement the provider
            // for their viewport icons), so a camera can't be dropped on the geo input.
            case kInputGeo: return op->geomOp() != nullptr || dynamic_cast<GeoOp*>(op) != nullptr;
#else
            case kInputGeo: return dynamic_cast<GeoOp*>(op)    != nullptr;
#endif
            case kInputMask: return dynamic_cast<Iop*>(op)     != nullptr;  // Roto / any Iop alpha
            default:        return false;
        }
    }

    // Hover labels shown on each input pin in the node graph.
    const char* input_label(int idx, char* /*buffer*/) const override
    {
        switch (idx) {
            case kInputImg: return "img";
            case kInputCam: return "cam";
            case kInputGeo: return "geo";
            case kInputMask: return "mask";
            default:        return "";
        }
    }

    // Image-pipeline validate. NoIop's default copies info from input(0)
    // unconditionally — which NULL-derefs when input 0 isn't connected.
    // Guard against that: if input 0 is wired, pass through normally
    // (2D viewer shows the plate); otherwise mask off output channels so
    // the viewer/engine skips this node entirely.
    void _validate(bool for_real) override
    {
        // Make sure pins_ reflect the saved blob after a .nk load. knob_changed
        // for pins_blob doesn't reliably fire on load, so the lazy loader (which
        // reloads when the blob content differs from what we last parsed) is the
        // reliable hook. This only reads state — it writes no knobs — so it's
        // safe to call here.
        ensure_pins_loaded();
        if (input(0)) {
            NoIop::_validate(for_real);
        } else {
            set_out_channels(Mask_None);
        }
    }

    // -------- Viewer overlay --------
    //
    // Viewer drawing for this Op is owned by PolychaseWireframeKnob (registered
    // in knobs() via CustomKnob1). Op::draw_handle only fires in 3D scene
    // mode with worldspace GL coordinates, which is not what we want for
    // overlaying the 2D plate. The default NoIop::build_handles handles
    // input/knob recursion correctly without us overriding it.

    // -------- Knobs --------
    void knobs(Knob_Callback f) override;
    int  knob_changed(Knob* k) override;

    // -------- Pin API used by the knob --------
    //
    // The knob calls these on mouse events in the 2D viewer:
    //
    //   on_mouse_push   — decides between placing a new pin (if the click
    //                     isn't near an existing pin) or starting a drag
    //                     (if it is)
    //   on_mouse_drag   — updates the dragged pin's target_x/y to follow
    //                     the cursor. Updates pins_ in place and triggers
    //                     a viewer redraw, but does NOT capture undo on
    //                     every mouse-move (that would create dozens of
    //                     undo entries per drag).
    //   on_mouse_release — commits the drag: writes pins_ → pins_blob
    //                     (single undo entry for the whole drag).
    //
    // All three need the GL state cached during the last DRAW_OPAQUE for
    // screen↔image coord conversion.
    //
    // The knob calls pins() during draw_handle to iterate and project the
    // pin positions onto the current frame.
    //
    // All mutations go through sync_blob_from_pins() so the serialized
    // pins_blob_ knob captures the change — that gives us .nk persistence
    // AND undo/redo for free (Nuke's undo stack captures knob changes).
    void on_mouse_push   (DD::Image::ViewerContext* ctx,
                          const double mv[16], const double pj[16],
                          const int vp[4], bool gl_state_valid);
    void on_mouse_drag   (DD::Image::ViewerContext* ctx,
                          const double mv[16], const double pj[16],
                          const int vp[4], bool gl_state_valid);
    void on_mouse_release(DD::Image::ViewerContext* ctx,
                          const double mv[16], const double pj[16],
                          const int vp[4], bool gl_state_valid);

    // Apply the 3D-viewer VERTEX selection (routed to this node's GEOSELECT_KNOB)
    // to the mask: a triangle changes only when all three of its corner points are
    // selected (strict). erase=false masks them, erase=true unmasks them — the two
    // "Mask Selected" / "Unmask Selected" buttons. Defined in tracker_mask.cpp.
    void apply_3d_vertex_selection(bool erase);

    const std::vector<Pin>& pins() { ensure_pins_loaded(); return pins_; }

    // Resolve a pin's 2D screen position at `frame`, in Nuke Y-UP pixels:
    //   1. a manual key on `frame`            (manual always wins)
    //   2. the linked user track at `frame`   (track store y-down -> y-up)
    //   3. linear interpolation between the pin's bracketing manual keys
    //   4. none — the pin has no 2D there (un-keyed, un-linked frame)
    // allow_interp=false stops at step 2 (no interpolation): used by the live
    // solve, where a frame with no explicit key/link should anchor at the pin's
    // current projection, not get pulled to a stale interpolated spot. fmt_h is
    // the format height (track y-flip). Non-const: may build the user-track set.
    std::optional<Eigen::Vector2f> resolve_pin_2d(const Pin& pin, int frame,
                                                  float fmt_h, bool allow_interp = true);
    // Poll the live_pose_blob every redraw and rebuild live_scene_ + the offset
    // readout when it differs from our cache. This is how a gizmo/pin gesture
    // UNDOES: knob_changed("live_pose_blob") is unreliable for INVISIBLE knobs
    // (same reason pins reload via a poller, not the event), so the draw pass
    // catches the reverted blob on the next frame.
    void ensure_live_pose_loaded();
    // Draw-time detector for our own Undo/Redo Move history: watches the shared
    // offset knobs and records a snapshot when a move settles. Public because the
    // wireframe draw pass calls it (same as ensure_live_pose_loaded above).
    void ensure_move_history();
    // Slot index of the pin currently being dragged, or -1. Used by the draw
    // pass to colour the active pin red and the rest amber.
    int dragging_pin_slot() const { return dragging_pin_idx_; }
    // True while a pin is actively being dragged (a drag tick landed very recently).
    // Drives the red/amber dot colour: red while moving, amber the moment it rests —
    // without touching dragging_pin_idx_, so the drag logic is unaffected.
    bool pin_drag_is_moving() const;
    void clear_pins();
    void delete_pin_at(unsigned slot);
    bool clear_pin_key_at(unsigned slot, int frame);   // drop one pin's key @ frame

    // Sentinel for effective_model_matrix's optional `frame` arg: "no explicit
    // frame, use the UI frame". A magic double rather than std::optional only to
    // keep the const, header-declared signature trivial; any value this negative
    // means "unset" (the draw path always passes a real uiContext frame).
    static constexpr double kNoFrame = -1e30;

    // Accessor used by PolychaseWireframeKnob to pick between
    // the solved pose (when live_scene_ is set) and the upstream geo
    // transform. Public because the knob is a separate class.
    DD::Image::Matrix4 effective_model_matrix(
        const DD::Image::Matrix4& upstream_default,
        double frame = kNoFrame) const;

    // Viewer-frame bridge. The Op's outputContext() lags the playhead while
    // scrubbing (it's served from the viewer cache without re-validating — see
    // the note in PolychaseWireframeKnob::draw_handle), so anything driven by a
    // mouse/key event must NOT read the current frame from outputContext() or it
    // operates on a stale frame (e.g. placing a pin on frame 45 created it on
    // frame 2 until Pin Edit was toggled). The overlay knob already evaluates at
    // uiContext().frame(), which tracks the playhead exactly; it pushes that here
    // on every draw_handle call so the interaction code reads the displayed frame.
    void set_editing_frame(int f) { editing_frame_ = f; has_editing_frame_ = true; }
    int  editing_frame() const;   // UI frame if known, else rounded outputContext

    // Draw the pose gizmo (translate/rotate handles) over the plate.
    // Called by the wireframe knob during DRAW_OPAQUE, after the wireframe/pins
    // passes. Draw-only for now (no interaction). Anchored at the proxy origin
    // (obj_to_world translation), oriented by the proxy axes, drawn at a fixed
    // SCREEN size. No-op when the Manipulator mode is "Pins". `view_proj`,
    // `scale`, `cx`, `cy` are the same projection the wireframe uses, so the
    // gizmo overlays the proxy exactly.
    void draw_gizmo(const DD::Image::Matrix4& view_proj,
                    const DD::Image::Matrix4& obj_to_world,
                    DD::Image::CameraOp* cam,
                    float scale, float cx, float cy,
                    const DD::Image::Vector3& pivot_local) const;

    // Manipulator mode (0=Gizmo, 1=Pins, 2=Both). Public so the wireframe knob
    // can gate pin drawing (pins are hidden in Gizmo-only mode).
    int manipulator_mode() const { return manipulator_mode_; }

    // Wireframe overlay colour (RGB), bound to the "wire_color" Color_knob.
    // Public so PolychaseWireframeKnob::draw_handle can read it each redraw and
    // tint the projected edges. Display-only: it never touches the solve/export.
    //
    // Reads the LIVE knob value (shared across Op instances) rather than the bound
    // member: the viewer overlay frequently runs on a different C++ instance than
    // the panel that was edited, and that instance's member doesn't refresh until it
    // re-validates (which only happened on a timeline scrub — the "must scrub to see
    // it" lag). Polling the knob is the same trick ensure_live_pose_loaded() uses for
    // the pose, so a colour/gradient change shows on the very next redraw. Falls back
    // to the member if the knob isn't built yet.
    void wire_color(float out[3]) const
    {
        if (DD::Image::Knob* k = knob("wire_color")) {
            out[0] = (float)k->get_value(0);
            out[1] = (float)k->get_value(1);
            out[2] = (float)k->get_value(2);
        } else {
            out[0] = wire_color_[0];
            out[1] = wire_color_[1];
            out[2] = wire_color_[2];
        }
    }
    // Axis-gradient toggle, bound to the "wire_gradient" Bool_knob. When on, the
    // wireframe is coloured by each vertex's LOCAL position (mesh origin 0,0,0):
    // X→red, Y→green, Z→blue. Object-local, so the colours rotate with the mesh.
    // Read from the live knob for the same cross-instance reason as wire_color().
    bool wire_gradient() const
    {
        if (DD::Image::Knob* k = knob("wire_gradient")) return k->get_value() != 0.0;
        return wire_gradient_;
    }

    // ---- Mask overlay accessors (read by the wireframe knob) ----------------
    // RGBA tint colour for masked triangles; live knob first (so a colour change
    // repaints immediately), member fallback. has_mask() lets the overlay skip the
    // whole tint pass when nothing is masked. is_triangle_masked() (defined in
    // tracker_mask.cpp) is the per-triangle query the tint loop calls.
    void mask_color(float out[4]) const
    {
        if (DD::Image::Knob* k = knob("mask_color")) {
            out[0] = (float)k->get_value(0);
            out[1] = (float)k->get_value(1);
            out[2] = (float)k->get_value(2);
            out[3] = (float)k->get_value(3);
        } else {
            out[0] = mask_color_[0];
            out[1] = mask_color_[1];
            out[2] = mask_color_[2];
            out[3] = mask_color_[3];
        }
    }
    bool has_mask() const
    {
        for (uint32_t w : mask_bits_) if (w) return true;
        return false;
    }
    bool is_triangle_masked(uint32_t tri) const;   // defined in tracker_mask.cpp
    // Reload the mask from the saved blob whenever the blob content differs from
    // what we last (de)serialized — covers .nk load (where knob_changed("mask_blob")
    // commonly does NOT fire and the showPanel hook may be defeated by a premature
    // load). Polled every redraw in the overlay, exactly like ensure_live_pose_loaded,
    // so a reopened script shows + solves with its mask without touching the panel.
    void ensure_mask_loaded()
    {
        if (!mask_loaded_ || mask_mirror_.needs_reload(mask_blob_)) load_mask_from_knob();
    }
    // Bumped on every mask mutation; lets the overlay/redraw cache detect changes.
    uint64_t mask_version() const { return mask_version_; }

    // ---- 2D occlusion mask (mask plate) accessors -------------------------
    // Read the LIVE knob value (shared across Op instances) so the viewer overlay
    // reflects a change on the next redraw, same trick as wire_color(). Mode: 0
    // None, 1 Mask Alpha, 2 Mask Alpha Inverted.
    int mask2d_mode() const {
        if (DD::Image::Knob* k = knob("mask2d_mode")) return (int)k->get_value();
        return mask2d_mode_;
    }
    double mask2d_threshold() const {
        if (DD::Image::Knob* k = knob("mask2d_threshold")) return k->get_value();
        return mask2d_threshold_;
    }
    bool mask2d_show() const {
        if (DD::Image::Knob* k = knob("mask2d_show")) return k->get_value() != 0.0;
        return mask2d_show_;
    }
    void mask2d_color(float out[4]) const {
        if (DD::Image::Knob* k = knob("mask2d_color")) {
            out[0] = (float)k->get_value(0); out[1] = (float)k->get_value(1);
            out[2] = (float)k->get_value(2); out[3] = (float)k->get_value(3);
        } else {
            out[0] = mask2d_color_[0]; out[1] = mask2d_color_[1];
            out[2] = mask2d_color_[2]; out[3] = mask2d_color_[3];
        }
    }
    // Build (cached) the current-frame masked-pixel bitset for the viewer overlay:
    // packed 1 bit/pixel, Y-up (Nuke) orientation, set bit = masked. Returns
    // nullptr when the 2D mask is off / no mask input. Defined in tracker_mask2d.cpp.
    const std::vector<uint32_t>* mask2d_overlay_bits(int frame, int& w, int& h);

    // ---- User (helper) track accessors ------------------------------------
    bool user_tracks_show() const {
        if (DD::Image::Knob* k = knob("user_tracks_show")) return k->get_value() != 0.0;
        return user_tracks_show_;
    }
    // Overlay index labels (1-based) on pin dots and user-track anchor dots. One
    // toggle for both; rides the same draw passes as the markers themselves.
    bool show_numbers() const {
        if (DD::Image::Knob* k = knob("show_numbers")) return k->get_value() != 0.0;
        return show_numbers_;
    }
    void user_tracks_color(float out[4]) const {
        if (DD::Image::Knob* k = knob("user_tracks_color")) {
            out[0] = (float)k->get_value(0); out[1] = (float)k->get_value(1);
            out[2] = (float)k->get_value(2); out[3] = (float)k->get_value(3);
        } else {
            out[0] = user_tracks_color_[0]; out[1] = user_tracks_color_[1];
            out[2] = user_tracks_color_[2]; out[3] = user_tracks_color_[3];
        }
    }
    // Anchored user tracks (object-space point + per-frame observations in OpenCV/
    // y-down px), rebuilt on demand when inputs change. Consumed by Track/Refine
    // and the viewer overlay. Defined in tracker_usertracks.cpp.
    const UserTracks& user_tracks_for_solve();

    // The SOLVE set handed to flow Track and Refine. When Connect has recorded
    // bindings, this is the CONNECTED tracks only, each re-anchored at its EXACT
    // pinned vertex (no raycast lossiness; unconnected tracks are not used, per
    // the Connect model). With no bindings it returns the full anchored set, so a
    // non-Connect workflow is unchanged. Returned by value (anchors are rewritten).
    UserTracks build_solve_tracks();

    // ---- Pin-refine routing -------------------
    // `pin_input_active_` is the sticky arm/disarm flag toggled by the `shift+P` key
    // (or the "Pin Edit" checkbox). Armed → the mouse drives pins and the gizmo
    // is inert; disarmed → the mouse drives the gizmo and pins are inert. It
    // NEVER touches the pose: gizmo↔pins compose through the same model_matrix
    // regardless of this flag. Public so the wireframe knob can read it (to
    // dim the gizmo / brighten the pins / draw the HUD) and toggle it on `shift+P`.
    bool pin_input_active() const
    {
        // Read the LIVE knob value (shared across Op instances) rather than the bound
        // member, so the viewer overlay reflects an arm/disarm on the very next redraw
        // instead of lagging until the instance re-validates (the "must scrub to see
        // Pin Edit" issue — same fix as wire_gradient()). Falls back to the member if
        // the knob isn't built yet.
        if (DD::Image::Knob* k = knob("pin_input_active")) return k->get_value() != 0.0;
        return pin_input_active_;
    }
    void toggle_pin_input();      // flip arm state (shift+P keydown / checkbox)

    // ---- Gizmo interaction --------------------------------------
    // The wireframe knob routes PUSH/DRAG/RELEASE here first (in Gizmo/Both
    // modes); a handle hit consumes the event, a miss falls through to pins.
    bool gizmo_dragging() const { return gizmo_active_handle_ >= 0; }
    // Returns the gizmo handle under the cursor, or -1. G1: only the centre (0).
    int  gizmo_hit_test  (DD::Image::ViewerContext* ctx,
                          const double mv[16], const double pj[16], const int vp[4]);
    // Begin a gizmo drag if a handle is hit; returns true if the event is consumed.
    bool on_gizmo_push   (DD::Image::ViewerContext* ctx,
                          const double mv[16], const double pj[16], const int vp[4]);
    void on_gizmo_drag   (DD::Image::ViewerContext* ctx,
                          const double mv[16], const double pj[16], const int vp[4]);
    void on_gizmo_release();

    // Live rotation-offset preview. The Rotate X/Y/Z knobs hold an absolute
    // offset (degrees) layered on the keyed/upstream base about the mesh centre;
    // editing one recomputes the preview pose into live_scene_ WITHOUT keying.
    // "Set Pose Key" bakes it and reset_rot_offsets() folds the fields back to 0.
    void preview_center_rotation();   // rot_x/y/z -> live_scene_, no key
    void reset_rot_offsets();         // rot_x/y/z -> 0 without re-firing the preview
    bool rot_offset_pending() const;  // any of rot_x/y/z != 0 (read from knobs)

    // Live translate/dolly-offset preview — the exact analogue of the rotation
    // helpers above, and the reason a gizmo move is now undoable: trans_x/y/z hold
    // a WORLD-space translation offset and `dolly` a signed distance along the
    // frozen camera->mesh viewing ray, both layered on a frozen base
    // (trans_base_). Editing a field, OR a gizmo translate/dolly drag, recomputes
    // live_scene_ WITHOUT keying; "Set Pose Key" bakes it and reset_trans_offsets()
    // folds the fields back to 0. The gizmo drag writes these knobs on RELEASE
    // (one undo entry per gesture), so Ctrl+Z reverts the move through the same
    // knob_changed -> preview path the Rotate knobs use.
    void preview_translate_offset();  // trans_x/y/z + dolly -> live_scene_, no key
    void reset_trans_offsets();       // trans_x/y/z + dolly -> 0, no re-fire
    bool trans_offset_pending() const;// any of trans_x/y/z/dolly != 0 (from knobs)

    // Unified offset<->pose maps shared by the typed knobs, the gizmo drag and the
    // pin solve. recompute_pose_from_offsets() reads the seven persistent offset
    // knobs and composes them onto the upstream entry-point base (rotation + dolly
    // about the current geo centroid) into live_scene_. sync_offsets_from_pose()
    // is the exact inverse: it reads an authoritative pose (e.g. a pin solve) back
    // into the translate/rotate knobs so the readout never lies.
    void recompute_pose_from_offsets();
    void sync_offsets_from_pose(const RowMajorMatrix4f& working);

    // Pin cap. The solve dispatches on pin count (polychase FindTransformation):
    // 1 pin = pan, 2 = similarity (2D rotate+scale), 3+ = rigid PnP (3D pose).
    // All placed pins are fed to the solver; the dragged pin moves to its new
    // target while the rest act as "stay" anchors. 16 is plenty of headroom for
    // hand alignment. Gates both placement and the delete-index range.
    static constexpr int kMaxPinSlots = 64;

    static Op::Description description;

private:
    // ---- Persistent serialized state ---------------------------------------
    const char* db_path_       = nullptr;
    int         first_frame_   = 1;
    int         last_frame_    = 100;
    int         solve_mode_    = 0;        // 0=Camera, 1=Model
    // "Export Zoom Lens Camera" (Model mode): also spawn 'Polychase_LensCamera_1'
    // matching the input camera's world pose but carrying the solved zoom focal, so
    // the exported TransformGeo can be rendered through a camera with the right lens.
    // Bound to the "export_lens_camera" Bool_knob; shown only in Model mode (hidden in
    // Camera mode, where the spawned camera already owns the focal). Default ON. Read
    // at export time; no-op in Camera mode / without a solved focal / without a cam.
    bool        export_lens_camera_ = true;

    // ---- Refine anchor list -------------------------------------------------
    // Frames that bound Refine segments and are held fixed by the solver.
    // refine_anchors_blob_ is the hidden, SAVED "refine_anchors" String_knob
    // (sorted comma-separated frames, e.g. "1,40,75,100") — free .nk
    // persistence + undo. refine_anchors_ is the in-memory mirror, AUTHORITATIVE
    // for the session once loaded/edited (set_text is async in this Nuke build,
    // so re-reading the knob on our own echo could clobber a just-added anchor —
    // same hazard the pins blob documents). refine_anchors_text_ is a read-only
    // display echoing the current anchors. Mirrors the pins_blob_ round-trip.
    const char*    refine_anchors_blob_       = nullptr;   // String_knob storage
    const char*    refine_anchors_text_       = nullptr;   // read-only display
    const char*    refine_range_              = nullptr;   // "1001-1100" Range knob (Refine Range)
    std::set<int>  refine_anchors_;
    std::string    refine_anchors_cache_;                  // last string we serialized
    bool           refine_anchors_loaded_     = false;
    bool           suppress_anchor_callback_  = false;     // block our own set_text echo

    // ---- Variable focal length / principal point solve ---------------------
    // When enabled, Track and Refine let the solver vary the focal (and
    // optionally the principal point) per frame instead of trusting the camera
    // knob — see VARIABLE_FOCAL_PLAN.md. The solver's per-frame pixel focal is
    // converted back to a Nuke focal (mm) via  focal_mm = fx_px * haperture / w
    // and stored on the animated, node-local "solved_focal" curve, which Export
    // keys onto the spawned Camera2 (and which a Camera2 can expression-link to
    // for a live preview). Both flags default OFF, so a solve with them off is
    // bit-identical to before.
    //
    // Principal-point write-back IS now implemented: the solved cx,cy (OpenCV
    // y-down pixels) are stored on the hidden animated "solved_cx"/"solved_cy"
    // curves and Export maps them to the Camera2's win_translate (lens shift).
    // The seed side also reads the input camera's win_translate (projection
    // a02/a12) so a pre-shifted source camera is respected, not flattened.
    bool   opt_focal_     = false;   // bound to "opt_focal"    Bool_knob
    bool   opt_principal_ = false;   // bound to "opt_principal" Bool_knob
    double min_fov_deg_   = 15.0;    // bound to "min_fov" — clamps the solved focal range
    double max_fov_deg_   = 160.0;   // bound to "max_fov"
    // ---- Zoom solve (Option 1: alternating focal/pose block-coordinate) ------
    // Tuning for on_solve_zoom (tracker_intrinsics.cpp): zoom_passes_ alternations
    // of {fixed-pose focal fit ; fixed-focal pose re-solve}, stopping early once
    // the largest per-frame focal change between passes is below zoom_tol_mm_.
    int    zoom_passes_   = 3;       // bound to "zoom_passes" — alternation count
    double zoom_tol_mm_   = 0.05;    // bound to "zoom_tol"    — converge if max |Δfocal| < tol (mm)
    // ---- Solved-focal curve cleanup (Option 3: temporal post-processing) -----
    // Non-destructive finishing ops on the solved_focal curve only (pose never
    // touched). focal_smooth_window_ is the odd-length centred moving-average
    // window used by on_smooth_focal.
    int    focal_smooth_window_ = 5; // bound to "focal_smooth" — smoothing window (frames)
    double solved_focal_  = 50.0;    // bound to the animated "solved_focal" curve (mm)
    double solved_cx_     = 0.0;     // bound to hidden animated "solved_cx" curve (px, OpenCV)
    double solved_cy_     = 0.0;     // bound to hidden animated "solved_cy" curve (px, OpenCV)

    // ---- 3D mask (exclude mesh triangles from the solve) -------------------
    // A per-triangle bitset the solver's RayCast consults (RayCast(...,
    // check_mask=true), already passed by Track and Refine) — a masked triangle
    // returns no hit, so feature points there are dropped from the PnP/bundle
    // problem. See MASKING_PLAN.md. mask_bits_ is the in-memory bitset (word t/32,
    // bit t%32 — identical layout to Mesh::masked_triangles, so it drops straight
    // into the AcceleratedMesh ctor). mask_blob_ persists it (hidden, SAVED hex
    // String_knob; .nk persistence + undo, like pins_blob_). mask_tri_count_ pins
    // the triangle count the mask was painted against: on a topology change the
    // indices no longer line up, so the mask is dropped (ensure_mask_sized).
    //
    // Empty mask => empty ArrayXu => behaviour identical to before (no-op fast path).
    std::vector<uint32_t> mask_bits_;                 // ceil(numTris/32) words
    int          mask_tri_count_      = 0;            // tris the mask was sized to
    const char*  mask_blob_           = nullptr;      // hidden hex String_knob
    // Cache + suppress + reload decision for the hidden "mask_blob" round-trip.
    // Replaces the former mask_blob_cache_ / suppress_mask_callback_ pair; its
    // save() writes through ScopedFlags so the suppress flag is exception-safe.
    BlobMirror   mask_mirror_         { "mask_blob" };
    bool         mask_loaded_         = false;
    uint64_t     mask_version_        = 0;             // bumped on every mask change
    float        mask_color_[4]       = {1.0f, 0.25f, 0.15f, 0.5f};  // overlay tint

    // ---- 2D occlusion mask (mask plate, input 3) ---------------------------
    // Per-frame image-space exclusion sampled from the mask input's alpha and
    // handed to Track/Refine as opts.is_masked (see tracker_mask2d.cpp). All OFF
    // by default (mode 0): no mask input read, solve identical to before.
    int    mask2d_mode_      = 0;     // 0 None, 1 Mask Alpha, 2 Mask Alpha Inverted
    double mask2d_threshold_ = 0.5;   // alpha cutoff
    bool   mask2d_show_      = false; // draw the excluded pixels in the viewer
    float  mask2d_color_[4]  = {1.0f, 0.15f, 0.85f, 0.45f};  // overlay tint (magenta)
    // One-frame overlay bitset cache; rebuilt when frame/mode/threshold/input change.
    // The input is keyed by BOTH the op pointer AND its hash(): the pointer alone
    // is unsafe because a freed upstream op can be reallocated at the same address
    // (a new Roto would then serve the old op's cached bits), while hash() changes
    // whenever the upstream content/animation changes. mask2d_cache_mtx_ guards the
    // whole cache so an off-main-thread viewer redraw can't race a rebuild.
    int         mask2d_cache_frame_ = -2147483647;
    int         mask2d_cache_mode_  = -1;
    double      mask2d_cache_thr_   = -1.0;
    const void* mask2d_cache_inptr_ = nullptr;
    uint64_t    mask2d_cache_hash_  = 0;
    int         mask2d_cache_w_     = 0;
    int         mask2d_cache_h_     = 0;
    bool        mask2d_cache_valid_ = false;
    std::vector<uint32_t> mask2d_cache_bits_;
    mutable std::mutex    mask2d_cache_mtx_;

    // ---- User (helper) tracks (2D tracks from a Tracker node) --------------
    // Each track is anchored to the mesh by ray-casting its position on the
    // Reference Frame; off-object tracks (ray miss) are dropped. The anchored set
    // (user_tracks_) is handed to Track/Refine as opts.user_tracks. Raw 2D tracks
    // (Nuke y-up px) are pulled from the named Tracker node by a Python callback
    // into the hidden user_tracks_blob (persisted + undo, like pins_blob). See
    // tracker_usertracks.cpp / USER_TRACKS_PLAN.md. All OFF until tracks are loaded.
    const char* user_track_src_   = nullptr;   // source Tracker node name
    const char* user_tracks_blob_ = nullptr;   // hidden raw-2D-tracks blob
    const char* user_tracks_text_ = nullptr;   // read-only summary
    int    user_track_ref_    = 1;             // reference frame for anchoring
    int    user_track_overscan_ = 0;           // px grow of silhouette when anchoring
    double connect_max_dist_  = 200.0;         // px cutoff for auto pin<->track pairing
    bool   track_only_user_   = false;         // Track via pin<->track PnP, not flow
    bool   refine_use_user_tracks_ = false;    // opt-in: user tracks as Refine helpers
    bool   user_tracks_show_  = false;         // viewer overlay toggle
    bool   show_numbers_      = true;          // index labels on pins + tracks
    float  user_tracks_color_[4] = {0.10f, 0.95f, 1.0f, 1.0f};  // marker (cyan)
    // Anchored-tracks cache, rebuilt when the signature (blob / ref / geo / cam /
    // img / mask) changes. user_tracks_ is what Track/Refine and the overlay use.
    UserTracks  user_tracks_;
    // Parallel to user_tracks_: the stable Tracker-node name of each anchored
    // track. Used to re-resolve pin links (linked_track_name -> linked_track index)
    // whenever the set is rebuilt, so links survive reload/reorder/drop.
    std::vector<std::string> user_track_names_;

    std::string user_tracks_sig_;
    bool        user_tracks_valid_     = false;
    int         user_tracks_total_     = 0;    // raw tracks parsed
    int         user_tracks_on_object_ = 0;    // anchored (ray hit the mesh)

    // RGB colour of the 2D-viewer wireframe overlay, bound to the "wire_color"
    // Color_knob. Default matches the original hard-coded green-cyan. Read by
    // PolychaseWireframeKnob::draw_handle every redraw (alpha stays 1.0). Saved
    // to the .nk like any other knob; purely a display preference.
    float       wire_color_[3] = {0.20f, 1.00f, 0.55f};
    // Axis-gradient toggle (bound to "wire_gradient"). Display-only, saved to .nk.
    bool        wire_gradient_ = false;

    // ---- Gizmo UI state -----------------------------------------
    // 0=Gizmo, 1=Pins, 2=Both. Under the pin-refine model the node always sits
    // in Both: both overlays draw at all times and `pin_input_active_` (the P
    // toggle) routes the mouse. Kept as a member so the existing mode-gated
    // draw/hit-test branches stay valid; it is no longer user-switchable.
    int         manipulator_mode_ = 2;     // always Both (gizmo + pins)
    double      gizmo_size_px_    = 80.0;  // on-screen handle radius in pixels

    // ---- Pin-refine routing -------------------
    // Sticky arm/disarm flag: armed = pins receive the mouse (gizmo inert);
    // disarmed = gizmo receives the mouse (pins inert). Toggled on `shift+P` keydown
    // and bound to the hidden "Pin Edit" Bool_knob so the panel mirrors it.
    // DO_NOT_WRITE on the knob keeps a saved .nk from reloading armed.
    bool        pin_input_active_ = false;

    // ---- Gizmo drag state (frozen at grab) ----------------------
    // gizmo_active_handle_: -1 none; 0 = centre (screen-plane translate);
    //   1/2/3 = X/Y/Z axis-constrained translate; 4 = dolly (depth).
    // The drag plane passes through the proxy origin (O0) facing the camera;
    // each drag intersects the cursor ray with that plane and applies the world
    // delta (hit - grab_hit) to the frozen pose's world translation.
    int                gizmo_active_handle_ = -1;
    DD::Image::Matrix4 gizmo_start_model_;   // pose model matrix at grab (Nuke)
    // Offset knob values {tx,ty,tz,dolly,rx,ry,rz} snapshotted on grab — the
    // pre-drag undo baseline so on_gizmo_release can record the whole drag as one
    // step (snap to these, then set the finals with undo live).
    double             gizmo_grab_off_[7] = {0,0,0,0,0,0,0};

    // ---- Viewer-gesture move history (our own undo, knob-backed & shared) ----
    // Nuke's undo doesn't reliably capture our viewer-driven gizmo moves, and the
    // viewer-handle Op instance is frequently a different C++ instance from the one
    // a panel button lands on. So instead of snapshotting in-memory pose state on a
    // release hook (which never reached the right instance), we TRACK THE SHARED
    // OFFSET NUMBERS: ensure_move_history() runs every redraw, reads the 7 offset
    // knobs (Translate XYZ, Dolly, Rotate XYZ — all shared across instances) and,
    // when they settle to a value different from the current history entry, appends
    // a snapshot. The whole stack + cursor lives in the shared move_hist_blob knob
    // (format "pos N\n<snap0>\n<snap1>..."). restore_move_state() writes the 7 knobs
    // back (like a typed edit) and republishes live_scene_, so Undo/Redo Move jump
    // the wireframe and the numbers to exactly where they were.
    //
    // `color`, `extra` and `db_path` carry the tracked "ordinary" knob state at the
    // time the snapshot was taken, so colour, First/Last Frame, Gizmo Size and Solve
    // Mode are all first-class undo steps in the SAME stack
    // as pose/pin moves (no separate Nuke-undo fallback). `extra` is parallel to
    // kExtraKnobNames (numeric scalar knobs, read as double). `db_path` is a
    // reserved slot, not yet populated. restore_move_state() writes them back.
    struct MoveSnapshot {
        double v[7] = {0,0,0,0,0,0,0};
        int frame = 0;
        std::string pins;
        float color[3] = {0.20f, 1.00f, 0.55f};
        std::vector<double> extra;     // parallel to kExtraKnobNames
        std::string db_path;           // tracked Database path
    };
    const char* move_hist_blob_ = nullptr;   // String_knob storage (shared mirror)
    // In-memory authoritative copy. A knob's bound member does NOT refresh
    // synchronously after set_text() within the same instance, so reading the blob
    // back every redraw returned a stale stack and consecutive moves never
    // accumulated. We therefore keep the stack + cursor in memory (the source of
    // truth this session) and only parse the blob once, on a fresh instance.
    std::vector<MoveSnapshot> move_hist_;
    int         move_pos_        = -1;
    std::string move_hist_cache_;            // last string we serialized to the blob
    bool        move_hist_loaded_ = false;
    // Capture is armed only once the user actually grabs the gizmo (or a pin) this
    // session. This stops a reload — where the transient offset knobs come back as
    // zero while the saved history still holds the last pose — from auto-appending a
    // junk entry or overwriting the saved tip. The history just sits ready for
    // Undo/Redo until a real grab resumes recording.
    bool        move_hist_armed_ = false;
    int         move_nav_ = 0;               // Python hotkey hook: -1 undo, +1 redo
    bool read_offset_values(double out[7]);
    void arm_and_seed_move_baseline();       // record pre-op state + arm capture (gizmo/pin grab)
    void record_move_snapshot(const char* why);  // commit current state as a distinct undo step
    void do_undo_move();                     // step the cursor back + restore
    void do_redo_move();                     // step the cursor forward + restore
    void clear_move_history();               // wipe the stack (keeps current state)
    void refresh_overlay();                  // discard live edit -> keyed/zero pose

    // ---- Generic "ordinary knob" tracking in the move history --------------
    // These scalar knobs join pose/pins in the single undo stack so Ctrl+Z steps
    // them too. `committed_` mirrors the tracked-extra state (colour + extra +
    // db_path) of the CURRENT history cursor — it's the "before" used to seed a
    // baseline for the first edit, and is kept in step centrally by
    // store_move_history(). `last_tracked_knob_` is the knob whose edit produced
    // the cursor entry, so repeated edits of the SAME knob (slider/picker drags)
    // coalesce into one undo step while distinct knobs each get their own.
    // `suppress_tracked_callback_` blocks the re-entrant knob_changed that
    // restore_move_state()'s own set_value/set_text fire during an undo/redo.
    MoveSnapshot committed_;
    std::string  last_tracked_knob_;
    bool         suppress_tracked_callback_ = false;
    // Fill s.color / s.extra / s.db_path from the live knobs (everything except the
    // pose offsets, frame and pins, which the existing build sites fill).
    void capture_tracked_extras(MoveSnapshot& s) const;
    // True if two snapshots carry the same tracked-extra state (colour+extra+db).
    bool tracked_extras_equal(const MoveSnapshot& a, const MoveSnapshot& b) const;
    // Seed committed_ from the live knobs (on panel open, before any edit).
    void prime_committed_from_live();
    // Record an ordinary-knob edit (colour / First-Last Frame / Gizmo Size
    // / Solve Mode / Database) into the move history (old -> new),
    // coalescing consecutive edits of the SAME knob into one undo step.
    void on_tracked_knob_changed(const char* which);
    void restore_move_state(const MoveSnapshot& to, const MoveSnapshot* from = nullptr);
    void load_move_history(std::vector<MoveSnapshot>& hist, int& pos);
    void store_move_history(const std::vector<MoveSnapshot>& hist, int pos);
    DD::Image::Vector3 gizmo_plane_origin_;  // O0 — proxy origin in world at grab
    DD::Image::Vector3 gizmo_plane_normal_;  // drag-plane normal (camera fwd at grab)
    DD::Image::Vector3 gizmo_grab_hit_;      // world hit under the cursor at grab
    // Axis-constrained translate (handles 1/2/3):
    DD::Image::Vector3 gizmo_axis_dir_;          // unit world axis being dragged
    float              gizmo_screen_dir_[2] = {1.0f, 0.0f};  // unit screen dir (GL)
    float              gizmo_px_per_world_  = 1.0f;          // screen px per world unit
    float              gizmo_grab_mx_ = 0.0f;    // grab cursor (GL screen px)
    float              gizmo_grab_my_ = 0.0f;
    // Dolly (handle 4):
    DD::Image::Vector3 gizmo_cam_center_;    // camera world position (frozen)
    DD::Image::Vector3 gizmo_dolly_v_;       // O0 - cam_center (frozen)

    // Numerical rotation about the mesh centre. Each holds an absolute offset
    // (degrees) layered on the upstream base; editing one previews the pose live
    // (no key) and the fields persist their values (Set Pose Key bakes them).
    double             rot_nudge_x_ = 0.0;
    double             rot_nudge_y_ = 0.0;
    double             rot_nudge_z_ = 0.0;
    // Set while reset_rot_offsets() writes the rot_* knobs back to 0, so the
    // re-entrant knob_changed those set_value() calls trigger is a no-op rather
    // than a spurious preview recompute.
    bool               suppress_rot_callback_ = false;
    // Frozen base for the in-progress rotation offset. Captured on the first
    // nonzero rot_* edit = the current working pose (an uncommitted gizmo
    // translate/dolly in live_scene_ if present, else the keyed/upstream pose)
    // WITHOUT the offset, so re-edits stay absolute and rotate-after-translate
    // composes. Cleared when the offset returns to 0, on commit, on a gizmo grab
    // (which bakes the rotation into the pose), and on Clear Solve.
    // rot_base_had_live_ records whether live_scene_ was set at freeze time so a
    // return-to-0 reverts to the right pose (the translate, or nothing).
    std::optional<DD::Image::Matrix4> rot_base_;
    bool                              rot_base_had_live_ = false;

    // Numerical translate / dolly — the analogue of the rotation offsets above,
    // bound to the trans_x/y/z + dolly knobs. trans_* are a WORLD-space offset;
    // dolly is a signed distance along trans_base_dolly_dir_ (the camera->mesh
    // viewing ray, frozen with the base). A gizmo translate/dolly drag writes
    // these on RELEASE (one undo per gesture) and they can also be typed.
    double             trans_off_x_ = 0.0;
    double             trans_off_y_ = 0.0;
    double             trans_off_z_ = 0.0;
    double             dolly_off_   = 0.0;   // signed world units along the dolly dir
    // Suppresses the re-entrant knob_changed while reset_trans_offsets()/the
    // gizmo-release commit write the trans knobs (mirrors suppress_rot_callback_).
    bool               suppress_trans_callback_ = false;
    // Frozen base for the in-progress translate/dolly offset (the working pose
    // WITHOUT the offset), plus the viewing-ray direction the dolly slides along.
    // Captured on a gizmo grab and on the first nonzero typed trans/dolly edit;
    // dropped when the offset returns to 0, on commit (Set Pose Key), on the next
    // grab, and on Clear Solve. trans_base_had_live_ records whether live_scene_
    // was set at freeze time so a return-to-0 reverts to the right pose.
    std::optional<DD::Image::Matrix4> trans_base_;
    bool                              trans_base_had_live_ = false;
    DD::Image::Vector3                trans_base_dolly_dir_ = DD::Image::Vector3(0, 0, 1);

    // Hidden serialized blobs — filled by Track in later phases.
    const char* keys_blob_     = nullptr;
    const char* anchor_node_   = nullptr;

    // ---- Transient ----------------------------------------------------------
    const char* status_text_   = nullptr;

    // ---- Pin storage + persistence -----------------------------
    // Each pin anchors a screen position to a mesh vertex (vertex_idx). The
    // pins_ vector is the working copy; pins_blob_ (a hidden String_knob
    // storage) is the persistent + undoable source of truth. They're kept
    // in sync via sync_blob_from_pins() (writes pins_ → blob) and
    // on_pins_blob_changed() (reads blob → pins_, called on load + undo).
    std::vector<Pin> pins_;
    unsigned         next_pin_id_              = 0;
    const char*      pins_blob_                = nullptr;  // String_knob storage
    const char*      pin_list_text_            = nullptr;  // Multiline display
    bool             pins_loaded_from_blob_    = false;
    bool             suppress_pin_blob_callback_ = false;
    // Last pins_blob string we deserialized. ensure_pins_loaded() reloads
    // whenever the live blob differs from this — that's what makes .nk load
    // work even though the one-shot loaded flag may be set early.
    std::string      pins_blob_cache_;

    // Once we have a real in-memory pin list (loaded from a non-empty blob, or
    // edited via place/drag/delete), pins_ is AUTHORITATIVE for the rest of the
    // session — the hidden knob is just a save/undo mirror. set_text/knob_changed
    // are async in this Nuke build, so a stale blob echo could otherwise be
    // re-read by ensure_pins_loaded and clobber a just-placed pin (DIAG proved
    // this: ensure reloaded 10 pins over the fresh 11). This flag makes the
    // polling reload defer to memory; genuine undo/.nk changes still come through
    // on_pins_blob_changed, which reloads only when the blob's CONTENT differs
    // from what we last wrote (so our own async echo is ignored).
    bool             pins_in_memory_authoritative_ = false;

    // ---- Live pin-mode solve state ----
    //
    // live_scene_ holds the current solved SceneTransformations. nullopt means
    // "no solve has run yet; use upstream geo transform directly". On every
    // successful drag step we run FindTransformation and store the result
    // here. The wireframe knob queries effective_model_matrix() so that the
    // overlay reflects the solved pose even though the upstream GeoOp hasn't
    // moved (yet — TransformGeo auto-insert + keyframing comes in 1C.4).
    //
    // Architecture choice: persist live_scene_ across drag events (and across
    // pin placements) but RESET it on:
    //   - external pins_blob changes (undo/redo, .nk load) — they imply a
    //     different constraint set, so the previous solve no longer applies
    //   - Clear Solve button
    //   - Clear All Pins button / Delete Selected Pin button
    //
    // We do NOT serialize live_scene_ — opening a .nk starts fresh; the user
    // re-drags to rebuild. Per-frame keyframes are the proper persistence
    // path and will be added with the TransformGeo writeback.
    // live_scene_ holds the solved object pose. It's mutable because the const
    // accessor effective_model_matrix() may read it; it's the transient,
    // in-progress pose during a drag.
    mutable std::optional<SceneTransformations> live_scene_;

    // Frame at which live_scene_ was staged. effective_model_matrix lets
    // live_scene_ win ONLY at this frame, so an uncommitted edit previews at the
    // frame being manipulated but never freezes the overlay across the whole
    // timeline. Set wherever live_scene_ is assigned; ignored when it's null.
    mutable int live_edit_frame_ = -1000000;

    // Undoable mirror of the gizmo's live pose. A gizmo translate/dolly drag only
    // mutates live_scene_ (plain C++ state that Nuke's undo cannot see), so on its
    // own a gizmo move is NOT undoable — unlike the rotate knobs, whose knob value
    // is the undoable source. This hidden String_knob fixes that: on each gizmo
    // RELEASE we serialize live_scene_'s model_matrix (+ the edit frame) into it,
    // Nuke records the change, and on Ctrl+Z/Ctrl+Y on_live_pose_blob_changed()
    // restores live_scene_ from it. It is written ONLY by the gizmo path and the
    // reset paths (Clear Solve / Track / Clear Pose Keys), so pin-drag and Set
    // Pose Key undo behaviour are untouched. Mirrors the pins_blob round-trip.
    const char* live_pose_blob_              = nullptr;   // String_knob storage
    std::string live_pose_blob_cache_;
    bool        suppress_live_pose_callback_ = false;
    bool        gizmo_moved_                 = false;      // a gizmo drag moved the pose
    void sync_blob_from_live_pose();    // live_scene_ -> live_pose_blob (captures undo)
    void on_live_pose_blob_changed();   // live_pose_blob -> live_scene_ (undo/redo/load)

    // UI/playhead frame pushed in by the overlay knob (uiContext().frame()) on
    // every draw_handle. Interaction code reads editing_frame() instead of
    // outputContext().frame() so place/grab/solve/key all happen on the frame
    // actually being displayed, not the stale Op context. has_editing_frame_
    // stays false until the knob has drawn at least once (headless / no viewer),
    // in which case editing_frame() falls back to the rounded outputContext.
    int  editing_frame_     = 0;
    bool has_editing_frame_ = false;

    // Persisted solved pose, stored as animated translate / rotate / scale on
    // XYZ_knobs. Animation curves are always written to the .nk, so the
    // position survives save/load and each solved frame is a keyframe on the
    // timeline (Dope Sheet / Curve Editor). The compose/decompose pair is
    // exact, so round-tripping the matrix through T/R/S loses nothing.
    double pose_t_[3] = {0, 0, 0};
    double pose_r_[3] = {0, 0, 0};   // degrees, XYZ order (R = Rx*Ry*Rz)
    double pose_s_[3] = {1, 1, 1};

    // Live Camera Solve — the object pose re-expressed as a moving camera (geo at
    // rest), kept as animated curves so a Camera2 expression-linked to them
    // previews the matchmove live. Refreshed on Track, on each pin/gizmo commit,
    // and via the Refresh Live Camera button. Same math as Export's Camera mode.
    double live_cam_t_[3] = {0, 0, 0};   // translate
    double live_cam_r_[3] = {0, 0, 0};   // rotate, degrees, ZXY (matches Export)
    // Cache of the seed-frame camera-to-world so bake/update don't re-sample the
    // camera (a setOutputContext + double validate) more than necessary.
    bool             live_cam_seed_valid_     = false;
    int              live_cam_seed_cached_    = 0;
    RowMajorMatrix4f live_cam_world_seed_cached_;

    DD::Image::Matrix4 pose_matrix_to_nuke(double frame) const;   // T/R/S knobs @ frame -> Matrix4
    bool  has_pose_keys() const;                      // any animation on the pose knobs?
    void  key_pose_at_current_frame();                // write live pose as a key
    void  clear_pose_keys();                          // remove all pose animation

    // Write an arbitrary object-world matrix as a T/R/S key at
    // `frame`. Generalizes key_pose_at_current_frame for the per-frame Track
    // loop (decompose_trs + set_value_at on each pose channel). Defined in
    // tracker_track.cpp. Returns false WITHOUT writing if `model_world` is
    // non-finite (NaN/Inf) — a single divergent PnP frame must not be baked onto
    // the curve, since every later frame reads it back through
    // effective_model_matrix and would cull the overlay (same invariant the
    // interactive commit paths enforce).
    bool  key_pose_matrix_at(double frame, const RowMajorMatrix4f& model_world);

    // ---- Solved focal curve (variable focal length) ------------------------
    // key_focal_at writes one keyframe (mm) on the animated "solved_focal" knob;
    // has_solved_focal reports whether that curve carries any animation (so
    // Export / Refine seeding can tell a solved-focal clip from a fixed one).
    // Both defined in tracker_track.cpp.
    void  key_focal_at(double frame, double focal_mm);
    bool  has_solved_focal() const;

    // ---- Solved principal point curves (variable principal point) ----------
    // key_principal_at writes one keyframe to the hidden animated "solved_cx" and
    // "solved_cy" curves (cx,cy in OpenCV/y-down pixels, as the solver returns
    // them); has_solved_principal reports whether they carry animation. Export
    // maps these to the Camera2's win_translate. Defined in tracker_track.cpp.
    void  key_principal_at(double frame, double cx_px, double cy_px);
    bool  has_solved_principal() const;

    // ---- 3D mask helpers (tracker_mask.cpp) --------------------------------
    // Bit ops index by triangle order (the same order extract_first_object_mesh
    // emits and the AcceleratedMesh is built in). build_mask_array reconciles the
    // stored mask against the live mesh and returns the ArrayXu to hand the mesh
    // ctor (empty when nothing is masked — the no-op fast path). Triangles are
    // masked/unmasked via the 3D-viewer vertex selection (apply_3d_vertex_selection).
    void  mask_triangle(uint32_t tri);
    void  unmask_triangle(uint32_t tri);
    void  clear_mask();
    void  load_mask_from_knob();
    void  save_mask_to_knob();
    void  ensure_mask_sized(uint32_t num_triangles);   // size to ceil/32; drop on topology change
    ArrayXu build_mask_array(uint32_t num_triangles);  // -> ctor arg (empty if unmasked)

    // ---- 2D occlusion mask helpers (tracker_mask2d.cpp) --------------------
    // sample_mask2d_frame reads the mask input's alpha at `frame` into a packed
    // Y-up (Nuke) bitset (set bit = excluded), applying the mode + threshold.
    // make_mask2d_predicate pre-renders [from,to] and returns the MaskPredicate
    // handed to TrackerOptions/RefinerOptions::is_masked (empty when off).
    bool sample_mask2d_frame(int frame, int& w, int& h,
                             std::vector<uint32_t>& bits) const;
    MaskPredicate make_mask2d_predicate(int from, int to);

    // ---- User (helper) track helpers (tracker_usertracks.cpp) --------------
    void on_load_user_tracks();        // button: pull tracks from the Tracker node
    void clear_user_tracks();          // button: forget loaded tracks
    void rebuild_user_tracks();        // parse blob -> anchor via RayCast -> user_tracks_
    void reresolve_pin_links();        // re-bind pin linked_track_name -> current index
    void ensure_user_tracks_built();   // rebuild when the input signature changes
    std::string user_tracks_signature() const;
    void refresh_user_tracks_label();  // echo counts in the read-only summary

    // Connect (Pin -> Track linking, v1 auto-nearest). On the Reference Frame:
    // pair each user track to its nearest UN-taken pin in 2D screen pixels (skip
    // a track whose nearest pin is beyond connect_max_dist_), set those pins'
    // targets to the track pixels, run the rigid pin solve, then Set Pose Key.
    void connect_pins_to_tracks();

    // Clean redo: clear ALL pin targets + bindings, then run a fresh Connect, so
    // stale targets on now-unmatched pins can't fight the new solve.
    void reconnect_pins_to_tracks();


    // Track via pin<->track bindings (the "Use Only User Tracks" mode). Pair pins
    // to tracks ONCE at the ref frame (auto-nearest 2D), then for every frame in
    // the direction solve the exact rigid PnP from the bound mesh VERTICES (not
    // raycast anchors) and their per-frame track pixels, and key the pose. With
    // pins on the exact reconciled corners this is an exact solve.
    //
    // seed_solved_intrinsics: when true AND a solved_focal (and/or solved
    // principal) curve exists, each frame's PnP intrinsics are seeded from those
    // curves and held fixed (the PnP never varies the lens), so the pose solves
    // UNDER the solved per-frame focal instead of the camera's base lens. This is
    // how Solve Zoom's pose re-solve step couples to the focal fit in user-track-
    // only mode (where there is no flow DB for on_refine). Default false keeps the
    // plain Track-via-pins behaviour (camera base lens) unchanged.
    // Returns true if at least one frame was keyed (false on the early-out failure
    // paths: missing inputs / no mesh / fewer than 3 eligible pins).
    bool track_via_pins(bool forward, bool seed_solved_intrinsics = false);

    // ---- Track / Apply state -------------------------------------
    // tracking_cancel_ is checked by the synchronous Track callback (Stop sets
    // it). Note: a synchronous track blocks the UI thread, so mid-run cancel is
    // only effective once the threaded path lands; wired now for completeness.
    bool tracking_cancel_ = false;
    // Frame the active solve was seeded from. Persisted in a hidden Int_knob so
    // Export can reconstruct the seed pose/camera after a reload.
    int  track_seed_frame_ = 0;

    // drag_anchor_ is the "initial" pose FROZEN for the duration of one drag.
    // It defines the model_view the pins are projected through and the "stay"
    // targets for the un-dragged pins, so it must NOT move while a drag is in
    // progress (re-reading it from live_scene_ each tick would make the stay
    // pins chase the moving pose and the solve would drift). Captured on the
    // first solve tick of a drag, cleared on release / reset.
    std::optional<SceneTransformations> drag_anchor_;

    void reset_live_solve();          // wipe solver state (and viewer redraws)
    void run_pin_solve(int pin_idx);  // called from on_mouse_drag

    // ---- Pin-refine rigid solve -----------
    // No-mover re-solve: fit the rigid pose from ALL current pins as equal-
    // weight anchors (dragged_row = -1), warm-started from the current pose,
    // then key the result. Used by delete_pin_at — a removed pin is a removed
    // constraint, not a reason to discard the pose.
    void resolve_pins_no_mover();

    // Permanent low-weight synthetic prior: >= 3 well-spread, non-collinear
    // mesh vertices, projected through the per-tick warm-start pose at a small
    // weight. They satisfy SolvePnPIterative's CHECK_GE(rows,3), hold the DOFs
    // that 1-2 real pins don't, and break the near-collinear-pin roll
    // degeneracy (which is why the old 35° lurch guard is gone). Cached;
    // recomputed only when the geo's vertex count changes.
    std::vector<unsigned> synth_anchor_verts_;
    Eigen::Index          synth_geo_vert_count_ = -1;
    void ensure_synth_anchors(const GeoMesh& gm);   // (re)fill synth_anchor_verts_

    // ---- Drag state + UI ----
    //
    // dragging_pin_idx_ tracks the index into pins_ currently being dragged.
    // -1 means no drag in progress. Set on PUSH that hits a pin, cleared on
    // RELEASE. The DRAG handler reads it to know which pin's target to update.
    //
    // pins_to_remove_ is the free-text field bound to the "Pins to remove"
    // String_knob: 1-based list positions (any separator) that the "Remove Pins"
    // button deletes. String_knob storage, so a const char* like the other blobs.
    int              dragging_pin_idx_         = -1;
    // True once the in-progress pin gesture has actually MOVED a pin (a drag tick
    // fired), as opposed to a bare click. Set false on PUSH, true on DRAG; RELEASE
    // only re-fits the body (resolve_pins_no_mover) when it's true, so a plain
    // click on a pin never keys a pose.
    bool             pin_drag_moved_           = false;
    // Wall-clock time of the last pin-drag motion tick. Purely a visual cue: a pin
    // is drawn red only while it is *actively* being moved (a drag tick landed in
    // the last ~250 ms) and amber once it comes to rest. Never touches
    // dragging_pin_idx_, so drag/commit/undo logic is unaffected.
    std::chrono::steady_clock::time_point last_pin_drag_tp_{};
    const char*      pins_to_remove_           = nullptr;

    // Pin persistence/undo plumbing
    void sync_blob_from_pins();        // pins_ → pins_blob_ knob (captures undo)
    void restore_pins_from_blob(const std::string& s);  // move-history undo: set pins_ from a snapshot blob
    void on_pins_blob_changed();       // pins_blob_ → pins_ (load + undo restore)
    void refresh_pin_ui();             // updates multiline display

    void ensure_pins_loaded();         // lazy-load pins_ from blob on first use

    // Click → place new pin (called from on_mouse_push when click isn't on
    // an existing pin). Returns true if a pin was placed.
    int  try_place_new_pin(DD::Image::ViewerContext* ctx,
                           const double mv[16], const double pj[16],
                           const int vp[4]);

    // Hit-test: given a mouse position (Nuke top-down convention), find the
    // index of the closest pin within hit radius. Returns -1 if none.
    int find_pin_under_cursor(int mouse_x_nuke, int mouse_y_nuke_topdown,
                              const double mv[16], const double pj[16],
                              const int vp[4]);

    // ---- Methods ------------------------------------------------------------
    void on_track(const char* direction);
    void on_stop();
    void on_clear_solve();
    void on_export();

    // ---- Background intrinsics solve (fixed-pose, non-destructive) ----------
    // Fit focal / principal point against the EXISTING tracked+refined pose
    // (R,t frozen), writing ONLY the solved_focal / solved_cx / solved_cy curves
    // — the pose is never re-solved. Implemented in tracker_intrinsics.cpp. The
    // two buttons are independent (one never clears the other's curve); the
    // "Export Solved …" checkboxes (opt_focal_/opt_principal_) only gate Export.
    void on_solve_focal();
    void on_solve_principal();
    bool solve_intrinsics_impl(bool do_focal, bool do_principal);

    // ---- Zoom solve (Option 1: alternating / block-coordinate) -------------
    // TRUE varying-focal solve for genuine optical zooms. Alternates the two
    // proven passes — solve_intrinsics_impl(do_focal=true) (fixed-pose focal
    // fit) and on_refine() (fixed-focal pose re-solve via its have_focal_curve
    // seeding path) — until the solved_focal curve converges. UNLIKE the two
    // buttons above this RE-BAKES the interior poses of the Refine Range (range
    // ends + anchors held), so it is NOT non-destructive — it is a deliberate,
    // opt-in tool, never wired to the default Solve Focal button. Implemented in
    // tracker_intrinsics.cpp. See ZOOM_SOLVE_IMPLEMENTATION.md (Option 1).
    void on_solve_zoom();

    // ---- Focal Sweep: a from-scratch, per-frame focal solver --------------
    // Unlike on_solve_zoom (which alternates and gets trapped in the constant-
    // focal local minimum, because fitting focal to a pose that was tracked under
    // the WRONG focal just confirms it), this does a GLOBAL focal search at each
    // frame: for every candidate focal it re-solves the pose fresh (PnP from the
    // pins/tracks) and measures reprojection error, then keeps the focal with the
    // lowest error. The rigid object's foreshortening under parallax makes the
    // error-vs-focal curve dip sharply at the true focal, so it recovers a VARYING
    // lens without any pose anchors or hand alignment. Needs >=4 pins with a key or
    // linked track on a frame. Writes ONLY the solved_focal curve. Per-frame log
    // line [fsweep] shows best focal, its RMS, and the dip sharpness (confidence;
    // a flat dip = degenerate/low-parallax frame). Implemented in tracker_pins.cpp.
    void on_solve_focal_sweep();

    // ---- Focal P4Pf: from-scratch per-frame focal via PoseLib -------------
    // The robust answer to varying-focal recovery. Solves the full pose AND focal
    // FROM SCRATCH at every frame with PoseLib's P4Pf minimal solver inside a small
    // LO-RANSAC — no warm start (so no lag), and it never reads Nuke's stale cooked
    // camera. Correspondences come from the USER TRACKS (per-frame 2D + anchored 3D).
    // Writes ONLY the Solved Focal curve (mm/frame); pair with 'Smooth Solved Focal'.
    // Implemented in tracker_focal_pnpf.cpp.
    void on_solve_focal_pnpf();

    // ---- Zoom tab: guarded entry point (ZOOM_TAB_PLAN.md, step 2a) ----------
    // on_solve_zoom_from_pins() validates the pin set for a trustworthy varying-
    // focal solve, then delegates to on_solve_zoom() (the proven alternating
    // focal/pose solver). The guards are REFUSALS with a clear status, not
    // placement limits:
    //   - >= 2 refine anchors (need at least two zoom frames to interpolate focal),
    //   - >= 4 pins,
    //   - the pinned vertices are NON-COPLANAR (must straddle the object's depth;
    //     4 points on one face cannot separate zoom from distance — that is the
    //     focal/depth ambiguity, not a solver weakness).
    // Step 2a drives off the EXISTING pin list; 2b/2c add the separate zoom-pin
    // store and the armed Zoom Pin Edit overlay. Implemented in tracker_intrinsics.cpp.
    void on_solve_zoom_from_pins();

    // Coplanarity test for the guard: returns true if the given local-space points
    // are (near) coplanar — i.e. their best-fit plane explains them to within
    // `tol_frac` of the point-set extent. Used to refuse a zoom solve on a flat
    // pin layout. Defined in tracker_intrinsics.cpp.
    static bool points_are_coplanar(const std::vector<Eigen::Vector3f>& pts,
                                    float tol_frac = 0.02f);

    // ---- Solved-focal curve cleanup (Option 3) -----------------------------
    // Pure temporal post-processing of the solved_focal curve — NON-destructive
    // (only that curve is edited; pose is never touched). Not a zoom solver, a
    // cleaner estimate of what is already there. Good as a finishing step after
    // Solve Focal or Solve Zoom. Implemented in tracker_intrinsics.cpp.
    //   on_flatten_focal() — replace the curve with its mean over First..Last
    //       (the honest output for a constant lens).
    //   on_smooth_focal()  — centred moving-average (focal_smooth_window_ frames)
    //       to tame per-frame jitter while keeping the trend (slow drift).
    void on_flatten_focal();
    void on_smooth_focal();

    // on_copy_focal_to_camera() — write the solved_focal curve onto the CONNECTED
    // input camera's 'focal' knob (animated) over [first_frame_, last_frame_]. This
    // MUTATES the input camera (unlike Export, which spawns a Camera2), so after it
    // the camera itself carries the zoom and the overlay/export read the right lens
    // straight from the camera — no Preview Solved Lens needed. Undoable (knob writes
    // go on the undo stack). No-op without a solved focal or a connected camera.
    void on_copy_focal_to_camera();

    // ---- Refine (global bundle adjustment between anchors) ------------------
    // Port of Polychase's Refine Sequence (cpp/refiner.cc::RefineTrajectory),
    // implemented in tracker_refine.cpp. After a first Track, the artist
    // hand-corrects a few frames (pin/gizmo + Set Pose Key); each correction is
    // auto-recorded as an ANCHOR. Refine re-solves the frames *between*
    // consecutive anchors against the same optical-flow DB Track used, holding
    // the anchors fixed as ground truth. The convention bridge is Track's,
    // inverted:  view_cv(t) = view_cv_seed * model'(t) * model0^-1  to fill the
    // trajectory, and  model'(t) = view_cv_seed^-1 * view_cv'(t) * model0  to
    // bake the refined interior frames back onto the pose_* curves.
    //
    //   on_refine()                  — button entry ("Refine Range"). Reads the
    //       "refine_range" knob (e.g. "1001-1100") and refines every gap INSIDE
    //       that range, holding the range ends AND any interior anchors fixed.
    //   build_segments_in_range(a,b) — boundary set = {a,b} ∪ interior anchors;
    //       -> consecutive [A,B] pairs with an interior frame to solve (>2).
    //   parse_refine_range(from,to)  — parse the "1001-1100"-style knob (any
    //       non-digit separates the two numbers); falls back to
    //       [first_frame_, last_frame_] when empty/garbage and writes that back.
    void on_refine();
    std::vector<std::pair<int, int>> build_segments_in_range(int from, int to) const;
    bool parse_refine_range(int& from, int& to);

    // Anchor list. The frames that bound Refine segments and are held fixed.
    // Stored in the hidden, saved "refine_anchors" String_knob (free .nk
    // persistence + undo) and mirrored in-memory in refine_anchors_. Recorded
    // automatically by key_pose_at_current_frame (a hand correction = a
    // boundary, matching the Blender KEYFRAME mental model).
    bool add_refine_anchor(int frame);       // insert + persist; true if newly added
    void clear_refine_anchors();             // button: wipe + status + repaint
    void reset_refine_anchors_silent();      // wipe + persist, NO status (Track / Clear Pose Keys)
    void load_anchors_from_knob();           // refine_anchors knob -> refine_anchors_
    void save_anchors_to_knob();             // refine_anchors_ -> knob (+ label)
    void refresh_anchor_label();             // echo current anchors in the read-only display

    // Live Camera Solve. live_camera_basis() fills the geo-rest matrix G and the
    // seed-frame camera-to-world (cached by seed frame). bake_live_camera()
    // rewrites the whole-range curves from the keyed pose; update_live_camera_key_current()
    // refreshes just the current frame's key (called on each pin/gizmo commit).
    bool live_camera_basis(RowMajorMatrix4f& G, RowMajorMatrix4f& cam_world_seed);
    void bake_live_camera();
    void update_live_camera_key_current();

    void set_status(const std::string& msg);
    static std::string timestamp();

    // Input-resolution helpers — read the connected Op pointer for each pin
    // and return as the typed pointer (or nullptr if not connected / wrong type).
    Iop*      input_img() const { return dynamic_cast<Iop*>     (Op::input(kInputImg)); }
    CameraOp* input_cam() const { return dynamic_cast<CameraOp*>(Op::input(kInputCam)); }
    GeoOp*    input_geo() const { return dynamic_cast<GeoOp*>   (Op::input(kInputGeo)); }
    // Optional 2D occlusion-mask input (input 3): a Roto / any Iop with alpha.
    Iop*      input_mask() const { return dynamic_cast<Iop*>    (Op::input(kInputMask)); }
    // Raw geo input as Op* — pass to extract_mesh(), which dispatches new (GeomOp,
    // via geomOp()) vs classic (GeoOp). Keep input_geo() for classic-only call sites
    // until they're migrated.
    Op*       input_geo_op() const { return Op::input(kInputGeo); }

#ifdef PCN_NEW_3D
    // ---- Publish the input geometry THROUGH this node (GeometryProviderI) -------
    // Viewing through PolychaseTracker shows the real input mesh AND lets the
    // viewer sub-object-select it. The minimal "return the input's provider" form
    // makes the geo VISIBLE, but the GeoCube stays the provider's owner, so the
    // viewer won't sub-object-select it *through* this node. The fix (per
    // GeometryProviderI.h) is the ATTACH form: this Op IS the provider
    // (asGeometryProvider/getGeometryProviderOp -> this, so we own the selection
    // anchor) and forwards the geometry-producing methods up to input 2's provider.
    // Same idea as ScanlineRender2 (an Iop exposing its geometry input). Pass-
    // through shows the input at its OWN transform; the solved pose stays in the
    // handle overlay.
    DD::Image::GeometryProviderI* geometryProvider() override { return this; }

    // The provider that actually generates the geometry (our geo input), if any.
    DD::Image::GeometryProviderI* geo_provider_in() const {
        DD::Image::Op* g = Op::input(kInputGeo);
        return g ? g->geometryProvider() : nullptr;
    }

    // --- GeometryProviderI: identity (we own the published geometry) ---
    DD::Image::GeometryProviderI* asGeometryProvider()  override { return this; }
    DD::Image::Op*                getGeometryProviderOp() override { return this; }

    // --- GeometryProviderI: state hashes / anim hint (forward the input's) ---
    // Nuke 17.1: these are now context-aware pure virtuals; the old no-arg
    // signatures still exist on the base but only as non-virtual convenience
    // wrappers that build a default context and call these.
    fdk::Hash geometryComposeState(const ndk::NodeEvalContext& engineGraphContext) override {
        DD::Image::GeometryProviderI* p = geo_provider_in();
        return p ? p->geometryComposeState(engineGraphContext) : fdk::Hash();
    }
    fdk::Hash geometryEditVersionState(const ndk::NodeEvalContext& engineGraphContext) override {
        DD::Image::GeometryProviderI* p = geo_provider_in();
        return p ? p->geometryEditVersionState(engineGraphContext) : fdk::Hash();
    }
    bool geometryStateVaries(const ndk::NodeEvalContext& engineGraphContext) override {
        DD::Image::GeometryProviderI* p = geo_provider_in();
        return p ? p->geometryStateVaries(engineGraphContext) : false;
    }

    // --- GeometryProviderI: layer output (forward) ---
    // Nuke 17.1: buildGeometryLayer(bool, TimeValueSet) is now a non-virtual
    // convenience on the base that calls this pure virtual with the Op's
    // own default context.
    usg::LayerRef buildGeometryLayerForContext(
        const ndk::NodeEvalContext& engineGraphContext,
        bool appendTo = false,
        const fdk::TimeValueSet& sampleTimes = fdk::TimeValueSet()) override {
        DD::Image::GeometryProviderI* p = geo_provider_in();
        return p ? p->buildGeometryLayerForContext(engineGraphContext, appendTo, sampleTimes)
                 : usg::LayerRef();
    }

    // --- GeometryProviderI: stage output (forward) ---
    bool canProvideGeometryStage() const override {
        DD::Image::GeometryProviderI* p =
            const_cast<PolychaseTracker*>(this)->geo_provider_in();
        return p ? p->canProvideGeometryStage() : false;
    }
    // Nuke 17.1: buildGeometryStage(...) (no context) is now a non-virtual
    // convenience; override the *ForContext virtual instead.
    void buildGeometryStageForContext(usg::StageRef& stage,
                                       const usg::ArgSet& requestArgs,
                                       const ndk::NodeEvalContext& engineGraphContext,
                                       const fdk::TimeValueSet& sampleTimes = fdk::TimeValueSet()) override {
        DD::Image::GeometryProviderI* p = geo_provider_in();
        if (p) p->buildGeometryStageForContext(stage, requestArgs, engineGraphContext, sampleTimes);
        else   stage.reset();
    }
#endif


    static constexpr const char* kNodeHelp =
        "Polychase mesh-based motion tracker (NDK port).\n"
        "\n"
        "Inputs:\n"
        "  img — Read/plate node providing footage frames\n"
        "  cam — Camera node providing intrinsics (focal, aperture)\n"
        "  geo — Geo node providing the 3D mesh to track\n"
        "\n"
        "Workflow:\n"
        "  1. Connect img + cam + geo\n"
        "  2. Set Database path to the .db built by the mvflow_to_db converter\n"
        "  3. On the FIRST frame, place pins in the Viewer and key the pose\n"
        "  4. Click TrackIt (solves First->Last from that pose, any playhead)\n"
        "  5. Pick Solve Mode (Camera or Model), click Export to spawn\n"
        "     an animated Camera2 or TransformGeo with the tracked motion.";
};

} // namespace pcn

#endif // POLYCHASE_TRACKER_H