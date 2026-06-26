// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Peter Mercell
//
// Developed with assistance from Claude (Anthropic).

// PolychaseTracker.cpp — part of the PolychaseTracker plugin (see polychase_tracker.h).
#include "polychase_tracker.h"

#include "DDImage/GeoSelectKnobI.h"
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <cstdio>
#include <cstring>
#include <cstdlib>

using namespace DD::Image;

namespace pcn {

// Solve-mode enumeration labels (used by knobs()).
const char* const kSolveModes[] = { "Camera", "Model", nullptr };

// 2D occlusion-mask mode labels (used by knobs()). "Mask Alpha" excludes pixels
// where alpha >= threshold (the roto IS the occluder); "Mask Alpha Inverted"
// excludes pixels where alpha < threshold (track ONLY inside the roto).
const char* const kMask2DModes[] = { "None", "Mask Alpha", "Mask Alpha Inverted", nullptr };


PolychaseTracker::PolychaseTracker(Node* node)
    : NoIop(node)
{}


// -----------------------------------------------------------------------------
// Knob declaration. Section order matches artist workflow:
//   Inputs -> Database -> Track -> Export -> Status
// -----------------------------------------------------------------------------
void PolychaseTracker::knobs(Knob_Callback f)
{
    // ========================================================================
    // (No "Inputs" section needed in Properties — the input pins ARE the
    // inputs and live in the node graph, not the Properties panel.)
    // ========================================================================

    // ========================================================================
    // Frames — the range TrackIt solves over (First -> Last).
    // ========================================================================
    Divider(f, "");

    Int_knob(f, &first_frame_, "first_frame", "First Frame");
    SetFlags(f, Knob::NO_UNDO);
    Tooltip(f, "First frame of the range. TrackIt seeds here and solves forward.");

    Int_knob(f, &last_frame_, "last_frame", "Last Frame");
    ClearFlags(f, Knob::STARTLINE);
    SetFlags(f, Knob::NO_UNDO);
    Tooltip(f, "Last frame of the range, inclusive.");

    // ========================================================================
    // Database — the flow DB is built OUTSIDE Nuke (the mvflow_to_db
    // converter / VisualizeFlowDB pipeline), which gives a far cleaner field
    // than an in-node analyze pass did, so there's no Analyze button any more.
    // ========================================================================
    Divider(f, "");

    File_knob(f, &db_path_, "db_path", "Database");
    Tooltip(f, "Path to the optical-flow database. Built externally by the "
               "mvflow_to_db converter (from your motion-vector EXRs); consumed "
               "by TrackIt. Point this at that .db file.");

    // ========================================================================
    // Wireframe — appearance of the 2D-viewer mesh overlay. Display-only: this
    // affects nothing in the solve, the pins, the gizmo, or the export.
    // ========================================================================
    Divider(f, "Wireframe");

    Color_knob(f, wire_color_, "wire_color", "Color");
    SetFlags(f, Knob::NO_ANIMATION | Knob::NO_UNDO);
    Tooltip(f, "Colour of the mesh wireframe drawn over the 2D plate. A pure "
               "viewer-display preference — it doesn't touch the pins, the gizmo, "
               "the solve, or the export, and is saved with the script. Defaults "
               "to the original green-cyan. Undo/redo it with the same Undo Move / "
               "Ctrl+Z as gizmo and pin edits.");

    Bool_knob(f, &wire_gradient_, "wire_gradient", "Axis Gradient");
    SetFlags(f, Knob::NO_ANIMATION | Knob::NO_UNDO);
    ClearFlags(f, Knob::STARTLINE);   // sit beside the Color swatch
    Tooltip(f, "Colour the wireframe by each vertex's position in the mesh's LOCAL "
               "space (origin 0,0,0): +X red, +Y green, +Z blue, the origin a neutral "
               "grey. The colours are locked to the object, so they rotate with it — "
               "a strong at-a-glance cue for orientation while posing/tracking. "
               "Overrides the flat Color above while enabled. Display-only; undoable "
               "with the same Undo Move / Ctrl+Z.");

    // ========================================================================
    // Pins. The artist clicks on the plate to anchor mesh
    // vertices to image features. Tracking (1D) will use these as
    // observation constraints when running FindTransformation.
    //
    // UI layout:
    //   - Multiline read-only display showing all pins
    //   - One delete button per slot (up to kMaxPinSlots), shown only
    //     when that slot is occupied
    //   - Clear All wipes everything
    //   - Hidden String_knob holds the serialized pin list for persistence
    //     AND undo capture (Nuke's undo stack records all knob value
    //     changes automatically)
    // ========================================================================
    Divider(f, "Manipulate");

    // Pin-refine arm/disarm. The 2D viewer always draws BOTH the gizmo and the
    // pins; this toggle (mirrored by the `P` shortcut over the viewer) routes
    // the mouse. Off = drag the gizmo handles (coarse, by-eye). On = drag pins
    // (fine, pixel-perfect rigid re-solve). The Manipulator selector is gone —
    // there is only one node state (Both) and this flag picks the input target.
    // DO_NOT_WRITE so a saved script never reloads in the armed state.
    Bool_knob(f, &pin_input_active_, "pin_input_active", "Pin Edit  [shift+P]");
    SetFlags(f, Knob::NO_ANIMATION | Knob::DO_NOT_WRITE);
    Tooltip(f, "Route the viewer mouse to the PINS (pixel-perfect refine) instead "
               "of the gizmo. Toggle with shift+P over the viewer. When on, the "
               "gizmo is dimmed/inert and the pin dots brighten; a 'PIN' badge "
               "shows in the viewer. Both editors write the same rigid pose, so "
               "gizmo → pins → gizmo all continue from each other.");

    // One toggle for the index numbers on BOTH overlays — the amber pin dots and
    // the user-track anchor dots. ON by default; turn off to keep the markers but
    // hide the labels. Drawn in wireframe_knob.cpp, gated on show_numbers().
    Bool_knob(f, &show_numbers_, "show_numbers", "Show Numbers");
    ClearFlags(f, Knob::STARTLINE);
    SetFlags(f, Knob::NO_ANIMATION | Knob::NO_UNDO);
    Tooltip(f, "Label each pin and user-track marker with its 1-based index in the "
               "2D viewer. Pins show their stable id; user tracks show their load "
               "order. The markers still draw when this is off.");

    Double_knob(f, &gizmo_size_px_, "gizmo_size", "Gizmo Size");
    SetFlags(f, Knob::NO_ANIMATION | Knob::NO_UNDO);
    Tooltip(f, "On-screen radius of the gizmo handles, in pixels.");

    // Numerical translate / dolly. Mirrors the Rotate fields below: each holds an
    // absolute offset layered on the current working pose, previewed live, kept
    // until "Set Pose Key" bakes it and folds them to 0. trans_x/y/z are WORLD
    // units; Dolly slides along the camera->mesh viewing ray (depth). A gizmo
    // translate/dolly drag fills these in on release, so the move is undoable the
    // same way a Rotate edit is. DO_NOT_WRITE (transient editing aid, like Rotate).
    Double_knob(f, &trans_off_x_, "trans_x", "Translate X");
    ClearFlags(f, Knob::SLIDER);
    SetFlags(f, Knob::NO_ANIMATION | Knob::DO_NOT_WRITE | Knob::KNOB_CHANGED_ALWAYS | Knob::NO_UNDO);
    Tooltip(f, "World-space translation offset on X. Filled in by a gizmo translate "
               "drag and editable directly. Previewed live; commit with Set Pose Key. "
               "Respects the Translate lock.");
    Double_knob(f, &trans_off_y_, "trans_y", "Y");
    ClearFlags(f, Knob::STARTLINE | Knob::SLIDER);
    SetFlags(f, Knob::NO_ANIMATION | Knob::DO_NOT_WRITE | Knob::KNOB_CHANGED_ALWAYS | Knob::NO_UNDO);
    Tooltip(f, "World-space translation offset on Y. Previewed live; commit with Set Pose Key.");
    Double_knob(f, &trans_off_z_, "trans_z", "Z");
    ClearFlags(f, Knob::STARTLINE | Knob::SLIDER);
    SetFlags(f, Knob::NO_ANIMATION | Knob::DO_NOT_WRITE | Knob::KNOB_CHANGED_ALWAYS | Knob::NO_UNDO);
    Tooltip(f, "World-space translation offset on Z. Previewed live; commit with Set Pose Key.");
    Double_knob(f, &dolly_off_, "dolly", "Dolly");
    ClearFlags(f, Knob::STARTLINE | Knob::SLIDER);
    SetFlags(f, Knob::NO_ANIMATION | Knob::DO_NOT_WRITE | Knob::KNOB_CHANGED_ALWAYS | Knob::NO_UNDO);
    Tooltip(f, "Depth offset along the camera->mesh viewing ray (positive = toward the "
               "camera). Filled in by a gizmo dolly drag and editable directly. Previewed "
               "live; commit with Set Pose Key. Respects the Dolly lock.");

    // Numerical rotation about the mesh centre. Each field holds an absolute
    // offset (degrees) layered on the current keyed/upstream pose about the mesh
    // centre. Editing one previews the rotation live in the viewer; the fields
    // keep their values until "Set Pose Key" bakes the pose and folds them to 0.
    // DO_NOT_WRITE: the offset is a transient editing aid (live_scene_ isn't
    // serialized), so a mid-edit save must not persist a stray angle.
    Double_knob(f, &rot_nudge_x_, "rot_x", "Rotate\xC2\xB0 X");
    ClearFlags(f, Knob::SLIDER);
    SetFlags(f, Knob::NO_ANIMATION | Knob::DO_NOT_WRITE | Knob::KNOB_CHANGED_ALWAYS | Knob::NO_UNDO);
    Tooltip(f, "Rotation offset (degrees) about the proxy's local X axis through its "
               "centre. Previewed live; commit with Set Pose Key. Respects the Rotate lock.");
    Double_knob(f, &rot_nudge_y_, "rot_y", "Y");
    ClearFlags(f, Knob::STARTLINE | Knob::SLIDER);
    SetFlags(f, Knob::NO_ANIMATION | Knob::DO_NOT_WRITE | Knob::KNOB_CHANGED_ALWAYS | Knob::NO_UNDO);
    Tooltip(f, "Rotation offset about the proxy's local Y axis through its centre. "
               "Previewed live; commit with Set Pose Key.");
    Double_knob(f, &rot_nudge_z_, "rot_z", "Z");
    ClearFlags(f, Knob::STARTLINE | Knob::SLIDER);
    SetFlags(f, Knob::NO_ANIMATION | Knob::DO_NOT_WRITE | Knob::KNOB_CHANGED_ALWAYS | Knob::NO_UNDO);
    Tooltip(f, "Rotation offset about the proxy's local Z axis through its centre. "
               "Previewed live; commit with Set Pose Key.");

    // Viewer-gesture move history. Nuke's own undo does not reliably capture our
    // viewer-driven gizmo moves (and the viewer-handle Op instance is often a
    // different C++ instance from the one a button click lands on), so the history
    // lives in a shared, knob-backed blob and we step through it with these buttons.
    // Assign hotkeys to them in Nuke's keyboard-shortcut editor for Ctrl+Z feel.
    Button(f, "undo_move", "Undo Move");
    SetFlags(f, Knob::STARTLINE);
    Tooltip(f, "Step BACK through gizmo moves made this session (our own history, "
               "independent of Nuke's Ctrl+Z). Restores the wireframe pose and the "
               "Translate/Dolly/Rotate readout to before the last move.");
    Button(f, "redo_move", "Redo Move");
    ClearFlags(f, Knob::STARTLINE);
    Tooltip(f, "Step FORWARD again through moves reverted with Undo Move.");

    Button(f, "clear_move_history", "Clear History");
    ClearFlags(f, Knob::STARTLINE);
    Tooltip(f, "Wipe the move history (gizmo/pin moves, colour, gradient and the "
               "other tracked knobs). Undo/Redo Move then have nothing to step until "
               "you make a new edit. Does NOT change the current pose, pins, colour "
               "or any knob value — it only forgets the undo trail. Saved with the "
               "script, so the cleared (empty) history persists on reload.");

    Button(f, "refresh_overlay", "Refresh Overlay");
    ClearFlags(f, Knob::STARTLINE);
    Tooltip(f, "Discard an uncommitted pose edit on the current frame so the overlay "
               "snaps back to the keyed animation. Use it when you start posing a "
               "frame with no key (between keyframes), change your mind, and the frame "
               "then flickers off the animation on playback. Zeroes the "
               "Translate/Dolly/Rotate edit and clears the live preview; the frame "
               "falls back to the keyed pose where keys exist, otherwise the geo's "
               "rest position. Keyframes and pins are NOT touched.");

    // Backing store for the move history: one shared String_knob holding the whole
    // stack plus a cursor ("pos N\n<snap0>\n<snap1>..."). Shared across Op instances
    // (so the viewer drag and the panel button see the same data) and SAVED with the
    // script so the history survives save/reload. Still INVISIBLE (no UI clutter),
    // NO_ANIMATION, and NO_UNDO so Ctrl+Z can't corrupt our own history.
    String_knob(f, &move_hist_blob_, "move_hist_blob");
    SetFlags(f, Knob::INVISIBLE | Knob::NO_ANIMATION | Knob::NO_UNDO);

    // Navigation hook a Nuke keyboard-shortcut script can poke: set to -1 for Undo
    // Move, +1 for Redo Move. knob_changed acts on it and resets it to 0. Invisible,
    // not saved, and excluded from Nuke's undo.
    Int_knob(f, &move_nav_, "move_nav");
    SetFlags(f, Knob::INVISIBLE | Knob::DO_NOT_WRITE | Knob::NO_ANIMATION | Knob::NO_UNDO);

    // ========================================================================
    Divider(f, "Pins");

    Multiline_String_knob(f, &pin_list_text_, "pin_list_display", "Pins", 4);
    SetFlags(f, Knob::READ_ONLY | Knob::DO_NOT_WRITE | Knob::NO_ANIMATION);
    Tooltip(f, "Pins live here. Arm Pin Edit (shift+P), then click on a mesh vertex in "
               "the 2D viewer to place a pin. Drag a placed pin to refine — the "
               "rigid pose re-solves continuously so the wireframe lands on your "
               "pixel, with all other pins held as anchors.");

    // Multi-pin removal. pins_to_remove_ is free text: the 1-based positions shown
    // in the Pins list above (the leading "N." numbers), separated by anything —
    // "1,5,9", "1 5 9" and "1, 5, 9" all parse the same (any non-digit is a
    // separator), so the comma is optional. The "delete_selected_pin" button is
    // handled in knob_changed(); it deletes highest-position-first so the lower
    // positions stay valid, re-solves the survivors, and clears the field.
    String_knob(f, &pins_to_remove_, "pins_to_remove", "Pins to remove");
    SetFlags(f, Knob::NO_ANIMATION | Knob::DO_NOT_WRITE);
    Tooltip(f, "1-based positions from the Pins list above (the leading numbers, "
               "e.g. '3.'), NOT pin# ids. Type one or several separated by anything "
               "— '1,5,9' or '1 5 9' both work — then click Remove Pins. The list "
               "renumbers afterwards and this field clears.");

    Button(f, "delete_selected_pin", "Remove Pins");
    ClearFlags(f, Knob::STARTLINE);
    Tooltip(f, "Remove every pin whose list position is named in 'Pins to remove'. "
               "The remaining pins on each affected frame re-solve to relax onto the "
               "surviving constraints; if none remain, the keyed pose is left as-is.");

    Button(f, "clear_pin_key_btn", "Clear Key @ Frame");
    ClearFlags(f, Knob::STARTLINE);
    Tooltip(f, "Drop just the CURRENT frame's 2D key from the pin(s) named in 'Pins "
               "to remove' (same 1-based positions). The pin stays; only its hand "
               "correction on this frame is removed, so it falls back to its linked "
               "track / interpolated curve there.");

    // Hidden serialized state — kept registered in BOTH builds so .nk scripts
    // that contain pins still load cleanly and round-trip. String_knob persists
    // across save/load and its changes are captured by Nuke's undo stack.
    String_knob(f, &pins_blob_, "pins_blob");
    SetFlags(f, Knob::INVISIBLE | Knob::NO_ANIMATION);
    Tooltip(f, "Internal: serialized pin list. Not for direct editing.");

    // Hidden undoable mirror of the gizmo's live pose (model matrix + frame).
    // INVISIBLE + writable (NOT DO_NOT_WRITE) so Nuke's undo stack captures gizmo
    // moves — Nuke's undo serializes through to_script, which DO_NOT_WRITE would
    // exclude, so the knob must be writable to be undoable. Restored by
    // on_live_pose_blob_changed() on Ctrl+Z/Ctrl+Y.
    String_knob(f, &live_pose_blob_, "live_pose_blob");
    SetFlags(f, Knob::INVISIBLE | Knob::NO_ANIMATION);
    Tooltip(f, "Internal: serialized live gizmo pose. Not for direct editing.");

    // Hidden 3D-mask storage (MASKING_PLAN). mask_blob is the per-triangle bitset
    // as concatenated hex; mask_tri_count is the triangle count it was painted
    // against (topology-change guard). Both INVISIBLE but SAVED, so the mask
    // round-trips through the .nk and is captured by undo — like pins_blob. The
    // interactive paint UI (arm / brush / colour / Clear) lands with the brush +
    // overlay; the bitset already feeds Track/Refine via build_mask_array.
    String_knob(f, &mask_blob_, "mask_blob");
    SetFlags(f, Knob::INVISIBLE | Knob::NO_ANIMATION);
    Tooltip(f, "Internal: serialized 3D-mask triangle bitset. Not for direct editing.");

    Int_knob(f, &mask_tri_count_, "mask_tri_count");
    SetFlags(f, Knob::INVISIBLE | Knob::NO_ANIMATION);
    Tooltip(f, "Internal: triangle count the 3D mask was painted against.");

    // ------------------------------------------------------------------------
    // Mask — exclude proxy triangles from the Track / Refine solve (deforming
    // parts, occluders, low-texture areas). Select VERTICES in the 3D viewer with
    // this node active, then "Mask Selected" masks every triangle whose three
    // corner points are all selected; "Unmask Selected" removes them. The bitset
    // round-trips through the hidden mask_blob (persisted in the .nk) and feeds the
    // solver via build_mask_array. Masking is per-triangle (topology), so it
    // applies on every frame regardless of when it was set.
    // ------------------------------------------------------------------------
    // Collapsible group, CLOSED by default — open it only when masking. The
    // geo_select target and the bitset stay live while collapsed (a closed group
    // only hides the UI rows, it doesn't disable the knobs).
    BeginClosedGroup(f, "Mask");
    AColor_knob(f, mask_color_, "mask_color", "Mask Color");
    SetFlags(f, Knob::NO_ANIMATION | Knob::NO_UNDO);
    Tooltip(f, "Tint colour (RGBA) of masked triangles in the 2D viewer overlay. "
               "Alpha controls the fill strength.");

    Button(f, "clear_mask", "Clear Mask");
    Tooltip(f, "Remove every masked triangle.");

    // --- 3D-select route: PolychaseTracker is the 3D selection target ----------
    // The GEOSELECT_KNOB captures 3D-viewer VERTEX selection on this node. Select
    // vertices with this node active, then "Mask Selected" masks every triangle
    // whose three corner points are ALL selected (strict, no spill); "Unmask
    // Selected" removes the mask from the same set. Native 3D selection
    // (marquee/lasso, reaches back faces).
    GeoSelect_knob(f, "geo_select");
    Button(f, "mask_selected", "Mask Selected");
    Tooltip(f, "Mask triangles whose three corner vertices are ALL selected in the "
               "3D viewer (select vertices with this node active). The masked "
               "triangles tint in the 2D viewer.");
    Button(f, "unmask_selected", "Unmask Selected");
    ClearFlags(f, Knob::STARTLINE);   // sit beside Mask Selected
    Tooltip(f, "Remove the mask from triangles whose three corner vertices are ALL "
               "selected in the 3D viewer.");
    EndGroup(f);   // end "Mask" group

    // ------------------------------------------------------------------------
    // 2D Mask (occlusion / mask plate) — exclude IMAGE-SPACE regions from the
    // Track / Refine solve using the alpha of the node's 4th input ("mask"), e.g.
    // a Roto over an occluder that passes IN FRONT of the tracked object. This is
    // independent of the 3D (per-triangle) mask above and stacks with it. "Show
    // Masked" tints the excluded pixels in the 2D viewer — the visual confirmation
    // that the mask is being applied. All OFF by default (mode None = no effect).
    // ------------------------------------------------------------------------
    BeginClosedGroup(f, "2D Mask");
    Enumeration_knob(f, &mask2d_mode_, kMask2DModes, "mask2d_mode", "Mask");
    SetFlags(f, Knob::NO_ANIMATION);
    Tooltip(f, "Use the alpha of the 'mask' input (input 4 — wire a Roto there) to "
               "exclude image regions from Track and Refine.\n"
               "  None: off.\n"
               "  Mask Alpha: exclude pixels where alpha >= Threshold (the roto "
               "covers the occluder).\n"
               "  Mask Alpha Inverted: exclude pixels where alpha < Threshold "
               "(track ONLY inside the roto).\n"
               "The mask is per-frame, so animate the roto to follow a moving "
               "occluder. The mask format should match the plate.");

    Double_knob(f, &mask2d_threshold_, "mask2d_threshold", "Threshold");
    SetFlags(f, Knob::NO_ANIMATION | Knob::NO_UNDO);
    Tooltip(f, "Alpha cutoff that separates masked from unmasked pixels (default "
               "0.5). Raise toward 1 to mask only the solid core of the roto.");

    Bool_knob(f, &mask2d_show_, "mask2d_show", "Show Masked");
    ClearFlags(f, Knob::STARTLINE);   // sit beside Threshold
    SetFlags(f, Knob::NO_ANIMATION | Knob::NO_UNDO);
    Tooltip(f, "Tint the excluded pixels over the 2D plate so you can see exactly "
               "what Track/Refine will drop — both a setup aid and proof the mask "
               "is being applied. Display-only; it does not change the solve.");

    AColor_knob(f, mask2d_color_, "mask2d_color", "Overlay Color");
    SetFlags(f, Knob::NO_ANIMATION | Knob::NO_UNDO);
    Tooltip(f, "Tint colour (RGBA) for the 'Show Masked' overlay. Alpha controls "
               "the fill strength.");
    EndGroup(f);   // end "2D Mask" group

    // ------------------------------------------------------------------------
    // Solved pose — keyframed on the timeline.
    //
    // The solved object pose is stored as animated translate / rotate / scale.
    // Because they're animated, they're written to the .nk (the position
    // survives save/load) and each keyed frame is a keyframe in the Dope Sheet /
    // Curve Editor. Gizmo translate/dolly and the Rotate offsets are previewed
    // live but NOT keyed until "Set Pose Key" — the artist commits the whole
    // pose explicitly; per-key edits/deletes work natively in the Dope Sheet /
    // Curve Editor. Shown as a static section (not a collapsible group).
    // ------------------------------------------------------------------------
    Divider(f, "Solved Pose");
    XYZ_knob(f, pose_t_, "pose_translate", "translate");
    Tooltip(f, "Solved object translation, keyed per solved frame.");
    XYZ_knob(f, pose_r_, "pose_rotate", "rotate");
    Tooltip(f, "Solved object rotation (degrees), keyed per solved frame.");
    XYZ_knob(f, pose_s_, "pose_scale", "scale");
    Tooltip(f, "Solved object scale, keyed per solved frame.");
    // Hidden from the panel by request: the 2-pin solve is a similarity
    // (rotate + uniform scale about the anchor), so scale can legitimately
    // differ from 1 and is still solved, keyed, and applied. It remains
    // editable in the Curve Editor / Dope Sheet. Remove this flag (or drop
    // the knob) once solving moves to 3+ pin full-PnP, where scale is fixed.
    SetFlags(f, Knob::INVISIBLE);

    Button(f, "set_pose_key", "Set Pose Key");
    SetFlags(f, Knob::STARTLINE);
    Tooltip(f, "Key the current solved pose at the current frame.");

    // ========================================================================
    // Intrinsics — recover a VARYING focal (optical zoom) for the shot.
    //
    // 'Solve Focal (P4Pf)' solves the per-frame focal from scratch (PoseLib P4Pf;
    // tracker_focal_pnpf.cpp) off the anchored user tracks and writes the animated
    // 'Solved Focal' curve below. It does NOT re-track — the pose is never touched.
    // Finish with 'Smooth Solved Focal', then tick 'Export Solved Focal' so Export
    // keys the curve onto the spawned Camera2's focal.
    //
    // (Static-lens fitting — fixed focal / principal point — was removed: this pass
    // exists for genuine zooms. For a non-varying lens just leave the focal as-is.)
    // ========================================================================
    // Collapsible group, CLOSED by default (open it when you need the lens).
    BeginClosedGroup(f, "Intrinsics");

    // Solve Focal (P4Pf) — the varying-focal (optical zoom) solver.
    // From-scratch per-frame pose+focal via PoseLib's P4Pf minimal solver in a
    // small LO-RANSAC (tracker_focal_pnpf.cpp). No warm-start lag, and it never
    // reads Nuke's stale cooked camera. Reads the anchored USER TRACKS (>= 4 per
    // frame) and writes ONLY the 'Solved Focal' curve below — then Smooth + Export.
    // The Min/Max FOV bounds below feed its focal sanity window.
    Button(f, "solve_focal_pnpf", "Solve Focal (P4Pf)");
    SetFlags(f, Knob::STARTLINE);
    Tooltip(f, "FROM-SCRATCH per-frame focal solver (PoseLib P4Pf) — the varying-"
               "focal (zoom) solve. At every frame it solves the full camera pose "
               "AND focal from scratch from your anchored user tracks (4-point "
               "minimal solver in a small RANSAC), so there is no warm-start lag and "
               "it never touches Nuke's stale cooked camera. Workflow: pose the "
               "object at the Reference Frame, 'Load Tracks' (needs >= 4 tracks "
               "anchored per frame), press this, then 'Smooth Solved Focal' and tick "
               "'Export Solved Focal'. Writes ONLY the 'Solved Focal' curve below; "
               "the per-frame [fpnpf] log shows each frame's focal + inlier count. "
               "Requires parallax across the shot (camera/object rotation) — which a "
               "real zoom plate has.");

    Bool_knob(f, &opt_focal_, "opt_focal", "Export Solved Focal");
    ClearFlags(f, Knob::STARTLINE);   // sit beside Solve Focal (P4Pf)
    SetFlags(f, Knob::NO_ANIMATION | Knob::NO_UNDO);
    Tooltip(f, "EXPORT GATE only. When ticked AND a focal has been solved, Export "
               "keys the 'Solved Focal' curve onto the spawned Camera2's focal. "
               "Unticked (or with nothing solved) Export uses the input camera's "
               "static focal. Press 'Solve Focal (P4Pf)' first.");

    Double_knob(f, &min_fov_deg_, "min_fov", "Min FOV");
    SetFlags(f, Knob::NO_ANIMATION | Knob::NO_UNDO);
    Tooltip(f, "Lower horizontal-FOV bound (degrees) the solved focal is clamped "
               "to. Tighten around a known lens to stop the focal running away on "
               "under-constrained shots. Default 15.");
    Double_knob(f, &max_fov_deg_, "max_fov", "Max FOV");
    ClearFlags(f, Knob::STARTLINE);
    SetFlags(f, Knob::NO_ANIMATION | Knob::NO_UNDO);
    Tooltip(f, "Upper horizontal-FOV bound (degrees) for the solved focal. "
               "Default 160. If the solved focal pins to a bound, the shot is "
               "under-constrained — widen the object's motion or fix the focal.");

    Double_knob(f, &solved_focal_, "solved_focal", "Solved Focal (mm)");
    SetFlags(f, Knob::NO_UNDO);
    Tooltip(f, "Per-frame solved focal length in millimetres, baked by 'Solve Focal "
               "(P4Pf)' (animated; edit in the Curve Editor if needed). Export keys "
               "this onto the Camera2's focal when 'Export Solved Focal' is ticked; "
               "you can also expression-link a Camera2's focal to it for a live "
               "preview. Empty (unanimated) means the focal was not solved.");

    // Solved-focal curve cleanup — NON-destructive finishing op on the 'Solved Focal'
    // curve above; the pose is never touched. See on_smooth_focal (tracker_intrinsics.cpp).
    Button(f, "smooth_focal", "Smooth Solved Focal");
    SetFlags(f, Knob::STARTLINE);   // new row, under the Solved Focal curve
    Tooltip(f, "Centred moving-average of the 'Solved Focal' curve (window below) — "
               "tames per-frame jitter on the solved zoom while keeping the trend. "
               "Non-destructive (focal curve only). Re-run for more smoothing.");

    Int_knob(f, &focal_smooth_window_, "focal_smooth", "Smooth Window");
    ClearFlags(f, Knob::STARTLINE);
    SetFlags(f, Knob::NO_ANIMATION | Knob::NO_UNDO);
    Tooltip(f, "Moving-average window (frames) used by 'Smooth Solved Focal'. Forced "
               "odd so it is centred; larger = smoother. Default 5.");

    // Bake the solved focal straight onto the CONNECTED camera (mutates the input).
    Button(f, "copy_focal_to_camera", "Copy Solved Focal \xE2\x86\x92 Camera");
    SetFlags(f, Knob::STARTLINE);
    Tooltip(f, "Write the 'Solved Focal' curve onto the CONNECTED input camera's "
               "focal (animated, over First..Last Frame). Unlike Export this MODIFIES "
               "your camera node in place, so the camera then carries the zoom and the "
               "overlay shows it WITHOUT 'Preview Solved Lens' — you can untick that "
               "afterwards. Undoable. Note: it copies focal only (haperture is left "
               "alone, since the solved mm already assume the camera's aperture). "
               "Needs a solved focal (Solve Focal first) and a camera on the 'cam' "
               "input.");

    EndGroup(f);   // end "Intrinsics" group

    // ========================================================================
    // Track — the inner loop the artist lives in.
    // ========================================================================
    Divider(f, "Track");

    Button(f, "trackit", "Track Forward");
    Tooltip(f, "Solve the WHOLE range, First Frame -> Last Frame, every time — "
               "no matter which frame the playhead is on. The track is seeded "
               "from the pose you keyed on the FIRST frame, so set your pose key "
               "there before pressing this. One button, one consistent result "
               "every press.");

    Button(f, "trackit_back", "Track Backwards");
    ClearFlags(f, Knob::STARTLINE);   // sit on the TrackIt row
    Tooltip(f, "Solve BACKWARDS from the frame the playhead is on down to First "
               "Frame. Go to the frame where you keyed your pose (e.g. 100), "
               "confirm 'Set Pose Key' is done there, then press this to track "
               "back to First Frame.");

    // When ON, Track Forward / Backwards solve an EXACT per-frame PnP from the
    // pin<->track bindings instead of the optical-flow DB: pins (exact mesh
    // vertices) are paired to tracks once at the ref frame, then each frame's pose
    // is solved from those vertices + the tracks' per-frame pixels and keyed.
    Bool_knob(f, &track_only_user_, "track_only_user", "Use Only User Tracks");
    SetFlags(f, Knob::STARTLINE | Knob::NO_ANIMATION | Knob::NO_UNDO);
    Tooltip(f, "Solve Track from the pins + user tracks alone, ignoring the optical-"
               "flow database. Pins are paired to tracks at the Reference Frame "
               "(within Connect Max Dist), then every frame is solved by exact PnP "
               "from the pinned vertices and the tracks' 2D. Needs >=3 pin<->track "
               "pairs and loaded tracks. Off = normal flow-based Track.");

    // ========================================================================
    // Refine — global bundle adjustment between anchors. After a first Track,
    // hand-correct a few drifting frames (pin/gizmo + Set Pose Key); each
    // correction is auto-recorded as an ANCHOR. Refine Range re-solves the frames
    // BETWEEN anchors inside the typed Range, holding the range ends and any
    // interior anchors fixed. See tracker_refine.cpp / REFINE_PLAN.md.
    // ========================================================================
    Divider(f, "Refine");

    String_knob(f, &refine_range_, "refine_range", "Range");
    SetFlags(f, Knob::NO_ANIMATION | Knob::NO_UNDO);
    Tooltip(f, "Frame range to refine, e.g. '1001-1100' (any separator works: "
               "'1001-1100', '1001 1100', '1001,1100'). The range ends are held "
               "fixed as anchors, along with any anchors inside them; the interior "
               "frames re-solve toward the flow. Leave empty to use First..Last "
               "Frame — the field then fills in with the resolved range.");

    Button(f, "refine_range_go", "Refine Range");
    ClearFlags(f, Knob::STARTLINE);   // sit beside the Range field
    Tooltip(f, "Re-solve every gap between anchors inside Range, holding the range "
               "ends and any interior anchors fixed. Hand-correct at least one "
               "interior frame first (pin/gizmo + Set Pose Key) so there's a gap to "
               "solve — each correction becomes an anchor automatically.");

    String_knob(f, &refine_anchors_text_, "refine_anchor_list", "Anchors");
    SetFlags(f, Knob::READ_ONLY | Knob::DO_NOT_WRITE | Knob::NO_ANIMATION);
    Tooltip(f, "Frames currently held fixed by Refine. Each is recorded "
               "automatically when you Set Pose Key on a hand-corrected frame. "
               "The Range ends are always boundaries too, even if not listed.");

    Button(f, "clear_refine_anchors", "Clear Refine Anchors");
    SetFlags(f, Knob::STARTLINE);
    Tooltip(f, "Forget all recorded anchors (start the corrections over). Only the "
               "Range ends then bound Refine. Does NOT touch the pose keyframes — "
               "re-correct frames to record new anchors. A fresh Track also clears "
               "anchors automatically.");

    Button(f, "add_refine_anchor_btn", "Add Anchor");
    ClearFlags(f, Knob::STARTLINE);
    Tooltip(f, "Hold the CURRENT timeline frame fixed in Refine (at its current keyed "
               "pose). Go to the frame you want and press this. Adds to the list — "
               "press it on as many frames as you like; if the frame is already an "
               "anchor it does nothing.");

    // Opt-in: feed the connected user tracks (exact pinned-vertex anchors) into
    // Refine as extra constraints. Off by default so Refine behaves as before.
    Bool_knob(f, &refine_use_user_tracks_, "refine_use_user_tracks", "Use User Tracks");
    SetFlags(f, Knob::STARTLINE | Knob::NO_ANIMATION | Knob::NO_UNDO);
    Tooltip(f, "When on, Refine also uses the Connected user tracks as helper "
               "constraints (each anchored at its exact pinned vertex), in addition "
               "to the optical flow. Connect pins to tracks first. Off = Refine uses "
               "flow + anchors only.");

    // Hidden, SAVED storage for the anchor list (sorted comma-separated frames).
    // Persisted (NOT DO_NOT_WRITE) so corrections survive save/load, and writable
    // so Nuke's undo stack captures it — same pattern as pins_blob. Resynced to
    // refine_anchors_ via knob_changed("refine_anchors") on undo/.nk load.
    String_knob(f, &refine_anchors_blob_, "refine_anchors");
    SetFlags(f, Knob::INVISIBLE | Knob::NO_ANIMATION);
    Tooltip(f, "Internal: serialized Refine anchor frames. Not for direct editing.");

    // ========================================================================
    // Export — the deliverable. Solve Mode is here, not in the Database section, because
    // it's an output choice: the same solve can be exported as either a
    // moving camera or a moving geo.
    // ========================================================================
    Divider(f, "Export");

    Enumeration_knob(f, &solve_mode_, kSolveModes, "solve_mode", "Solve Mode");
    SetFlags(f, Knob::NO_UNDO);
    Tooltip(f, "Camera: spawn a Camera2 animated with the solved poses.\n"
               "Model: spawn a TransformGeo animated with the solved poses.\n"
               "The underlying solve is the same; only the output node "
               "type differs.");

    // Export sits on the SAME line as Solve Mode so the Model-only checkbox below can
    // appear/disappear without shoving the Export button around.
    Button(f, "export_solve", "Export");
    ClearFlags(f, Knob::STARTLINE);   // sit beside Solve Mode
    Tooltip(f, "Spawn a Polychase_Camera_1 or Polychase_TransformGeo_1 with "
               "the solved per-frame poses as keyframes. Re-exporting "
               "replaces the existing one if it still exists.");

    // Model-mode helper: also spawn a camera carrying the solved zoom focal, since a
    // bare TransformGeo has no lens and won't line up on a zoom through a flat camera.
    // Its OWN line, BELOW the Solve Mode / Export row, and hidden in Camera mode
    // (toggled in knob_changed) — so showing/hiding it never moves Export.
    Bool_knob(f, &export_lens_camera_, "export_lens_camera", "Export Zoom Lens Camera");
    SetFlags(f, Knob::STARTLINE | Knob::NO_ANIMATION | Knob::NO_UNDO);
    Tooltip(f, "MODEL mode only (hidden in Camera mode). On Export, also spawn "
               "'Polychase_LensCamera_1' — a camera that matches the input camera's "
               "viewpoint but carries the SOLVED per-frame focal (the zoom). Render "
               "the exported TransformGeo's geo through THIS camera and the object "
               "locks across the zoom; the bare TransformGeo viewed through a static-"
               "focal camera drifts in scale.\n\n"
               "The pose is baked from the input camera (lookat/constraints included) "
               "as editable ZXY curves; aperture and lens shift are copied. It NEVER "
               "edits your input camera. On by default. No effect without a Solved "
               "Focal curve, or with no camera wired to the cam input. Re-exporting "
               "refreshes the same camera.");

    // ========================================================================
    // Status — passive log of what just happened.
    // ========================================================================
    Divider(f, "Status");

    Multiline_String_knob(f, &status_text_, "status", "Status", 8);
    SetFlags(f, Knob::DISABLED | Knob::NO_ANIMATION | Knob::DO_NOT_WRITE);
    Tooltip(f, "Status output from buttons (Track/Export/etc). Also mirrored to "
               "stdout when the plugin is built with debug logging (PCN_DEBUG).");

    // ========================================================================
    // Hidden serialized state.
    // ========================================================================
    Multiline_String_knob(f, &keys_blob_, "keys", "Keys (JSON)");
    SetFlags(f, Knob::INVISIBLE);

    String_knob(f, &anchor_node_, "anchor_node", "Anchor Node");
    SetFlags(f, Knob::INVISIBLE);

    // Seed frame of the last Track, persisted so Export can
    // reconstruct the seed pose/camera (model0, cam-at-seed) after a reload.
    Int_knob(f, &track_seed_frame_, "track_seed_frame", "Track Seed Frame");
    SetFlags(f, Knob::INVISIBLE | Knob::NO_ANIMATION);

    // ========================================================================
    // Viewer overlay knob — draws the wireframe over the 2D plate.
    //
    // Custom Knob subclass whose build_handle/draw_handle get invoked by the
    // 2D viewer (unlike Op::draw_handle which only fires in the 3D scene).
    // Registered with CustomKnob1; the second arg `this` is passed through
    // to the knob's constructor and (later) used to reach back into the Op
    // for input access. INVISIBLE so it doesn't show as a knob row.
    // ========================================================================
    CustomKnob1(PolychaseWireframeKnob, f, this, "wireframe_overlay");
    SetFlags(f, Knob::INVISIBLE);

    // ========================================================================
    // User Tracks — on its OWN tab (after the main panel, before Live Camera) so
    // the Status list has room to work. Feed 2D tracks made with a Tracker node
    // into the solve as extra stabilising constraints. Each track is anchored to
    // the mesh by ray-casting its position on the Reference Frame; tracks whose
    // ray misses the mesh are ignored ("only trackers on the object count"). Load
    // reads the named Tracker node; Track/Refine then use the anchored set.
    // "Show User Tracks" overlays them. See USER_TRACKS_PLAN.md.
    // ========================================================================
    Tab_knob(f, "User Tracks");
    Divider(f, "User Tracks");

    // Master overlay toggle on top, on its own line.
    Bool_knob(f, &user_tracks_show_, "user_tracks_show", "Show User Tracks");
    SetFlags(f, Knob::STARTLINE | Knob::NO_ANIMATION | Knob::NO_UNDO);
    Tooltip(f, "Overlay the tracks on the 2D viewer: a CROSS at each track's measured "
               "position on the current frame, and a DOT where its mesh anchor "
               "projects through the solved pose. They coincide when the track sits "
               "on the object and the solve is good — your visual proof.");

    String_knob(f, &user_track_src_, "user_track_src", "Tracker Node");
    SetFlags(f, Knob::STARTLINE | Knob::NO_ANIMATION);
    Tooltip(f, "Name of the Tracker node whose 2D tracks to use (type the node's "
               "exact name, e.g. 'Tracker1'). Only tracks that land ON the proxy "
               "are used; the rest are ignored.");

    Int_knob(f, &user_track_ref_, "user_track_ref", "Reference Frame");
    SetFlags(f, Knob::NO_ANIMATION | Knob::NO_UNDO);
    Tooltip(f, "Frame on which each track is anchored to the mesh (by ray-cast). "
               "Pick a frame where the proxy is well aligned and the tracks sit on "
               "the object. A track with no key on this frame can't be anchored.");

    Int_knob(f, &user_track_overscan_, "track_overscan", "Overscan (px)");
    ClearFlags(f, Knob::STARTLINE);
    SetFlags(f, Knob::NO_ANIMATION | Knob::NO_UNDO);
    Tooltip(f, "Grow the proxy silhouette by this many pixels when anchoring. With a "
               "tight wireframe, tracks sitting just off an edge ray-miss the mesh and "
               "are dropped; with overscan, a near-miss snaps to the nearest surface "
               "point within this many pixels. 0 = exact silhouette. Re-anchors live.");

    Button(f, "load_user_tracks", "Load Tracks");
    SetFlags(f, Knob::STARTLINE);
    Tooltip(f, "Read the 2D tracks from the named Tracker node and store them. Re-run "
               "after editing the tracks or changing the Reference Frame. Anchoring "
               "to the mesh happens here and at every Track/Refine.");
    Button(f, "clear_user_tracks", "Clear");
    ClearFlags(f, Knob::STARTLINE);
    Tooltip(f, "Forget the loaded user tracks.");

    AColor_knob(f, user_tracks_color_, "user_tracks_color", "Marker Color");
    SetFlags(f, Knob::STARTLINE | Knob::NO_ANIMATION | Knob::NO_UNDO);
    Tooltip(f, "Colour of the user-track overlay markers.");

    // Pin -> Track linking (v1: auto-nearest). Before tracking, pair each track to
    // its nearest pin and snap the proxy onto the tracks, keying the start pose.
    Divider(f, "Pin -> Track Linking");

    Double_knob(f, &connect_max_dist_, "connect_max_dist", "Connect Max Dist");
    SetFlags(f, Knob::NO_ANIMATION | Knob::NO_UNDO);
    Tooltip(f, "Pixel cutoff for auto-pairing on the Reference Frame: a track whose "
               "nearest free pin is farther than this (in screen pixels) is left "
               "unpaired. Raise it if the proxy starts far from the tracks; lower it "
               "to avoid wrong matches.");

    Button(f, "connect_pins_tracks", "Connect");
    ClearFlags(f, Knob::STARTLINE);
    Tooltip(f, "On the Reference Frame: pair each user track to its nearest pin "
               "(1 pin per track, within Connect Max Dist), move those pins onto the "
               "track positions, run the rigid pin solve, and Set Pose Key. Park the "
               "viewer on the Reference Frame first. This is the 'line up before "
               "tracking' step.");

    Button(f, "reconnect_pins_tracks", "Re-Connect");
    ClearFlags(f, Knob::STARTLINE);
    Tooltip(f, "Clean redo: forget ALL existing pin targets and bindings first, then "
               "run a fresh Connect. Use this when a previous Connect paired the wrong "
               "pins/tracks or left stale pin targets behind — those would otherwise "
               "fight the new solve. Same Reference-Frame requirement as Connect.");

    Multiline_String_knob(f, &user_tracks_text_, "user_track_list", "Status", 6);
    SetFlags(f, Knob::READ_ONLY | Knob::DO_NOT_WRITE | Knob::NO_ANIMATION);
    Tooltip(f, "Loaded user-track summary: total parsed, and how many were anchored "
               "to the object (those are the ones the solve uses).");

    // Hidden serialized raw 2D tracks (Nuke y-up px), filled by Load Tracks.
    // Persisted + undoable like pins_blob.
    String_knob(f, &user_tracks_blob_, "user_tracks_blob");
    SetFlags(f, Knob::INVISIBLE | Knob::NO_ANIMATION);
    Tooltip(f, "Internal: serialized raw 2D user tracks. Not for direct editing.");

    // ========================================================================
    // Live Camera Solve — on its OWN tab so the controls don't crowd the main
    // panel. Animated curves baked from the solve (same math as Export's Camera
    // mode), refreshed on Track, on each pin/gizmo commit, and via the button.
    // Expression-link a Camera2's translate/rotate to these (useMatrix off,
    // rot_order ZXY, focal/aperture from your input cam) to preview the matchmove.
    // ========================================================================
    Tab_knob(f, "Live Camera");

    Divider(f, "Live Camera Solve");
    XYZ_knob(f, live_cam_t_, "live_cam_translate", "translate");
    Tooltip(f, "Live camera-solve TRANSLATION, baked from the solve on every Track "
               "and pin/gizmo commit. Expression-link a Camera2's translate to "
               "this. Set the camera useMatrix OFF, rot_order = ZXY, and copy "
               "focal/aperture from your input camera. Export still spawns a "
               "standalone camera for delivery.");

    XYZ_knob(f, live_cam_r_, "live_cam_rotate", "rotate");
    Tooltip(f, "Live camera-solve ROTATION (degrees, ZXY order). Expression-link "
               "your Camera2's rotate to this. Baked on Track and on every commit.");

    Button(f, "bake_live_camera", "Refresh Live Camera");
    Tooltip(f, "Recompute the Live Camera curves over First..Last from the current "
               "solved pose. Runs automatically after Track and after each pin or "
               "gizmo commit; use this to refresh after manual key edits in the "
               "Curve Editor.");

    // ========================================================================
    // About — credits + thanks. Static text knobs (no value, nothing saved).
    // ========================================================================
    Tab_knob(f, "About");
    Divider(f, "PolychaseTracker");

    Text_knob(f, "Mesh-based 3D object tracker for Nuke.");
    SetFlags(f, Knob::STARTLINE);
    Text_knob(f, "Version 1.0");
    SetFlags(f, Knob::STARTLINE);
    Text_knob(f, "Developed by Peter Mercell");
    SetFlags(f, Knob::STARTLINE);
    Text_knob(f, " ");
    SetFlags(f, Knob::STARTLINE);
    Text_knob(f, "Built on Polychase, an open-source motion tracker");
    SetFlags(f, Knob::STARTLINE);
    Text_knob(f, "by Ahmed Essam (theartful) and contributors.");
    SetFlags(f, Knob::STARTLINE);
    Text_knob(f, "github.com/theartful/polychase  —  thank you!");
    SetFlags(f, Knob::STARTLINE);
    Text_knob(f, " ");
    SetFlags(f, Knob::STARTLINE);
    Text_knob(f, "Zoom / focal solve uses PoseLib (P4Pf minimal solver)");
    SetFlags(f, Knob::STARTLINE);
    Text_knob(f, "by Viktor Larsson and contributors (BSD-3-Clause).");
    SetFlags(f, Knob::STARTLINE);
    Text_knob(f, "P4Pf after Kukelova et al. (E3Q3, CVPR 2016).");
    SetFlags(f, Knob::STARTLINE);
    Text_knob(f, "github.com/PoseLib/PoseLib  —  thank you!");
    SetFlags(f, Knob::STARTLINE);
    Text_knob(f, " ");
    SetFlags(f, Knob::STARTLINE);
    Text_knob(f, "License");
    SetFlags(f, Knob::STARTLINE);
    Text_knob(f, "PolychaseTracker  Copyright (C) 2026  Peter Mercell");
    SetFlags(f, Knob::STARTLINE);
    Text_knob(f, "Licensed under the GNU General Public License v3.0 (GPL-3.0).");
    SetFlags(f, Knob::STARTLINE);
    Text_knob(f, "This program comes with ABSOLUTELY NO WARRANTY, and is free");
    SetFlags(f, Knob::STARTLINE);
    Text_knob(f, "software you may redistribute under the terms of that license.");
    SetFlags(f, Knob::STARTLINE);
    Text_knob(f, "Polychase is also GPL-3.0; see the bundled COPYING file.");
    SetFlags(f, Knob::STARTLINE);
    Text_knob(f, "github.com/theartful/polychase  (GPL-3.0)");
    SetFlags(f, Knob::STARTLINE);
    Text_knob(f, "PoseLib is BSD-3-Clause (GPL-compatible); see its bundled LICENSE.");
    SetFlags(f, Knob::STARTLINE);
}


// -----------------------------------------------------------------------------
// Knob dispatch.
// -----------------------------------------------------------------------------
int PolychaseTracker::knob_changed(Knob* k)
{
    if (k == &Knob::showPanel) {
        if (!status_text_ || status_text_[0] == '\0') {
            set_status("ready");
        }
        // Repopulate the pin list from the saved blob and refresh the display.
        // knob_changed("pins_blob") may not have fired on .nk load, so do it
        // here when the user opens the panel.
        ensure_pins_loaded();
        refresh_pin_ui();
        // Load the saved Refine anchors (knob_changed("refine_anchors") may not
        // have fired on .nk load) and echo them in the read-only display.
        if (!refine_anchors_loaded_) load_anchors_from_knob();
        refresh_anchor_label();
        // Restore the 3D mask from the saved blob (knob_changed("mask_blob") may
        // not fire on .nk load). Content-diff rather than the !mask_loaded_ one-shot,
        // so a premature load (mask_loaded_ set true from an empty blob before knob
        // values were deserialized) still reloads once the real hex is present.
        ensure_mask_loaded();
        // Capture the current ordinary-knob values (colour, First/Last Frame, Gizmo
        // Size, Solve Mode) as the "before" baseline so the
        // first such edit this session is undoable back to where it started.
        prime_committed_from_live();
        // Initial visibility for the Model-mode-only 'Export Zoom Lens Camera' knob.
        if (Knob* lk = knob("export_lens_camera")) lk->visible(solve_mode_ == 1);
        return 1;
    }
    if (k) {
        // TrackIt always solves the whole range First->Last, seeded from the
        // first frame, regardless of the playhead. One button, one result.
        if (k->is("trackit"))         { on_track("forward");       return 1; }
        if (k->is("trackit_back"))    { on_track("backward");      return 1; }
        if (k->is("clear_pins"))      { clear_pins();              return 1; }
        if (k->is("clear_mask"))      { clear_mask();              return 1; }
        if (k->is("mask_selected"))   { apply_3d_vertex_selection(false); return 1; }
        if (k->is("unmask_selected")) { apply_3d_vertex_selection(true);  return 1; }
        // Tint colour is display-affecting: repaint so the mask recolours live.
        if (k->is("mask_color")) { asapUpdate(); return 1; }

        // 2D occlusion-mask controls. None changes the image output (NoIop), so
        // invalidate() to force the overlay to recook/repaint, then asapUpdate().
        // The overlay-bits cache keys on mode/threshold/frame/input, so it rebuilds
        // automatically on the next redraw.
        if (k->is("mask2d_mode") || k->is("mask2d_threshold") ||
            k->is("mask2d_show") || k->is("mask2d_color")) {
            invalidate();
            asapUpdate();
            return 1;
        }

        // User (helper) tracks.
        if (k->is("load_user_tracks"))  { on_load_user_tracks(); return 1; }
        if (k->is("clear_user_tracks")) { clear_user_tracks();   return 1; }
        if (k->is("connect_pins_tracks")) { connect_pins_to_tracks(); return 1; }
        if (k->is("reconnect_pins_tracks")) { reconnect_pins_to_tracks(); return 1; }
        // Raw-tracks blob changed (Load / undo / .nk load): drop the anchored cache
        // and rebuild + refresh the summary. rebuild guards on missing inputs.
        if (k->is("user_tracks_blob")) {
            user_tracks_valid_ = false;
            ensure_user_tracks_built();
            refresh_user_tracks_label();
            asapUpdate();
            return 1;
        }
        // Reference frame changes the anchors, so invalidate the cache. Overlay
        // knobs are display-only — just repaint.
        if (k->is("user_track_ref") || k->is("track_overscan")) {
            user_tracks_valid_ = false;
            invalidate();
            asapUpdate();
            return 1;
        }
        if (k->is("user_tracks_show") ||
            k->is("user_tracks_color") || k->is("show_numbers")) {
            invalidate();
            asapUpdate();
            return 1;
        }

        // Ordinary tracked knobs — colour, First/Last Frame, Gizmo Size and Solve
        // Mode all record into the move history so Ctrl+Z /
        // Undo Move steps them, in one stack with pose/pin moves. Skip while we're
        // the ones writing during a move-history restore. (Gizmo Size also just wants
        // a repaint, which on_tracked_knob_changed does via asapUpdate.)
        {
            const char* which = nullptr;
            if      (k->is("wire_color"))        which = "wire_color";
            else if (k->is("wire_gradient"))     which = "wire_gradient";
            else if (k->is("gizmo_size"))        which = "gizmo_size";
            else if (k->is("first_frame"))       which = "first_frame";
            else if (k->is("last_frame"))        which = "last_frame";
            else if (k->is("solve_mode"))        which = "solve_mode";
            if (which) {
                // 'Export Zoom Lens Camera' is a Model-mode-only control: show it for
                // Model (solve_mode_ == 1), hide it for Camera. Done before the suppress
                // early-out so an undo/redo of Solve Mode re-syncs visibility too.
                // NOTE (verify on build): Knob::visible(bool) is the DDImage show/hide
                // setter; if a given SDK spells it differently, this is the one line to
                // adjust.
                if (k->is("solve_mode"))
                    if (Knob* lk = knob("export_lens_camera")) lk->visible(solve_mode_ == 1);
                if (suppress_tracked_callback_) return 1;
                on_tracked_knob_changed(which);
                return 1;
            }
        }

        // Pin-refine arm/disarm checkbox changed (panel click). Sync the flag
        // from the knob and repaint so the gizmo dims / pins brighten. The P
        // shortcut goes through toggle_pin_input(), which writes this knob, so
        // this handler also covers the keyboard path.
        if (k->is("pin_input_active")) {
            if (Knob* kk = knob("pin_input_active"))
                pin_input_active_ = (kk->get_value() != 0.0);
            if (!pin_input_active_) {
                // Disarmed: only the gizmo is live now. End any in-progress pin
                // gesture so a stray drag can't keep nudging the last pin, and store
                // the current refined pose into the Translate/Dolly/Rotate knobs so
                // the readout reflects where the pins left it (these set_value writes
                // merge into this checkbox action's single undo step).
                dragging_pin_idx_ = -1;
                drag_anchor_.reset();
                if (live_scene_) sync_offsets_from_pose(live_scene_->model_matrix);
            }
            asapUpdate();
            return 1;
        }
        // Numerical rotation about the mesh centre. Each field is an absolute
        // offset PREVIEWED (not keyed) as it's edited; "Set Pose Key" commits.
        if (k->is("rot_x") || k->is("rot_y") || k->is("rot_z")) {
            if (suppress_rot_callback_) return 1;
            preview_center_rotation();   // live_scene_ + asapUpdate(), NO key
            return 1;
        }

        // Numerical translate / dolly — same contract as the rotation fields.
        if (k->is("trans_x") || k->is("trans_y") || k->is("trans_z") || k->is("dolly")) {
            if (suppress_trans_callback_) return 1;
            preview_translate_offset();  // live_scene_ + asapUpdate(), NO key
            return 1;
        }

        if (k->is("set_pose_key"))    { key_pose_at_current_frame();
                                        // Persistent offsets: do NOT reset the
                                        // translate/dolly/rotate fields to 0 on
                                        // commit. They stay as the running pose
                                        // relative to the upstream entry point, so
                                        // the artist carries them frame-to-frame
                                        // and always sees how far they've moved.
                                        // Auto-refresh the Live Camera over the whole
                                        // range so a linked Camera2 follows immediately —
                                        // no manual "Refresh Live Camera" click needed.
                                        // key_pose_at_current_frame already refreshed THIS
                                        // frame's key; this rebakes the rest to stay
                                        // consistent. No-ops if nothing was keyed.
                                        bake_live_camera();
                                        return 1; }
        if (k->is("clear_pose_keys")) { clear_pose_keys();                   return 1; }

        // Our own move-history undo/redo (viewer gizmo gestures), backed by the
        // shared move_hist_blob knob so it survives the viewer/panel instance split.
        // load_move_history() reads the stack + cursor; we move the cursor and
        // restore_move_state() applies the target (which republishes through
        // live_pose_blob so the viewer instance's poller rebuilds the wireframe).
        if (k->is("undo_move")) { do_undo_move(); return 1; }
        if (k->is("redo_move")) { do_redo_move(); return 1; }
        if (k->is("clear_move_history")) { clear_move_history(); return 1; }
        if (k->is("refresh_overlay"))    { refresh_overlay();    return 1; }
        // Python-settable navigation hook: a hotkey script sets move_nav to -1 (undo)
        // or +1 (redo); we act and reset it to 0. This is the reliable way to drive
        // the history from a Nuke keyboard shortcut, since an NDK Button can't be
        // fired from Python the way a value change can.
        if (k->is("move_nav")) {
            if (move_nav_ < 0)      do_undo_move();
            else if (move_nav_ > 0) do_redo_move();
            if (move_nav_ != 0) { move_nav_ = 0; if (Knob* mk = knob("move_nav")) mk->set_value(0); }
            return 1;
        }

        // Multi-pin delete. Parse every integer out of the free-text field (any
        // non-digit separates, so "1,5,9" / "1 5 9" / "1, 5, 9" all work), then
        // delete those 1-based positions highest-first so the lower positions stay
        // valid as the list shrinks. All writes merge into this one button action's
        // undo step; the field is cleared afterwards.
        if (k->is("delete_selected_pin")) {
            ensure_pins_loaded();
            std::vector<int> pos;
            {
                int cur = 0; bool in = false;
                for (const char* p = pins_to_remove_ ? pins_to_remove_ : ""; *p; ++p) {
                    if (*p >= '0' && *p <= '9') { cur = cur * 10 + (*p - '0'); in = true; }
                    else if (in) { pos.push_back(cur); cur = 0; in = false; }
                }
                if (in) pos.push_back(cur);
            }
            std::sort(pos.begin(), pos.end(), [](int a, int b){ return a > b; });
            pos.erase(std::unique(pos.begin(), pos.end()), pos.end());

            int removed = 0;
            for (int p : pos) {
                if (p >= 1 && p <= (int)pins_.size()) { delete_pin_at((unsigned)(p - 1)); ++removed; }
            }
            PCN_LOG("[PolychaseTracker] Remove Pins: requested {"
                      << (pins_to_remove_ ? pins_to_remove_ : "")
                      << "} removed " << removed << " (have " << pins_.size()
                      << " left)\n");
            if (Knob* rk = knob("pins_to_remove")) rk->set_text("");   // clear for next time
            asapUpdate();
            return 1;
        }

        // Clear just the current frame's key from the named pin(s). Same field +
        // 1-based positions as Remove Pins; only the per-frame hand correction is
        // dropped, the pins stay. (Positions don't shift, so no highest-first.)
        if (k->is("clear_pin_key_btn")) {
            ensure_pins_loaded();
            const int frame = editing_frame();
            std::vector<int> pos;
            {
                int cur = 0; bool in = false;
                for (const char* p = pins_to_remove_ ? pins_to_remove_ : ""; *p; ++p) {
                    if (*p >= '0' && *p <= '9') { cur = cur * 10 + (*p - '0'); in = true; }
                    else if (in) { pos.push_back(cur); cur = 0; in = false; }
                }
                if (in) pos.push_back(cur);
            }
            int cleared = 0;
            for (int p : pos)
                if (p >= 1 && p <= (int)pins_.size() && clear_pin_key_at((unsigned)(p - 1), frame))
                    ++cleared;
            set_status("[" + timestamp() + "] Cleared " + std::to_string(cleared)
                       + " key(s) @ frame " + std::to_string(frame) + ".");
            if (Knob* rk = knob("pins_to_remove")) rk->set_text("");
            refresh_pin_ui();
            asapUpdate();
            return 1;
        }

        // pins_blob value changed — either loaded from .nk, or undo/redo
        // walked us back to a previous state. Resync pins_ from the blob.
        if (k->is("pins_blob")) {
            on_pins_blob_changed();
            return 1;
        }
        // live_pose_blob changed — a gizmo move being undone/redone (or restored
        // on load). Rebuild live_scene_ from it so the overlay reverts/advances.
        if (k->is("live_pose_blob")) {
            on_live_pose_blob_changed();
            return 1;
        }
        // mask_blob changed — undo/redo or .nk load. Reload the bitset only when
        // the content differs from what we last wrote, so our own async set_text
        // echo is ignored (same guard as the pins blob).
        if (k->is("mask_blob")) {
            // needs_reload() folds both the suppress-our-own-write guard and the
            // content-differs check into one: reloads on a genuine undo/redo/.nk
            // change, ignores our own async set_text echo.
            if (mask_mirror_.needs_reload(mask_blob_)) load_mask_from_knob();
            return 1;
        }
        if (k->is("solve_focal_pnpf"))     { on_solve_focal_pnpf();     return 1; }
        if (k->is("smooth_focal"))    { on_smooth_focal();    return 1; }
        if (k->is("copy_focal_to_camera")) { on_copy_focal_to_camera(); return 1; }
        if (k->is("export_solve"))    { on_export();               return 1; }

        // Refine — global bundle adjustment between anchors (tracker_refine.cpp).
        if (k->is("refine_range_go"))       { on_refine();             return 1; }
        if (k->is("clear_refine_anchors"))  { clear_refine_anchors();  return 1; }
        if (k->is("add_refine_anchor_btn")) {
            const int frame = editing_frame();          // the current timeline frame
            const bool added = add_refine_anchor(frame); // true if newly inserted
            refresh_anchor_label();
            set_status("[" + timestamp() + "] "
                       + (added ? "Refine anchor added @ frame "
                                : "Frame is already an anchor: ")
                       + std::to_string(frame) + ".  ("
                       + std::to_string((int)refine_anchors_.size()) + " anchor(s))");
            asapUpdate();
            return 1;
        }
        // refine_anchors blob changed — undo/redo or .nk load walked it back.
        // Reload only when the content differs from what we last wrote, so our
        // own (async) set_text echo is ignored (same guard as the pins blob).
        if (k->is("refine_anchors")) {
            if (!suppress_anchor_callback_) {
                const std::string s(refine_anchors_blob_ ? refine_anchors_blob_ : "");
                if (s != refine_anchors_cache_) { load_anchors_from_knob(); refresh_anchor_label(); }
            }
            return 1;
        }
        if (k->is("bake_live_camera")) {
            bake_live_camera();
            set_status("[" + timestamp() +
                       "] Live Camera refreshed from the current solve.\n"
                       "  Link a Camera2's translate/rotate to live_cam_translate "
                       "/ live_cam_rotate (useMatrix off, rot_order ZXY).");
            return 1;
        }
    }
    return NoIop::knob_changed(k);
}


// -----------------------------------------------------------------------------
// toggle_pin_input — flip the pin-refine arm state.
//
// Called from the `P` keydown over the viewer (wireframe_knob.cpp) and kept in
// sync with the "Pin Edit" checkbox by writing the knob. set_value() re-fires
// knob_changed("pin_input_active"), which simply re-reads the (now matching)
// value into pin_input_active_ — idempotent, so no suppression flag is needed.
// Deliberately does NOT touch live_scene_ or any pose state: arming swaps the
// input target, never the pose.
// -----------------------------------------------------------------------------
void PolychaseTracker::toggle_pin_input()
{
    // Flip from the SHARED current state (pin_input_active() reads the knob), so the
    // shift+P hotkey — handled on the viewer instance, whose bound member can lag a
    // panel-side change — toggles relative to what's actually current, keeping the
    // checkbox and the hotkey in sync. Keep the member mirrored for completeness.
    pin_input_active_ = !pin_input_active();
    if (Knob* k = knob("pin_input_active"))
        k->set_value(pin_input_active_ ? 1.0 : 0.0);
    PCN_LOG("[pin] input "
              << (pin_input_active_ ? "ARMED — pins drive the mouse (gizmo inert)"
                                    : "disarmed — gizmo drives the mouse (pins inert)")
              << "\n");
    asapUpdate();
}






// on_track / on_stop / on_export live in tracker_track.cpp.


// Clear Solve wipes the keys/anchor blobs and resets the live solve so the
// overlay falls back to the upstream geo.
void PolychaseTracker::on_clear_solve()
{
    if (Knob* k = knob("keys")) {
        k->set_text("");
    }
    if (Knob* k = knob("anchor_node")) {
        k->set_text("");
    }
    // Also wipe the live solve state so the wireframe overlay
    // jumps back to the upstream geo transform.
    reset_live_solve();

    std::ostringstream oss;
    oss << "[" << timestamp() << "] Clear Solve\n"
        << "  [PASS] keys blob wiped, anchor_node cleared, live solve reset.\n"
        << "  Database is preserved on disk.";
    set_status(oss.str());
}


// on_export() lives in tracker_track.cpp.


void PolychaseTracker::set_status(const std::string& msg)
{
    PCN_LOG("[PolychaseTracker] ");
    for (char c : msg) {
        PCN_LOG(c);
        if (c == '\n') PCN_LOG("[PolychaseTracker] ");
    }
    PCN_LOG(std::endl);

    if (Knob* k = knob("status")) {
        k->set_text(msg.c_str());
    }
}


std::string PolychaseTracker::timestamp()
{
    const auto now = std::chrono::system_clock::now();
    const auto t   = std::chrono::system_clock::to_time_t(now);
    std::tm     tm{};
    localtime_r(&t, &tm);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%H:%M:%S");
    return oss.str();
}

// ---- Plugin registration ----
static Op* build(Node* node) { return new PolychaseTracker(node); }
Op::Description PolychaseTracker::description("PolychaseTracker", build);

} // namespace pcn