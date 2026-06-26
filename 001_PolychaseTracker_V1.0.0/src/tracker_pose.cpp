// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Peter Mercell
//
// Developed with assistance from Claude (Anthropic).

// tracker_pose.cpp — part of the PolychaseTracker plugin (see polychase_tracker.h).
#include "polychase_tracker.h"

#include <iostream>

using namespace DD::Image;

namespace pcn {


// -----------------------------------------------------------------------------
// effective_model_matrix — used by the WireframeKnob's draw_handle to choose
// between the solved pose and the upstream default. Centralizes the lookup
// so callers don't have to know about live_scene_'s existence.
// -----------------------------------------------------------------------------
DD::Image::Matrix4 PolychaseTracker::effective_model_matrix(
    const DD::Image::Matrix4& upstream_default,
    double frame) const
{
    // Default (no explicit frame) means an interaction caller — use the UI frame
    // the knob tracks, not the lagging Op context. The draw path always passes an
    // explicit uiContext frame, so it's unaffected.
    const double f = (frame <= kNoFrame * 0.1) ? (double)editing_frame() : frame;

    // Any uncommitted edit — a gizmo translate/dolly preview OR a rotation-offset
    // preview — lives in live_scene_, and it wins ONLY at the frame it was staged
    // at (live_edit_frame_). That keeps the preview visible at the frame being
    // manipulated while letting every OTHER frame fall through to the keyed pose,
    // so a stranded live_scene_ (e.g. an edit never committed with Set Pose Key)
    // can't freeze the whole timeline on one stale pose. live_scene_ is cleared
    // on commit (Set Pose Key) and on discard (Clear Solve / pin reset / .nk load).
    if (live_scene_) {
        const double df = f - (double)live_edit_frame_;
        if (df > -0.5 && df < 0.5) {
            return eigen_to_nuke_m4(live_scene_->model_matrix);
        }
    }
    // No uncommitted edit: prefer the keyframed pose, evaluated at the frame.
    if (has_pose_keys()) {
        return pose_matrix_to_nuke(f);
    }
    return upstream_default;
}


// Build a Nuke Matrix4 from the animated translate/rotate/scale curves
// evaluated at `frame`. Reading get_value_at (not the cached knob storage)
// makes the overlay animate correctly even right after a runtime key edit.
DD::Image::Matrix4 PolychaseTracker::pose_matrix_to_nuke(double frame) const
{
    double t[3], r[3], s[3];
    auto eval = [&](const char* name, const double fallback[3], double out[3]) {
        DD::Image::Knob* k = knob(name);
        if (k && k->is_animated()) {
            for (int i = 0; i < 3; ++i) out[i] = k->get_value_at(frame, i);
        } else {
            for (int i = 0; i < 3; ++i) out[i] = fallback[i];
        }
    };
    eval("pose_translate", pose_t_, t);
    eval("pose_rotate",    pose_r_, r);
    eval("pose_scale",     pose_s_, s);
    return eigen_to_nuke_m4(compose_trs(t, r, s));
}


bool PolychaseTracker::has_pose_keys() const
{
    // knob() is callable from a const method; is_animated() is non-const, so
    // hold a non-const Knob*.
    DD::Image::Knob* kt = knob("pose_translate");
    DD::Image::Knob* kr = knob("pose_rotate");
    DD::Image::Knob* ks = knob("pose_scale");
    return (kt && kt->is_animated()) ||
           (kr && kr->is_animated()) ||
           (ks && ks->is_animated());
}


// Key the current solved pose (live_scene_->model_matrix) at the current frame.
// set_value_at on each channel creates/updates the keyframe, so re-keying the
// same frame overwrites it.
void PolychaseTracker::key_pose_at_current_frame()
{
    if (!live_scene_) {
        PCN_LOG("[pose] no solved pose to key\n");
        return;
    }
    // Final safety net before a pose hits the persisted keyframe curve. The pin
    // path already rejects non-finite solves upstream, but the gizmo/rotation
    // paths also commit through here — so refuse a NaN/Inf pose unconditionally
    // and discard the bad edit, reverting the overlay to the last good keyed
    // pose. Keying garbage here is what permanently poisons the curve.
    if (!live_scene_->model_matrix.allFinite()) {
        PCN_LOG("[pose] refused to key a non-finite pose; discarding the "
                     "bad edit and reverting to the last good pose\n");
        live_scene_.reset();
        dragging_pin_idx_ = -1;
        drag_anchor_.reset();
        rot_base_.reset();
        rot_base_had_live_ = false;
        trans_base_.reset();
        trans_base_had_live_ = false;
        asapUpdate();
        return;
    }
    double t[3], r[3], s[3];
    decompose_trs(live_scene_->model_matrix, t, r, s);

    // Use the displayed (UI) frame, not outputContext() which lags during scrub.
    // A pin/gizmo drag released on frame 45 must key on 45, not the stale frame.
    const double frame = (double)editing_frame();
    DD::Image::Knob* kt = knob("pose_translate");
    DD::Image::Knob* kr = knob("pose_rotate");
    DD::Image::Knob* ks = knob("pose_scale");

    // Key one XYZ knob: ensure each channel has an animation curve first, then
    // write the keyframe. set_value_at on a still-static knob sets the value but
    // doesn't reliably create the curve, which is why keying used to require
    // manually enabling animation on translate/rotate beforehand. set_animated()
    // creates the curve on the first key and is a no-op (preserving existing
    // keys) once the channel is already animated.
    auto key_xyz = [&](DD::Image::Knob* k, const double v[3]) {
        if (!k) return;
        for (int i = 0; i < 3; ++i) {
            if (!k->is_animated(i)) k->set_animated(i);
            k->set_value_at(v[i], frame, i);
        }
        k->changed();
    };
    key_xyz(kt, t);
    key_xyz(kr, r);
    key_xyz(ks, s);
    PCN_LOG("[pose] keyed pose @ frame " << frame << "\n");

    // A hand correction = a Refine boundary (the Blender KEYFRAME mental model).
    // Record THIS frame as a fixed anchor so Refine re-solves only the gaps
    // around it and holds this pose unchanged. Auto — no extra clicks.
    add_refine_anchor((int)frame);

    // Keep the Live Camera curve in sync for this frame (uses live_scene_, which
    // is still set here) so any expression-linked Camera2 follows each refine.
    update_live_camera_key_current();

    // Commit the key, but KEEP the working pose (live_scene_) alive, pinned to
    // THIS frame. Two reasons:
    //   1. Undo. The keyframe write is on the undo stack; live_scene_ is plain C++
    //      state that undo can't touch. If we cleared it here, a later Ctrl+Z
    //      would revert the curve and — with no live pose left — the overlay would
    //      fall back to the upstream geo (the rest/"hero" pose), snapping the
    //      wireframe away from where you posed it. Keeping it means undoing the key
    //      leaves the wireframe exactly where it was: the visual stays put, only
    //      the persisted key is removed (which is what Set Pose + undo should do).
    //   2. It costs nothing elsewhere: effective_model_matrix only lets live_scene_
    //      win at live_edit_frame_ (this frame), so every OTHER frame still reads
    //      the keyed curve. The preview is cleared by Clear Solve, a pin reset,
    //      .nk load, Track (re-bakes the curve) and Clear Pose Keys.
    // The frozen rotation base and drag bookkeeping are still dropped — only the
    // pose preview itself is preserved.
    live_edit_frame_  = (int)frame;   // ensure the kept preview wins at this frame
    dragging_pin_idx_ = -1;
    drag_anchor_.reset();
    rot_base_.reset();
    rot_base_had_live_ = false;
    trans_base_.reset();
    trans_base_had_live_ = false;
    asapUpdate();
}


// Remove all pose animation (every channel, every frame). For deleting a
// single keyframe, use the Dope Sheet / Curve Editor.
void PolychaseTracker::clear_pose_keys()
{
    const char* names[3] = {"pose_translate", "pose_rotate", "pose_scale"};
    for (const char* n : names) {
        if (DD::Image::Knob* k = knob(n)) {
            k->clear_animated();
            k->changed();
        }
    }
    // The Live Camera curves are derived from the pose — clear them too.
    for (const char* n : {"live_cam_translate", "live_cam_rotate"}) {
        if (DD::Image::Knob* k = knob(n)) { k->clear_animated(); k->changed(); }
    }
    // Reset to the neutral pose so the overlay falls back to the upstream geo.
    pose_t_[0]=pose_t_[1]=pose_t_[2]=0.0;
    pose_r_[0]=pose_r_[1]=pose_r_[2]=0.0;
    pose_s_[0]=pose_s_[1]=pose_s_[2]=1.0;

    // Drop any kept working pose too, otherwise a live_scene_ left over from the
    // last Set Pose / drag would keep masking the now-empty curve at its frame and
    // the overlay wouldn't actually fall back to the upstream geo.
    live_scene_.reset();
    rot_base_.reset();
    rot_base_had_live_ = false;
    trans_base_.reset();
    trans_base_had_live_ = false;
    sync_blob_from_live_pose();   // keep the undoable gizmo-pose mirror in step

    // A wiped pose curve has no valid corrections, so its anchors are stale too.
    // Silent: clear_pose_keys owns the status line below.
    reset_refine_anchors_silent();

    PCN_LOG("[pose] cleared all pose keys\n");
    asapUpdate();
}

} // namespace pcn
