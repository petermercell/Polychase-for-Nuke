# Change Log — Polychase core modifications

This file records modifications made to **Polychase** (an open-source motion
tracker by Ahmed Essam <aessam.dahy@gmail.com> and contributors,
https://github.com/theartful/polychase), which is licensed under the
**GNU General Public License v3.0 (GPL-3.0)**.

These modifications are themselves released under the GPL-3.0, as required by
that license. This file is provided to satisfy GPL-3.0 §5(a) ("you must cause
the modified files to carry prominent notices stating that you changed the files
and the date of any change").

- **Modified by:** Peter Mercell <info@petermercell.com>
- **Date of changes:** 2026-06-20
- **Assistance:** changes developed with assistance from Claude (Anthropic).
- **Base:** the unmodified Polychase `cpp/` tree as obtained from the upstream
  project. Only the files listed below were changed; one file was added.

---

## Summary

Two optional, opt-in solve inputs were added to the core tracker and refiner so
a host application can stabilise a 3D track the same way KeenTools GeoTracker
does:

1. **2D occlusion mask ("mask plate").** A per-frame, image-space predicate that
   excludes feature points falling inside a masked region (occluders, props,
   anything that is not the tracked object on that frame).

2. **User tracks ("helper tracks").** Hand-made 2D point trajectories, each
   anchored to a rigid point on the object, added as extra 2D↔3D correspondences
   that stabilise the solve. Tracks whose anchor does not lie on the object are
   simply not supplied by the caller.

Both features work in **Track** (`tracker.cc`) and **Refine** (`refiner.cc`).

**Backward compatibility:** both features are off by default. An empty mask
predicate masks nothing and an empty user-track list adds nothing, so a
default-constructed `TrackerOptions` / `RefinerOptions` produces results that are
bit-for-bit identical to the unmodified library. No public function signature was
removed or repurposed; only new optional fields and (file-local) parameters were
added.

---

## Files added

### `cpp/user_constraints.h`  *(new file)*

New shared, host-agnostic header defining the data types for both features:

- `MaskPredicate` — `std::function<bool(int32_t frame_id, const Eigen::Vector2f& px)>`;
  returns true when an image point should be excluded. Empty ⇒ no masking.
- `UserTrackObservation` — one `{frame_id, image_point}` measurement.
- `UserTrack` — `{object_point (object/local space), weight, observations[]}`.
- `UserTracks` — `std::vector<UserTrack>`.

Convention documented in the header: every image point is in the database
convention (real pixels, Y-down / top-origin, OpenCV).

---

## Files changed

### `cpp/tracker.h`

- Added `#include "user_constraints.h"`.
- Added three optional fields to `struct TrackerOptions`, all defaulting to "off":
  - `MaskPredicate is_masked;`
  - `UserTracks user_tracks;`
  - `float user_track_weight = 1.0f;`

### `cpp/tracker.cc`

- `SolveFrame()` (file-local static): added parameters
  `const MaskPredicate& is_masked`, `const UserTracks& user_tracks`,
  `float user_track_weight`.
- **2D mask:** in the per-correspondence loop, a correspondence is skipped when
  either endpoint is masked on its own frame — the source keypoint on its flow
  frame, or the target keypoint on the frame being solved.
- **User tracks:** after the flow correspondences are gathered, one
  correspondence per track with an observation on the current frame is appended:
  the object-space anchor mapped to world through the (fixed) model matrix, paired
  with the track's measured pixel.
- **Weights:** a PnP weight vector is now built *only* when user tracks
  contribute (flow points at weight 1, user points at `track.weight *
  user_track_weight`); otherwise the weight vector is left empty so the flow-only
  path stays uniform-weighted exactly as before.
- `TrackTrajectory()`: updated the `SolveFrame()` call site to forward
  `opts.is_masked`, `opts.user_tracks`, `opts.user_track_weight`.

### `cpp/refiner.h`

- Added `#include "user_constraints.h"`.
- Added the same three optional fields to `struct RefinerOptions`
  (`is_masked`, `user_tracks`, `user_track_weight = 1.0f`).

### `cpp/refiner.cc`

- Added `#include <map>`.
- **2D mask:** `CachedDatabase` now takes an optional `MaskPredicate` (stored as
  `is_masked_`); `CachedDatabase::FilterKeypoints()` drops keypoints that fall in
  a masked region on their frame, in addition to the existing model-bounding-box
  cull. `RefineTrajectory()` passes `opts.is_masked` into `CachedDatabase`.
- **User tracks (bundle residuals):**
  - `RefinementProblemBase` ctor gained optional params
    `const UserTracks& user_tracks`, `float user_track_weight`, and precomputes:
    per-track world points (object point × fixed model matrix), per-track weights,
    and a list of "user edges" — one per *interior* frame that carries
    observations.
  - New protected members: `UserEdge` struct, `user_world_points`,
    `user_track_weights`, `user_edges`, `user_ref_frame`,
    `user_track_weight_global`.
  - `GlobalRefinementProblem`: `NumEdges`, `NumResiduals`, `EdgeWeight` branch to
    cover user edges; new `GetEdge`, `ResidualWeight`, `Evaluate` overrides; and
    `EvaluateWithJacobian` branches to two new helpers `EvaluateUser` /
    `EvaluateUserWithJacobian`.
  - **Design note:** the sparse Levenberg–Marquardt solver requires every edge to
    connect two *distinct* parameter blocks (`CHECK_NE`). A user-track
    observation constrains only one camera, so each user edge pairs the observed
    interior frame with the segment's first (ground-truth) frame; the partner
    block's Jacobian is left zero, so only the observed camera is constrained. The
    residual is the target-camera-only case of the existing flow residual (the 3D
    point is fixed/rigid), reusing `Pose::ApplyWithJac` and
    `CameraIntrinsics::ProjectWithJac`.
  - `RefineTrajectory()` (template): forwards `opts.user_tracks` and
    `opts.user_track_weight` to `GlobalRefinementProblem`.

---

## Verification

Both translation units were syntax-checked (`g++ -fsyntax-only`) against the
project's vcpkg-provided Eigen / embree / tbb / spdlog headers:
`tracker.cc` (C++17 and C++20) and `refiner.cc` (C++20) both compile without
errors.

---

## GPL-3.0 §5(a) note

In addition to this change log, the GPL-3.0 expects each modified source file to
carry a prominent notice that it was changed and the date. The recommended notice
for the top of each modified file (`tracker.h`, `tracker.cc`, `refiner.h`,
`refiner.cc`) is:

    // Modified 2026-06-20 by Peter Mercell <info@petermercell.com>:
    // added optional 2D occlusion mask and user (helper) track constraints.
    // See CHANGELOG.md. Original work Copyright (c) 2025 Ahmed Essam. GPL-3.0.

`user_constraints.h` is a new file and already carries the GPL-3.0 SPDX header.
