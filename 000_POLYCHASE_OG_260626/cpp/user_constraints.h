// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Peter Mercell <info@petermercell.com>
//
// New file added to Polychase (original work Copyright (c) 2025 Ahmed Essam,
// GPL-3.0). Developed with assistance from Claude (Anthropic).
//
// user_constraints.h — optional, opt-in solve inputs shared by the tracker and
// the refiner:
//
//   1. MaskPredicate  — a 2D occlusion mask. A per-frame test that returns true
//      when an image point should be EXCLUDED from the solve (an occluder, a
//      prop, anything that is not the tracked object on that frame). This is the
//      "Masking 2D / mask plate" workflow.
//
//   2. UserTracks     — "helper" tracks made with a 2D point tracker (e.g. the
//      host's Tracker node). Each is a single rigid point ON the object plus its
//      measured 2D position per frame; they add extra 2D<->3D correspondences
//      that stabilise the solve. This is the "User Tracks" workflow.
//
// BOTH are optional and default to "off": an empty MaskPredicate masks nothing,
// and an empty UserTracks vector adds nothing. With both unset, Track and Refine
// behave exactly as before — bit-for-bit — so existing callers are unaffected.
//
// CONVENTION: every image point here (mask test position, track observation)
// is in the SAME convention as the database keypoints: real pixels, Y-DOWN
// (top-origin), OpenCV. The caller is responsible for any flip from its own
// image convention before handing data in.
#pragma once

#include <cstdint>
#include <functional>
#include <vector>

#include <Eigen/Core>

// Returns true when the image point `px` on frame `frame_id` falls in a masked
// (excluded) region. Empty std::function => nothing is masked (the fast path).
// `px` is in database convention (real pixels, Y-down). The predicate should
// return false for frames outside its known range rather than throwing.
using MaskPredicate =
    std::function<bool(int32_t frame_id, const Eigen::Vector2f& px)>;

// One measured 2D position of a user track on a specific frame.
struct UserTrackObservation {
    int32_t frame_id;
    Eigen::Vector2f image_point;  // real pixels, Y-down (database convention)
};

// One user (helper) track: a rigid point on the object plus its per-frame 2D
// observations. `object_point` is in OBJECT / LOCAL space (the same space as the
// mesh vertices); the solver maps it to world via the fixed model matrix, just
// like a ray-cast flow point. The caller computes this anchor once (e.g. by
// ray-casting the track's position on a reference frame onto the mesh); a track
// whose reference ray misses the mesh is "not on the object" and should simply
// not be added.
struct UserTrack {
    Eigen::Vector3f object_point;
    float weight = 1.0f;  // per-track relative weight in the solve
    std::vector<UserTrackObservation> observations;
};

using UserTracks = std::vector<UserTrack>;
