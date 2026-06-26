// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025 Ahmed Essam <aessam.dahy@gmail.com>
//
// Modified 2026-06-20 by Peter Mercell <info@petermercell.com>:
// added optional 2D occlusion mask and user (helper) track constraints
// (see CHANGELOG.md). Original work Copyright (c) 2025 Ahmed Essam. GPL-3.0.

#pragma once

#include <cstdint>
#include <functional>
#include <string>

#include "camera_trajectory.h"
#include "database.h"
#include "geometry.h"
#include "pnp/solvers.h"
#include "ray_casting.h"
#include "user_constraints.h"

struct TrackerOptions {
    PnPOptions pnp_opts;
    int32_t frame_from;
    int32_t frame_to_inclusive;

    // Optional, opt-in solve inputs (see user_constraints.h). Both default to
    // "off": an empty predicate masks nothing, an empty vector adds nothing, so
    // a default-constructed TrackerOptions tracks exactly as before.
    MaskPredicate is_masked;          // 2D occlusion mask (per-frame)
    UserTracks user_tracks;           // helper tracks (extra correspondences)
    float user_track_weight = 1.0f;   // global multiplier on user-track weight
};

struct TrackerUpdate {
    int32_t frame;
    PnPResult pnp_result;
};

using TrackerCallback = std::function<bool(const TrackerUpdate&)>;

// TODO: Drop this function in favor of TrackTrajectory
void TrackSequence(const std::string& database_path,
                   const SceneTransformations& scene_transform,
                   const AcceleratedMesh& accel_mesh, TrackerCallback callback,
                   const TrackerOptions& opts);

void TrackTrajectory(const Database& database, CameraTrajectory& camera_traj,
                     const RowMajorMatrix4f& model_matrix,
                     const AcceleratedMesh& accel_mesh,
                     TrackerCallback callback, const TrackerOptions& opts);
