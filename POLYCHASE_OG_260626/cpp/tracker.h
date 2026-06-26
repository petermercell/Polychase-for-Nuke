// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025 Ahmed Essam <aessam.dahy@gmail.com>

#pragma once

#include <cstdint>
#include <functional>
#include <string>

#include "camera_trajectory.h"
#include "database.h"
#include "geometry.h"
#include "pnp/solvers.h"
#include "ray_casting.h"

struct TrackerOptions {
    PnPOptions pnp_opts;
    int32_t frame_from;
    int32_t frame_to_inclusive;
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
