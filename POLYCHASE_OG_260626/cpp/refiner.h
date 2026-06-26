// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025 Ahmed Essam <aessam.dahy@gmail.com>

#pragma once

#include <functional>
#include <string>

#include "camera_trajectory.h"
#include "eigen_typedefs.h"
#include "ray_casting.h"

class Database;

struct RefinerOptions {
    BundleOptions bundle_opts;
    Float min_fov_deg = 15.0;
    Float max_fov_deg = 160.0;
    bool optimize_focal_length = false;
    bool optimize_principal_point = false;
};

struct RefinerUpdate {
    float progress;
    std::string message;
    BundleStats stats;
};

using RefinerCallback = std::function<bool(RefinerUpdate)>;

void RefineTrajectory(const std::string& database_path, CameraTrajectory& traj,
                      const RowMajorMatrix4f& model_matrix,
                      const AcceleratedMesh& accel_mesh,
                      RefinerCallback callback, const RefinerOptions& opts);

void RefineTrajectory(const Database& database, CameraTrajectory& traj,
                      const RowMajorMatrix4f& model_matrix,
                      const AcceleratedMesh& accel_mesh,
                      RefinerCallback callback, const RefinerOptions& opts);
