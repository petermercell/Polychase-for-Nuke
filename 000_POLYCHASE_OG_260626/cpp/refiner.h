// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025 Ahmed Essam <aessam.dahy@gmail.com>
//
// Modified 2026-06-20 by Peter Mercell <info@petermercell.com>:
// added optional 2D occlusion mask and user (helper) track constraints
// (see CHANGELOG.md). Original work Copyright (c) 2025 Ahmed Essam. GPL-3.0.

#pragma once

#include <functional>
#include <string>

#include "camera_trajectory.h"
#include "eigen_typedefs.h"
#include "ray_casting.h"
#include "user_constraints.h"

class Database;

struct RefinerOptions {
    BundleOptions bundle_opts;
    Float min_fov_deg = 15.0;
    Float max_fov_deg = 160.0;
    bool optimize_focal_length = false;
    bool optimize_principal_point = false;

    // Optional, opt-in solve inputs (see user_constraints.h). Both default to
    // "off": an empty predicate masks nothing, an empty vector adds nothing, so
    // a default-constructed RefinerOptions refines exactly as before.
    MaskPredicate is_masked;          // 2D occlusion mask (per-frame)
    UserTracks user_tracks;           // helper tracks (extra residuals)
    float user_track_weight = 1.0f;   // global multiplier on user-track weight
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
