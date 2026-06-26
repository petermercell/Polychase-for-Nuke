// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025 Ahmed Essam <aessam.dahy@gmail.com>

#include "tracker.h"

#include <spdlog/spdlog.h>

#include <Eigen/Eigen>
#include <Eigen/LU>
#include <vector>

#include "camera_trajectory.h"
#include "database.h"
#include "pnp/solvers.h"
#include "pnp/types.h"
#include "ray_casting.h"

struct SolveFrameCache {
    std::vector<Eigen::Vector3f> object_points_worldspace;
    std::vector<Eigen::Vector2f> image_points;
    std::vector<float> weights;
    std::vector<int32_t> flow_frames_ids;
    Keypoints keypoints;
    ImagePairFlow flow;

    void Clear() {
        object_points_worldspace.clear();
        image_points.clear();
        weights.clear();
        flow_frames_ids.clear();
        keypoints.clear();
        flow.Clear();
    }
};

static std::optional<PnPResult> SolveFrame(const Database& database,
                                           const CameraTrajectory& camera_traj,
                                           const RowMajorMatrix4f& model_matrix,
                                           const int32_t frame_id,
                                           const AcceleratedMesh& accel_mesh,
                                           const PnPOptions& pnp_opts,
                                           SolveFrameCache& cache) {
    cache.Clear();
    database.FindOpticalFlowsToImage(frame_id, cache.flow_frames_ids);

    for (int32_t flow_frame_id : cache.flow_frames_ids) {
        CHECK_NE(flow_frame_id, frame_id);

        if (!camera_traj.IsFrameFilled(flow_frame_id)) {
            continue;
        }

        database.ReadKeypoints(flow_frame_id, cache.keypoints);
        database.ReadImagePairFlow(flow_frame_id, frame_id, cache.flow);

        CHECK_EQ(cache.flow.src_kps_indices.size(), cache.flow.tgt_kps.size());
        const size_t num_matches = cache.flow.src_kps_indices.size();

        const std::optional<CameraState>& maybe_camera_state =
            camera_traj.Get(flow_frame_id);
        CHECK(maybe_camera_state.has_value());
        const CameraState& camera_state = *maybe_camera_state;

        // TODO: benchmark / vectorize / parallelize
        for (size_t i = 0; i < num_matches; i++) {
            const uint32_t kp_idx = cache.flow.src_kps_indices[i];
            const Eigen::Vector2f& kp = cache.keypoints[kp_idx];
            const Eigen::Vector2f& tgt_kp = cache.flow.tgt_kps[i];

            const SceneTransformations scene_transform = {
                .model_matrix = model_matrix,
                .view_matrix = camera_state.pose.Rt4x4(),
                .intrinsics = camera_state.intrinsics,
            };
            // Mabye collect rays, and bulk RayCast using embrees optimized
            // rtcIntersect4/8/16
            const std::optional<RayHit> hit =
                RayCast(accel_mesh, scene_transform, kp, true);

            if (hit) {
                const Eigen::Vector3f intersection_point_worldspace =
                    model_matrix.block<3, 3>(0, 0) * hit->pos +
                    model_matrix.block<3, 1>(0, 3);

                cache.object_points_worldspace.push_back(
                    intersection_point_worldspace);
                cache.image_points.push_back(tgt_kp);
#if 0
                cache.weights.push_back(
                    std::min(1.0 / cache.flow.flow_errors[i], 1e3));
#endif
            }
        }
    }

    if (cache.object_points_worldspace.size() < 3) {
        return std::nullopt;
    }

    const Eigen::Map<const RowMajorMatrixX3f> object_points_eigen{
        reinterpret_cast<const float*>(cache.object_points_worldspace.data()),
        static_cast<Eigen::Index>(cache.object_points_worldspace.size()), 3};

    const Eigen::Map<const RowMajorMatrixX2f> image_points_eigen{
        reinterpret_cast<const float*>(cache.image_points.data()),
        static_cast<Eigen::Index>(cache.image_points.size()), 2};

    const Eigen::Map<const Eigen::ArrayXf> weights_eigen{
        reinterpret_cast<const float*>(cache.weights.data()),
        static_cast<Eigen::Index>(cache.weights.size()), 1};

    PnPResult result;
    // The solution should be very close to the previous/next pose
    if (camera_traj.IsFrameFilled(frame_id)) {
        result.camera = *camera_traj.Get(frame_id);
    } else if (camera_traj.IsFrameFilled(frame_id - 1)) {
        result.camera = *camera_traj.Get(frame_id - 1);
    } else if (camera_traj.IsFrameFilled(frame_id + 1)) {
        result.camera = *camera_traj.Get(frame_id + 1);
    }

    SolvePnPIterative(object_points_eigen, image_points_eigen, weights_eigen,
                      pnp_opts, result);
    return result;
}

void TrackTrajectory(const Database& database, CameraTrajectory& camera_traj,
                     const RowMajorMatrix4f& model_matrix,
                     const AcceleratedMesh& accel_mesh,
                     TrackerCallback callback, const TrackerOptions& opts) {
    SPDLOG_INFO("Tracking from frame #{} to frame #{}", opts.frame_from,
                opts.frame_to_inclusive);

    // Sanity checks
    CHECK(camera_traj.IsFrameFilled(opts.frame_from));
    CHECK(camera_traj.IsValidFrame(opts.frame_to_inclusive));

    // To avoid unnecessary re-alloactions
    SolveFrameCache cache;

    const int32_t dir = (opts.frame_from < opts.frame_to_inclusive) ? 1 : -1;

    for (int32_t frame_id = opts.frame_from + dir;
         frame_id != opts.frame_to_inclusive + dir; frame_id += dir) {
        SPDLOG_DEBUG("Tracking frame {}", frame_id);

        const std::optional<PnPResult> maybe_result =
            SolveFrame(database, camera_traj, model_matrix, frame_id,
                       accel_mesh, opts.pnp_opts, cache);

        if (!maybe_result) {
            throw std::runtime_error(fmt::format(
                "Could not track to frame: {}. Not enough features.",
                frame_id));
        }

        const PnPResult& pnp_result = *maybe_result;

        if (callback) {
            const bool ok = callback({frame_id, pnp_result});
            if (!ok) {
                SPDLOG_INFO("User requested to stop at frame {}", frame_id);
                return;
            }
        }

        camera_traj.Set(frame_id, pnp_result.camera);
    }

    SPDLOG_INFO("Tracking finished successfuly from frame #{} to frame #{}",
                opts.frame_from, opts.frame_to_inclusive);
    return;
}

void TrackSequence(const std::string& database_path,
                   const SceneTransformations& scene_transform,
                   const AcceleratedMesh& accel_mesh, TrackerCallback callback,
                   const TrackerOptions& opts) {
    const Database database{database_path};

    const size_t num_frames =
        std::abs(opts.frame_to_inclusive - opts.frame_from) + 1;

    CameraTrajectory camera_traj{
        std::min(opts.frame_from, opts.frame_to_inclusive), num_frames};

    camera_traj.Set(opts.frame_from,
                    CameraState{scene_transform.intrinsics,
                                Pose::FromRt(scene_transform.view_matrix)});

    TrackTrajectory(database, camera_traj, scene_transform.model_matrix,
                    accel_mesh, callback, opts);
}
