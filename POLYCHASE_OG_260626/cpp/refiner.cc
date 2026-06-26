// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025 Ahmed Essam <aessam.dahy@gmail.com>
//
// Modified 2026-06-20 by Peter Mercell <info@petermercell.com>:
// added optional 2D occlusion mask and user (helper) track constraints
// (see CHANGELOG.md). Original work Copyright (c) 2025 Ahmed Essam. GPL-3.0.

#include "refiner.h"

#include <fmt/chrono.h>
#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <Eigen/SparseCore>
#include <atomic>
#include <map>

#include "database.h"
#include "pnp/lev_marq.h"
#include "pnp/quaternion.h"
#include "pnp/robust_loss.h"

static Bbox2 TransformBbox(const Bbox3& bbox,
                           const RowMajorMatrix4f& transform) {
    const std::array<Eigen::Vector3f, 8> points = {
        Eigen::Vector3f{bbox.pmin.x(), bbox.pmin.y(), bbox.pmin.z()},  // 0 0 0
        Eigen::Vector3f{bbox.pmin.x(), bbox.pmin.y(), bbox.pmax.z()},  // 0 0 1
        Eigen::Vector3f{bbox.pmin.x(), bbox.pmax.y(), bbox.pmin.z()},  // 0 1 0
        Eigen::Vector3f{bbox.pmin.x(), bbox.pmax.y(), bbox.pmax.z()},  // 0 1 1
        Eigen::Vector3f{bbox.pmax.x(), bbox.pmin.y(), bbox.pmin.z()},  // 1 0 0
        Eigen::Vector3f{bbox.pmax.x(), bbox.pmin.y(), bbox.pmax.z()},  // 1 0 1
        Eigen::Vector3f{bbox.pmax.x(), bbox.pmax.y(), bbox.pmin.z()},  // 1 1 0
        Eigen::Vector3f{bbox.pmax.x(), bbox.pmax.y(), bbox.pmax.z()},  // 1 1 1
    };

    Eigen::Vector2f transformed_pmin{std::numeric_limits<float>::max(),
                                     std::numeric_limits<float>::max()};
    Eigen::Vector2f transformed_pmax{std::numeric_limits<float>::lowest(),
                                     std::numeric_limits<float>::lowest()};

    for (const auto& point : points) {
        Eigen::Vector3f transformed =
            (transform * point.homogeneous()).hnormalized();

        transformed_pmin.x() = std::min(transformed.x(), transformed_pmin.x());
        transformed_pmax.x() = std::max(transformed.x(), transformed_pmax.x());

        transformed_pmin.y() = std::min(transformed.y(), transformed_pmin.y());
        transformed_pmax.y() = std::max(transformed.y(), transformed_pmax.y());
    }

    return Bbox2{
        .pmin = transformed_pmin,
        .pmax = transformed_pmax,
    };
}

static Bbox2 ComputeBbox(const CameraTrajectory& traj, const Mesh& mesh,
                         const RowMajorMatrix4f& model_matrix,
                         int32_t frame_id) {
    CHECK(traj.IsFrameFilled(frame_id));
    const CameraState& state = *traj.Get(frame_id);
    const RowMajorMatrix4f mvp = state.intrinsics.To4x4ProjectionMatrix() *
                                 state.pose.Rt4x4() * model_matrix;

    Bbox2 bbox = TransformBbox(mesh.bbox, mvp);

    // Add padding
    constexpr float kPadding = 20.0f;
    bbox.pmin -= Eigen::Vector2f(kPadding, kPadding);
    bbox.pmax += Eigen::Vector2f(kPadding, kPadding);

    return bbox;
}

class CachedDatabase {
   public:
    CachedDatabase(const Database& database, const CameraTrajectory& traj,
                   const Mesh& mesh, const RowMajorMatrix4f& model_matrix,
                   const MaskPredicate& is_masked = {})
        : first_frame_id(traj.FirstFrame()),
          num_frames(traj.Count()),
          is_masked_(is_masked) {
        LoadDatabase(database, traj, mesh, model_matrix);
    }

    inline size_t NumFrames() const { return num_frames; }

    inline size_t NumFlows() const { return flows.size(); }

    inline int32_t FirstFrame() const { return first_frame_id; }

    inline int32_t LastFrame() const { return first_frame_id + num_frames - 1; }

    const Keypoints& ReadKeypoints(int32_t frame_id) const {
        CHECK_GE(frame_id, FirstFrame());
        CHECK_LE(frame_id, LastFrame());

        return frames_keypoints[frame_id - first_frame_id];
    }

    const ImagePairFlow& ReadFlow(size_t idx) const { return flows[idx]; }

   private:
    void LoadDatabase(const Database& database, const CameraTrajectory& traj,
                      const Mesh& mesh, const RowMajorMatrix4f& model_matrix) {
        SPDLOG_DEBUG("Loading database into memory...");

        frames_keypoints.resize(num_frames);
        flows.reserve(num_frames * 8);

        std::vector<int32_t> ids;
        std::vector<size_t> to_filtered_idx;

        for (int32_t id = traj.FirstFrame(); id <= traj.LastFrame(); id++) {
            LoadFrameFlows(database, traj, mesh, model_matrix, id, ids,
                           to_filtered_idx);
        }

        SPDLOG_DEBUG("Done loading database into memory...");
    }
    void LoadFrameFlows(const Database& database, const CameraTrajectory& traj,
                        const Mesh& mesh, const RowMajorMatrix4f& model_matrix,
                        int32_t frame_id, std::vector<int32_t>& ids,
                        std::vector<size_t>& to_filtered_idx) {
        CHECK_EQ(frames_keypoints.size(), traj.Count());
        CHECK(traj.IsValidFrame(frame_id));

        ids.clear();
        to_filtered_idx.clear();

        Keypoints& keypoints = frames_keypoints[frame_id - first_frame_id];
        database.ReadKeypoints(frame_id, keypoints);

        FilterKeypoints(traj, mesh, model_matrix, frame_id, keypoints,
                        to_filtered_idx);

        database.FindOpticalFlowsFromImage(frame_id, ids);

        for (int32_t frame_id_to : ids) {
            if (!traj.IsValidFrame(frame_id_to)) {
                continue;
            }

            ImagePairFlow flow;
            database.ReadImagePairFlow(frame_id, frame_id_to, flow);

            // Remap flow data to the filtered keypoints
            size_t k = 0;
            for (size_t j = 0; j < flow.tgt_kps.size(); j++) {
                const size_t src_kp_idx =
                    to_filtered_idx[flow.src_kps_indices[j]];

                if (src_kp_idx != kInvalidIdx) {
                    flow.src_kps_indices[k] = src_kp_idx;
                    flow.tgt_kps[k] = flow.tgt_kps[j];
                    flow.flow_errors[k] = flow.flow_errors[j];
                    k++;
                }
            }
            if (k != 0) {
                flow.src_kps_indices.resize(k);
                flow.tgt_kps.resize(k);
                flow.flow_errors.resize(k);

                flows.push_back(std::move(flow));
            }
        }
    }

    void FilterKeypoints(const CameraTrajectory& traj, const Mesh& mesh,
                         const RowMajorMatrix4f& model_matrix, int32_t frame_id,
                         Keypoints& keypoints,
                         std::vector<size_t>& to_filtered_idx) {
        // Remove all keypoints that are not in the bounding box of the model
        // And store the permutation done in "to_filtered_idx", which maps the
        // old index of a keypoint, to its new index in the mapped keypoints.
        const Bbox2 bbox_imagespace =
            ComputeBbox(traj, mesh, model_matrix, frame_id);

        to_filtered_idx.resize(keypoints.size());

        size_t k = 0;
        for (size_t j = 0; j < keypoints.size(); j++) {
            // 2D occlusion mask: drop keypoints that fall in a masked region on
            // this frame, in addition to the existing model-bbox cull. Empty
            // predicate => nothing is masked. Killing the keypoint at its source
            // removes it from every flow it participates in (both roles).
            const bool masked_2d =
                is_masked_ && is_masked_(frame_id, keypoints[j]);
            if (bbox_imagespace.Contains(keypoints[j]) && !masked_2d) {
                keypoints[k] = keypoints[j];
                to_filtered_idx[j] = k;

                k++;
            } else {
                to_filtered_idx[j] = kInvalidIdx;
            }
        }
        keypoints.resize(k);
    }

   private:
    static constexpr size_t kInvalidIdx = std::numeric_limits<size_t>::max();

    int32_t first_frame_id;
    size_t num_frames;
    MaskPredicate is_masked_;

    std::vector<Keypoints> frames_keypoints;
    std::vector<ImagePairFlow> flows;
};

class RefinementProblemBase {
   public:
    using Parameters = CameraTrajectory;

    static constexpr int kResidualLength = 2;
    static constexpr bool kShouldNormalize = true;

    RefinementProblemBase(const CachedDatabase& cached_database,
                          const AcceleratedMesh& mesh,
                          const RowMajorMatrix4f& model_matrix,
                          const CameraTrajectory& initial_camera_traj,
                          bool optimize_focal_length,
                          bool optimize_principal_point,
                          const CameraIntrinsics::Bounds& bounds,
                          const UserTracks& user_tracks = {},
                          float user_track_weight = 1.0f)
        : cached_database(cached_database),
          mesh(mesh),
          model_matrix(model_matrix),
          model_matrix_inv(model_matrix.inverse()),
          optimize_focal_length(optimize_focal_length),
          optimize_principal_point(optimize_principal_point),
          bounds(bounds) {
        // Sanity checks
        CHECK_EQ(cached_database.NumFrames(), initial_camera_traj.Count());
        CHECK_EQ(cached_database.FirstFrame(),
                 initial_camera_traj.FirstFrame());
        CHECK_EQ(cached_database.LastFrame(), initial_camera_traj.LastFrame());

        // 6 parameters for extrinsics, and potentially 3 for intrinsics.
        if (optimize_focal_length || optimize_principal_point) {
            num_params_per_camera = 9;
        } else {
            num_params_per_camera = 6;
        }

        intersections.reserve(cached_database.NumFrames());
        for (int32_t frame = cached_database.FirstFrame();
             frame <= cached_database.LastFrame(); frame++) {
            const size_t num_keypoints =
                cached_database.ReadKeypoints(frame).size();
            intersections.emplace_back(num_keypoints, kInvalidIndex);

            CHECK_EQ(intersections.back().size(), num_keypoints);
        }
        CHECK_EQ(intersections.size(), cached_database.NumFrames());

        // --- precompute user (helper) track constraints ---
        // Each track's object-space anchor -> world via the FIXED model matrix
        // (exactly how flow points are placed). Observations on INTERIOR frames
        // become extra residuals. Each such frame becomes a "user edge" paired
        // with the (ground-truth) first frame purely to satisfy the sparse
        // solver's "an edge connects two distinct param blocks" rule — the
        // partner (first) block gets a zero Jacobian, so only the observed
        // interior camera is actually constrained.
        user_track_weight_global = user_track_weight;
        user_ref_frame = initial_camera_traj.FirstFrame();
        user_world_points.reserve(user_tracks.size());
        user_track_weights.reserve(user_tracks.size());
        for (const UserTrack& tr : user_tracks) {
            user_world_points.push_back(model_matrix.block<3, 3>(0, 0) *
                                            tr.object_point +
                                        model_matrix.block<3, 1>(0, 3));
            user_track_weights.push_back(tr.weight);
        }
        {
            const int32_t first = initial_camera_traj.FirstFrame();
            const int32_t last = initial_camera_traj.LastFrame();
            std::map<int32_t, std::vector<std::pair<uint32_t, Eigen::Vector2f>>>
                by_frame;
            for (uint32_t ti = 0; ti < user_tracks.size(); ti++) {
                for (const UserTrackObservation& ob :
                     user_tracks[ti].observations) {
                    // Interior frames only: first/last are frozen ground truth.
                    if (ob.frame_id <= first || ob.frame_id >= last) {
                        continue;
                    }
                    if (!initial_camera_traj.IsValidFrame(ob.frame_id)) {
                        continue;
                    }
                    by_frame[ob.frame_id].push_back({ti, ob.image_point});
                }
            }
            user_edges.reserve(by_frame.size());
            for (auto& kv : by_frame) {
                user_edges.push_back(UserEdge{kv.first, std::move(kv.second)});
            }
        }
    }

    std::pair<size_t, size_t> GetEdge(size_t edge_idx) const {
        const ImagePairFlow& flow = cached_database.ReadFlow(edge_idx);
        return {FrameIdx(flow.image_id_from), FrameIdx(flow.image_id_to)};
    }

    Float FrameWeight(int32_t frame_id) const {
        const int32_t first_frame_id = cached_database.FirstFrame();
        const int32_t last_frame_id = cached_database.LastFrame();
        const int32_t distance =
            std::min(frame_id - first_frame_id, last_frame_id - frame_id);

        return Float(1.0) / (distance + Float(1.0));
    }

    Float ResidualWeight([[maybe_unused]] size_t edge_idx,
                         [[maybe_unused]] size_t res_idx) const {
#if 0
        const ImagePairFlow& flow = cached_database.ReadFlow(edge_idx);
        return std::min(1.0 / flow.flow_errors[res_idx], 1e3);
#else
        return 1.0f;
#endif
    }

    bool IsGroundTruth(int32_t frame_id) const {
        return frame_id == cached_database.FirstFrame() ||
               frame_id == cached_database.LastFrame();
    }

    std::optional<Eigen::Vector2f> Evaluate(const CameraTrajectory& traj,
                                            size_t flow_idx,
                                            size_t kp_idx) const {
        // Sanity checks
        CHECK_EQ(cached_database.NumFrames(), traj.Count());
        CHECK_EQ(cached_database.FirstFrame(), traj.FirstFrame());
        CHECK_EQ(cached_database.LastFrame(), traj.LastFrame());

        const ImagePairFlow& flow = cached_database.ReadFlow(flow_idx);
        const int32_t src_frame = flow.image_id_from;
        const int32_t tgt_frame = flow.image_id_to;

        const Keypoints& src_kps = cached_database.ReadKeypoints(src_frame);
        const KeypointsIndices& src_kps_indices = flow.src_kps_indices;
        const Keypoints& tgt_kps = flow.tgt_kps;

        const auto& maybe_src_camera = traj.Get(src_frame);
        CHECK(maybe_src_camera.has_value());
        const auto& maybe_tgt_camera = traj.Get(tgt_frame);
        CHECK(maybe_tgt_camera.has_value());

        const CameraState& src_camera = *maybe_src_camera;
        const CameraState& tgt_camera = *maybe_tgt_camera;

        const uint32_t src_kp_idx = src_kps_indices[kp_idx];

        CHECK_LT(kp_idx, src_kps_indices.size());
        CHECK_LT(src_kp_idx, src_kps.size());
        CHECK_LT(kp_idx, tgt_kps.size());

        const Eigen::Vector2f src_point = src_kps[src_kp_idx];
        const Eigen::Vector2f tgt_point = tgt_kps[kp_idx];

        const Eigen::Vector3f dir_camspace =
            src_camera.intrinsics.Unproject(src_point);
        const Ray ray_worldspace = {
            .origin = src_camera.pose.Center(),
            .dir = src_camera.pose.Derotate(dir_camspace),
        };

        const Ray ray_objectspace = {
            .origin = (model_matrix_inv * ray_worldspace.origin.homogeneous())
                          .hnormalized(),
            .dir = model_matrix_inv.block<3, 3>(0, 0) * ray_worldspace.dir,
        };

        bool found_intersection = false;
        Eigen::Vector3f point_object;

        const uint32_t primitive_id =
            GetIntersectionPrimitiveId(src_frame, src_kp_idx);

        if (primitive_id != kInvalidIndex) {
            const Triangle triangle = mesh.Inner().GetTriangle(primitive_id);
            const auto maybe_point = Intersect(ray_objectspace, triangle);
            if (maybe_point) {
                point_object = *maybe_point;
                found_intersection = true;
            }
        }

        if (!found_intersection) {
            const std::optional<RayHit> maybe_rayhit =
                mesh.RayCast(ray_objectspace.origin, ray_objectspace.dir, true);

            if (maybe_rayhit.has_value()) {
                SetIntersectionPrimitiveId(src_frame, src_kp_idx,
                                           maybe_rayhit->primitive_id);
                point_object = maybe_rayhit->pos;
                found_intersection = true;
            }
        }

        if (!found_intersection) {
            SetIntersectionPrimitiveId(src_frame, src_kp_idx, kInvalidIndex);
            return std::nullopt;
        }

        const Eigen::Vector3f point_world =
            (model_matrix * point_object.homogeneous()).hnormalized();
        const Eigen::Vector3f point_camera = tgt_camera.pose.Apply(point_world);

        if (tgt_camera.intrinsics.IsBehind(point_camera)) {
            return std::nullopt;
        }

        return tgt_camera.intrinsics.Project(point_camera) - tgt_point;
    }

    bool EvaluateWithJacobian(
        const CameraTrajectory& traj, const ImagePairFlow& flow, size_t kp_idx,
        RefRowMajorMatrixf<kResidualLength, Eigen::Dynamic>* J_src,
        RefRowMajorMatrixf<kResidualLength, Eigen::Dynamic>* J_tgt,
        Eigen::Vector2f& res) const {
        const int32_t src_frame = flow.image_id_from;
        const int32_t tgt_frame = flow.image_id_to;

        const Keypoints& src_kps = cached_database.ReadKeypoints(src_frame);
        const KeypointsIndices& src_kps_indices = flow.src_kps_indices;
        const Keypoints& tgt_kps = flow.tgt_kps;

        const auto& maybe_src_camera = traj.Get(src_frame);
        CHECK(maybe_src_camera.has_value());
        const auto& maybe_tgt_camera = traj.Get(tgt_frame);
        CHECK(maybe_tgt_camera.has_value());

        const CameraState& src_camera = *maybe_src_camera;
        const CameraState& tgt_camera = *maybe_tgt_camera;

        CHECK_LT(kp_idx, src_kps_indices.size());
        CHECK_LT(src_kps_indices[kp_idx], src_kps.size());
        CHECK_LT(kp_idx, tgt_kps.size());

        const uint32_t src_kp_idx = src_kps_indices[kp_idx];
        const uint32_t primitive_id =
            GetIntersectionPrimitiveId(src_frame, src_kp_idx);

        if (primitive_id == kInvalidIndex) {
            return false;
        }

        const Eigen::Vector2f src_point = src_kps[src_kp_idx];
        const Eigen::Vector2f tgt_point = tgt_kps[kp_idx];

        RowMajorMatrixf<3, 3> dDirCam_dIntrin;
        Eigen::Vector3f dirCam;
        src_camera.intrinsics.UnprojectWithJac(src_point, &dirCam, nullptr,
                                               &dDirCam_dIntrin);

        Eigen::Vector3f origin;
        RowMajorMatrixf<3, 3> dOrigin_dR;
        RowMajorMatrixf<3, 3> dOrigin_dt;
        src_camera.pose.CenterWithJac(&origin, &dOrigin_dR, &dOrigin_dt);

        Eigen::Vector3f dirWorld;
        RowMajorMatrixf<3, 3> dDirWorld_dDirCam;
        RowMajorMatrixf<3, 3> dDirWorld_dR;
        src_camera.pose.DerotateWithJac(dirCam, &dirWorld, &dDirWorld_dDirCam,
                                        &dDirWorld_dR);

        const Ray ray_worldspace = {
            .origin = origin,
            .dir = dirWorld,
        };

        const Triangle triangle_objectspace =
            mesh.Inner().GetTriangle(primitive_id);
        const Plane plane_worldspace = {
            .point = (model_matrix * triangle_objectspace.p1.homogeneous())
                         .head<3>(),
            .normal =
                model_matrix_inv.transpose().block<3, 3>(0, 0) *
                (triangle_objectspace.p2 - triangle_objectspace.p1)
                    .cross(triangle_objectspace.p3 - triangle_objectspace.p1),
        };

        Eigen::Vector3f X;
        RowMajorMatrixf<3, 3> dX_dOrigin;
        RowMajorMatrixf<3, 3> dX_dDirWorld;

        const bool ok = IntersectWithJac(ray_worldspace, plane_worldspace, &X,
                                         &dX_dOrigin, &dX_dDirWorld);
        CHECK(ok);

        Eigen::Vector3f XCam;
        RowMajorMatrixf<3, 3> dXCam_dX;
        RowMajorMatrixf<3, 3> dXCam_dR;
        // dXCam_dt is Identity, so we can drop it
        // RowMajorMatrixf<3, 3> dXCam_dt;
        tgt_camera.pose.ApplyWithJac(X, &XCam, &dXCam_dX, &dXCam_dR, nullptr);
        if (tgt_camera.intrinsics.IsBehind(XCam)) {
            return false;
        }

        Eigen::Vector2f p;
        RowMajorMatrixf<2, 3> dp_dXCam;
        RowMajorMatrixf<2, 3> dp_dIntrin;
        tgt_camera.intrinsics.ProjectWithJac(XCam, &p, &dp_dXCam, &dp_dIntrin);

        // Copmute residual
        res = p - tgt_point;

        // Compute jacboian
        const RowMajorMatrixf<2, 3> dp_dX = dp_dXCam * dXCam_dX;

        // Source camera
        if (J_src) {
            CHECK_EQ(J_src->rows(), 2);
            CHECK_GE(J_src->cols(), 6);

            J_src->block<2, 3>(0, 0) =
                dp_dX * (dX_dOrigin * dOrigin_dR + dX_dDirWorld * dDirWorld_dR);
            J_src->block<2, 3>(0, 3) = dp_dX * dX_dOrigin * dOrigin_dt;

            if (optimize_focal_length || optimize_principal_point) {
                CHECK_GE(J_src->cols(), 9);

                J_src->block<2, 3>(0, 6) =
                    dp_dX * dX_dDirWorld * dDirWorld_dDirCam * dDirCam_dIntrin;

                if (!optimize_focal_length) {
                    J_src->block<2, 1>(0, 6).setZero();
                }
                if (!optimize_principal_point) {
                    J_src->block<2, 2>(0, 7).setZero();
                }
            }
        }

        // Target camera
        if (J_tgt) {
            CHECK_EQ(J_tgt->rows(), 2);
            CHECK_GE(J_tgt->cols(), 6);

            J_tgt->block<2, 3>(0, 0) = dp_dXCam * dXCam_dR;
            J_tgt->block<2, 3>(0, 3) = dp_dXCam;

            if (optimize_focal_length || optimize_principal_point) {
                CHECK_GE(J_tgt->cols(), 9);

                J_tgt->block<2, 3>(0, 6) = dp_dIntrin;

                if (!optimize_focal_length) {
                    J_tgt->block<2, 1>(0, 6).setZero();
                }
                if (!optimize_principal_point) {
                    J_tgt->block<2, 2>(0, 7).setZero();
                }
            }
        }

        return true;
    }

    void Step(CameraState& state, const RefConstVectorXf& dp) const {
        CHECK_GE(dp.rows(), 6);

        state.pose.q = QuatStepPost(state.pose.q, dp.block<3, 1>(0, 0));
        state.pose.t = state.pose.t + dp.block<3, 1>(3, 0);

        if (optimize_focal_length) {
            CHECK_GE(dp.rows(), 7);

            state.intrinsics.fy = state.intrinsics.fy + dp(6, 0);
            state.intrinsics.fx =
                state.intrinsics.fy * state.intrinsics.aspect_ratio;

            state.intrinsics.fy =
                std::clamp(state.intrinsics.fy, bounds.f_low, bounds.f_high);
            state.intrinsics.fx =
                std::clamp(state.intrinsics.fx, bounds.f_low, bounds.f_high);
        }
        if (optimize_principal_point) {
            CHECK_GE(dp.rows(), 9);

            state.intrinsics.cx = state.intrinsics.cx + dp(7, 0);
            state.intrinsics.cy = state.intrinsics.cy + dp(8, 0);

            state.intrinsics.cx =
                std::clamp(state.intrinsics.cx, bounds.cx_low, bounds.cx_high);
            state.intrinsics.cy =
                std::clamp(state.intrinsics.cy, bounds.cy_low, bounds.cy_high);
        }
    }

   protected:
    inline uint32_t FrameIdx(int32_t frame_id) const {
        CHECK_GE(frame_id, cached_database.FirstFrame());
        CHECK_LE(frame_id, cached_database.LastFrame());

        return frame_id - cached_database.FirstFrame();
    }

    inline uint32_t GetIntersectionPrimitiveId(int32_t frame_id,
                                               uint32_t kp_idx) const {
        std::atomic_ref<uint32_t> primitive_atomic{
            intersections[FrameIdx(frame_id)][kp_idx]};
        return primitive_atomic.load(std::memory_order_relaxed);
    }

    inline void SetIntersectionPrimitiveId(int32_t frame_id, uint32_t kp_idx,
                                           uint32_t primitive_id) const {
        std::atomic_ref<uint32_t> primitive_atomic{
            intersections[FrameIdx(frame_id)][kp_idx]};
        primitive_atomic.store(primitive_id, std::memory_order_relaxed);
    }

   protected:
    static constexpr uint32_t kInvalidIndex =
        std::numeric_limits<uint32_t>::max();

    const CachedDatabase& cached_database;
    const AcceleratedMesh& mesh;
    const RowMajorMatrix4f model_matrix;
    const RowMajorMatrix4f model_matrix_inv;
    const bool optimize_focal_length;
    const bool optimize_principal_point;
    const CameraIntrinsics::Bounds bounds;

    size_t num_params_per_camera;

    mutable std::vector<std::vector<uint32_t>> intersections;

    // --- user (helper) track constraints (see user_constraints.h) ---
    // One UserEdge per interior frame that carries >=1 observation; `obs` holds
    // (track index, measured pixel) pairs for that frame.
    struct UserEdge {
        int32_t frame_id;
        std::vector<std::pair<uint32_t, Eigen::Vector2f>> obs;
    };
    std::vector<Eigen::Vector3f> user_world_points;   // per track (world space)
    std::vector<float> user_track_weights;            // per track
    std::vector<UserEdge> user_edges;                 // one per interior frame
    int32_t user_ref_frame = 0;                       // partner block (FirstFrame)
    float user_track_weight_global = 1.0f;            // global weight multiplier
};

class GlobalRefinementProblem : public RefinementProblemBase {
   public:
    using RefinementProblemBase::RefinementProblemBase;

    size_t NumParams() const {
        return num_params_per_camera * cached_database.NumFrames();
    }

    size_t NumResiduals(size_t edge_idx) const {
        const size_t num_flows = cached_database.NumFlows();
        if (edge_idx < num_flows) {
            return cached_database.ReadFlow(edge_idx).tgt_kps.size();
        }
        return user_edges[edge_idx - num_flows].obs.size();
    }

    size_t NumParamBlocks() const { return cached_database.NumFrames(); }

    size_t ParamBlockLength() const { return num_params_per_camera; }

    // Flow edges first, then one user (helper) track edge per interior frame
    // that carries observations.
    size_t NumEdges() const {
        return cached_database.NumFlows() + user_edges.size();
    }

    std::pair<size_t, size_t> GetEdge(size_t edge_idx) const {
        const size_t num_flows = cached_database.NumFlows();
        if (edge_idx < num_flows) {
            return RefinementProblemBase::GetEdge(edge_idx);
        }
        // User edge: {observed interior frame, first frame}. The first frame is
        // ground truth (frozen) and only acts as the required distinct partner
        // block; its Jacobian half is left zero.
        const UserEdge& e = user_edges[edge_idx - num_flows];
        return {FrameIdx(e.frame_id), FrameIdx(user_ref_frame)};
    }

    Float EdgeWeight(size_t edge_idx) const {
        const size_t num_flows = cached_database.NumFlows();
        if (edge_idx < num_flows) {
            const ImagePairFlow& flow = cached_database.ReadFlow(edge_idx);
            return FrameWeight(flow.image_id_from);
        }
        // User edges are deliberate anchors: not distance-attenuated. A zero
        // global weight disables them entirely (the solver skips zero-weight
        // edges).
        return user_track_weight_global;
    }

    Float ResidualWeight(size_t edge_idx, size_t res_idx) const {
        const size_t num_flows = cached_database.NumFlows();
        if (edge_idx < num_flows) {
            return RefinementProblemBase::ResidualWeight(edge_idx, res_idx);
        }
        const UserEdge& e = user_edges[edge_idx - num_flows];
        return user_track_weights[e.obs[res_idx].first];
    }

    std::optional<Eigen::Vector2f> Evaluate(const CameraTrajectory& traj,
                                            size_t edge_idx,
                                            size_t res_idx) const {
        const size_t num_flows = cached_database.NumFlows();
        if (edge_idx < num_flows) {
            return RefinementProblemBase::Evaluate(traj, edge_idx, res_idx);
        }
        return EvaluateUser(traj, edge_idx - num_flows, res_idx);
    }

    bool EvaluateWithJacobian(
        const CameraTrajectory& traj, size_t edge_idx, size_t kp_idx,
        RowMajorMatrixf<kResidualLength, Eigen::Dynamic>& J,
        Eigen::Vector2f& res) const {
        const size_t num_flows = cached_database.NumFlows();
        if (edge_idx >= num_flows) {
            return EvaluateUserWithJacobian(traj, edge_idx - num_flows, kp_idx,
                                            J, res);
        }

        RefRowMajorMatrixf<kResidualLength, Eigen::Dynamic> J_src =
            J.block(0, 0, kResidualLength, ParamBlockLength());
        RefRowMajorMatrixf<kResidualLength, Eigen::Dynamic> J_tgt =
            J.block(0, ParamBlockLength(), kResidualLength, ParamBlockLength());

        const ImagePairFlow& flow = cached_database.ReadFlow(edge_idx);
        auto* J_src_ptr = IsGroundTruth(flow.image_id_from) ? nullptr : &J_src;
        auto* J_tgt_ptr = IsGroundTruth(flow.image_id_to) ? nullptr : &J_tgt;

        return RefinementProblemBase::EvaluateWithJacobian(
            traj, flow, kp_idx, J_src_ptr, J_tgt_ptr, res);
    }

    // --- user (helper) track residuals ---
    // The 3D point is FIXED (rigid on the object), so a user residual depends
    // only on the observed camera (no source-camera / ray-intersection term).
    // It is the J_tgt-only case of the flow residual.
    std::optional<Eigen::Vector2f> EvaluateUser(const CameraTrajectory& traj,
                                                size_t user_idx,
                                                size_t res_idx) const {
        const UserEdge& e = user_edges[user_idx];
        const auto& ob = e.obs[res_idx];
        const Eigen::Vector3f& point_world = user_world_points[ob.first];

        const auto& maybe_camera = traj.Get(e.frame_id);
        CHECK(maybe_camera.has_value());
        const CameraState& camera = *maybe_camera;

        const Eigen::Vector3f point_camera = camera.pose.Apply(point_world);
        if (camera.intrinsics.IsBehind(point_camera)) {
            return std::nullopt;
        }
        return camera.intrinsics.Project(point_camera) - ob.second;
    }

    bool EvaluateUserWithJacobian(
        const CameraTrajectory& traj, size_t user_idx, size_t res_idx,
        RowMajorMatrixf<kResidualLength, Eigen::Dynamic>& J,
        Eigen::Vector2f& res) const {
        // GetEdge(user) = {observed frame, first frame}, so J's FIRST param
        // block belongs to the observed (interior) camera; the SECOND (first
        // frame) half stays zero — the solver already zeroed J for us.
        const UserEdge& e = user_edges[user_idx];
        const auto& ob = e.obs[res_idx];
        const Eigen::Vector3f& point_world = user_world_points[ob.first];

        const auto& maybe_camera = traj.Get(e.frame_id);
        CHECK(maybe_camera.has_value());
        const CameraState& camera = *maybe_camera;

        Eigen::Vector3f point_camera;
        RowMajorMatrixf<3, 3> dPointCam_dR;
        // dPointCam_dt is Identity, so we drop it (handled below as dp_dXCam).
        Pose::ApplyWithJac(point_world, camera.pose.R(), camera.pose.t,
                           &point_camera, nullptr, &dPointCam_dR, nullptr);
        if (camera.intrinsics.IsBehind(point_camera)) {
            return false;
        }

        Eigen::Vector2f p;
        RowMajorMatrixf<2, 3> dp_dPointCam;
        RowMajorMatrixf<2, 3> dp_dIntrin;
        camera.intrinsics.ProjectWithJac(point_camera, &p, &dp_dPointCam,
                                         &dp_dIntrin);

        res = p - ob.second;

        // Fill the first half of J (the observed camera). The second half is the
        // frozen first-frame partner block and stays zero.
        RefRowMajorMatrixf<kResidualLength, Eigen::Dynamic> J_obs =
            J.block(0, 0, kResidualLength, ParamBlockLength());
        J_obs.block<2, 3>(0, 0) = dp_dPointCam * dPointCam_dR;  // rotation
        J_obs.block<2, 3>(0, 3) = dp_dPointCam;                 // translation

        if (optimize_focal_length || optimize_principal_point) {
            J_obs.block<2, 3>(0, 6) = dp_dIntrin;
            if (!optimize_focal_length) {
                J_obs.block<2, 1>(0, 6).setZero();
            }
            if (!optimize_principal_point) {
                J_obs.block<2, 2>(0, 7).setZero();
            }
        }

        return true;
    }

    void Step(const CameraTrajectory& traj,
              const RowMajorMatrixf<Eigen::Dynamic, 1>& dp,
              CameraTrajectory& result) const {
        CHECK_EQ(result.FirstFrame(), traj.FirstFrame());
        CHECK_EQ(result.LastFrame(), traj.LastFrame());
        CHECK_EQ(cached_database.FirstFrame(), traj.FirstFrame());
        CHECK_EQ(cached_database.LastFrame(), traj.LastFrame());
        CHECK_EQ(dp.rows(), static_cast<Eigen::Index>(num_params_per_camera *
                                                      traj.Count()));

        // The first and last cameras are constant.
        // TODO: Maybe don't add them to the problem to begin with?
        result.Set(traj.FirstFrame(), *traj.Get(traj.FirstFrame()));
        result.Set(traj.LastFrame(), *traj.Get(traj.LastFrame()));

        const int32_t from = traj.FirstFrame() + 1;
        const int32_t to = traj.LastFrame() - 1;

        for (int32_t frame = from; frame <= to; frame++) {
            CameraState camera = *traj.Get(frame);

            const Eigen::Index J_idx = num_params_per_camera * FrameIdx(frame);
            const RefConstVectorXf dp_cam =
                dp.block(J_idx, 0, num_params_per_camera, 1);

            RefinementProblemBase::Step(camera, dp_cam);
            result.Set(frame, camera);
        }
    }
};

template <typename LossFunction>
static void RefineTrajectory(const Database& database, CameraTrajectory& traj,
                             const RowMajorMatrix4f& model_matrix,
                             const AcceleratedMesh& accel_mesh,
                             const LossFunction& loss_fn,
                             RefinerCallback callback,
                             const RefinerOptions& opts) {
    SPDLOG_INFO("Refining trajectory from frame #{} to frame #{} inclusive",
                traj.FirstFrame(), traj.LastFrame());

    CHECK_GT(traj.Count(), 2u);
    for (int32_t frame = traj.FirstFrame(); frame <= traj.LastFrame();
         frame++) {
        CHECK(traj.IsFrameFilled(frame));
    }

    CachedDatabase cached_database{database, traj, accel_mesh.Inner(),
                                   model_matrix, opts.is_masked};
    RefinerUpdate update = {};

    auto callback_wrapper = [&](const BundleStats& stats) {
        update.progress = static_cast<float>(stats.iterations) /
                          opts.bundle_opts.max_iterations;
        update.message = fmt::format("Cost: {:.02f} (Initial: {:.02f})",
                                     stats.cost, stats.initial_cost);
        update.stats = stats;

        return callback(update);
    };

    GlobalRefinementProblem problem{
        cached_database,
        accel_mesh,
        model_matrix,
        traj,
        opts.optimize_focal_length,
        opts.optimize_principal_point,
        traj.Get(traj.FirstFrame())
            ->intrinsics.GetBounds(opts.min_fov_deg, opts.max_fov_deg),
        opts.user_tracks,
        opts.user_track_weight};

    LevMarqSparseSolve(problem, loss_fn, &traj, opts.bundle_opts,
                       callback_wrapper);
}

void RefineTrajectory(const Database& database, CameraTrajectory& traj,
                      const RowMajorMatrix4f& model_matrix,
                      const AcceleratedMesh& accel_mesh,
                      RefinerCallback callback, const RefinerOptions& opts) {
    switch (opts.bundle_opts.loss_type) {
#define SWITCH_LOSS_FUNCTION_CASE(LossFunction)                             \
    {                                                                       \
        LossFunction loss_fn(opts.bundle_opts.loss_scale);                  \
        RefineTrajectory(database, traj, model_matrix, accel_mesh, loss_fn, \
                         callback, opts);                                   \
    }
        SWITCH_LOSS_FUNCTIONS
#undef SWITCH_LOSS_FUNCTION_CASE
        default:
            throw std::runtime_error(
                fmt::format("Unknown loss type: {}",
                            static_cast<int>(opts.bundle_opts.loss_type)));
    }
}

void RefineTrajectory(const std::string& database_path, CameraTrajectory& traj,
                      const RowMajorMatrix4f& model_matrix,
                      const AcceleratedMesh& accel_mesh,
                      RefinerCallback callback, const RefinerOptions& opts) {
    Database database{database_path};
    RefineTrajectory(database, traj, model_matrix, accel_mesh, callback, opts);
}
