#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "mloam/offline/joint_backend.hpp"

namespace offline = mloam::offline;

namespace {

offline::RigidTransform pose(double x, double y, double yaw_degrees) {
  offline::RigidTransform result;
  result.translation = Eigen::Vector3d(x, y, 0.0);
  result.rotation =
      Eigen::AngleAxisd(yaw_degrees * M_PI / 180.0,
                        Eigen::Vector3d::UnitZ());
  return result;
}

std::vector<Eigen::Vector3d> roomPoints() {
  std::vector<Eigen::Vector3d> result;
  for (int x = -4; x <= 16; ++x)
    for (int y = -6; y <= 10; ++y)
      result.emplace_back(0.5 * x, 0.5 * y, 0.0);
  for (int y = -6; y <= 10; ++y)
    for (int z = 0; z <= 6; ++z)
      result.emplace_back(6.0, 0.5 * y, 0.5 * z);
  for (int x = -4; x <= 16; ++x)
    for (int z = 0; z <= 6; ++z)
      result.emplace_back(0.5 * x, 4.0, 0.5 * z);
  return result;
}

offline::PreparedLidarFrame cloud(
    const std::string& name, const offline::RigidTransform& world_T_lidar,
    const std::vector<Eigen::Vector3d>& world_points) {
  offline::PreparedLidarFrame result;
  result.lidar_name = name;
  const auto lidar_T_world = world_T_lidar.inverse();
  result.native_points.reserve(world_points.size());
  for (const auto& world : world_points) {
    const Eigen::Vector3d local =
        lidar_T_world.rotation * world + lidar_T_world.translation;
    offline::NativePointXYZIRT point;
    point.x = static_cast<float>(local.x());
    point.y = static_cast<float>(local.y());
    point.z = static_cast<float>(local.z());
    result.native_points.push_back(point);
  }
  return result;
}

offline::LidarMapper syntheticMapper(
    const offline::RigidTransform& true_extrinsic,
    const offline::RigidTransform& initial_extrinsic) {
  offline::LidarConfig top;
  top.name = "top";
  top.enabled = true;
  top.color = {255, 0, 0};
  offline::LidarConfig side = top;
  side.name = "side";
  side.color = {0, 255, 0};
  offline::LidarMapper mapper({top, side}, 0.1);
  const auto world_points = roomPoints();
  for (std::size_t frame = 0; frame < 6; ++frame) {
    const auto truth = pose(0.35 * frame, 0.08 * frame, 2.0 * frame);
    auto estimate = truth;
    if (frame != 0) {
      estimate.translation +=
          Eigen::Vector3d(0.03 * std::sin(frame), -0.02 * std::cos(frame), 0.0);
      estimate.rotation =
          Eigen::AngleAxisd(0.4 * M_PI / 180.0,
                            Eigen::Vector3d::UnitZ()) *
          estimate.rotation;
    }
    offline::MappingFrame input;
    input.frame_index = frame;
    input.timestamp = 0.1 * frame;
    input.world_T_reference = estimate;
    input.reference_T_lidar["top"] = offline::RigidTransform();
    input.reference_T_lidar["side"] = initial_extrinsic;
    input.lidars.push_back(cloud("top", truth, world_points));
    input.lidars.push_back(
        cloud("side", truth * true_extrinsic, world_points));
    mapper.processFrame(std::move(input));
  }
  return mapper;
}

offline::JointBackendConfig testConfig() {
  offline::JointBackendConfig result;
  result.enabled = true;
  result.mode = offline::JointBackendMode::kCoarseBootstrap;
  result.initial_voxel_size = 2.0;
  result.minimum_voxel_size = 0.25;
  result.downsample_size = 0.2;
  result.planarity_ratio = 10.0;
  result.minimum_points_per_voxel = 8;
  result.maximum_points_per_observation = 4;
  result.maximum_points_per_cloud = 5000;
  result.minimum_keyframes = 4;
  result.maximum_keyframes = 6;
  result.minimum_mixed_voxels_per_lidar = 3;
  result.heldout_fraction = 0.2;
  result.minimum_heldout_improvement = 0.0;
  result.pose_outer_iterations = 1;
  result.extrinsic_outer_iterations = 4;
  result.joint_outer_iterations = 8;
  result.solver_iterations = 30;
  result.maximum_condition_number = 1e14;
  result.minimum_relative_eigenvalue = 1e-12;
  result.maximum_pose_rotation_update_deg = 20.0;
  result.maximum_pose_translation_update_m = 5.0;
  return result;
}

}  // namespace

TEST(JointBackend, ReportsDisabledWithoutChangingExtrinsics) {
  offline::LidarConfig top;
  top.name = "top";
  offline::LidarMapper mapper({top}, 0.1);
  offline::JointCalibrationBackend backend;
  const auto result = backend.optimize(
      mapper, "top", {{"top", offline::RigidTransform()}},
      offline::JointBackendConfig());
  EXPECT_EQ(offline::JointBackendState::kDisabled, result.state);
  EXPECT_FALSE(result.accepted);
}

TEST(JointBackend, RecoversFiveDegreeAndThirtyCentimeterPriorError) {
  offline::RigidTransform truth;
  truth.translation = Eigen::Vector3d(0.55, -0.18, 0.12);
  truth.rotation =
      Eigen::AngleAxisd(4.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-2.0 * M_PI / 180.0, Eigen::Vector3d::UnitY());
  offline::RigidTransform initial = truth;
  initial.translation += Eigen::Vector3d(0.24, -0.15, 0.09);
  initial.rotation =
      Eigen::AngleAxisd(5.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
      initial.rotation;
  const auto mapper = syntheticMapper(truth, initial);
  offline::JointCalibrationBackend backend;
  const auto result = backend.optimize(
      mapper, "top",
      {{"top", offline::RigidTransform()}, {"side", initial}}, testConfig());

  EXPECT_TRUE(result.eligible) << result.reason;
  EXPECT_TRUE(result.converged) << result.reason;
  EXPECT_TRUE(result.accepted)
      << result.reason << " reference " << result.initial_reference_objective
      << " -> " << result.final_reference_objective;
  EXPECT_LT(result.final_training_objective,
            result.initial_training_objective);
  EXPECT_LT(result.final_heldout_objective,
            result.initial_heldout_objective);
  ASSERT_EQ(6u, result.optimized_keyframe_poses.size());
  const auto estimated = result.optimized_extrinsics.at("side");
  const double translation_error =
      (estimated.translation - truth.translation).norm();
  Eigen::Quaterniond rotation_difference =
      truth.rotation.conjugate() * estimated.rotation;
  rotation_difference.normalize();
  const double rotation_error =
      Eigen::AngleAxisd(rotation_difference).angle() * 180.0 / M_PI;
  EXPECT_LT(translation_error, 0.10);
  EXPECT_LT(rotation_error, 1.0);
}
