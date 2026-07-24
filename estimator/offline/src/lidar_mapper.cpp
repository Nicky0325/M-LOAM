#include "mloam/offline/lidar_mapper.hpp"

#include <cmath>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace mloam {
namespace offline {

LidarMapper::LidarMapper(std::vector<LidarConfig> lidars, double voxel_size)
    : voxel_size_(voxel_size) {
  if (voxel_size_ <= 0.0) throw std::invalid_argument("voxel size must be positive");
  for (const auto& lidar : lidars) {
    if (!lidar.enabled) continue;
    if (lidar_order_.size() >= 256)
      throw std::invalid_argument("at most 256 LiDARs are supported");
    lidar_indices_[lidar.name] =
        static_cast<std::uint8_t>(lidar_order_.size());
    lidar_order_.push_back(lidar.name);
    colors_[lidar.name] = lidar.color;
    keyframes_[lidar.name] = {};
  }
}

void LidarMapper::processFrame(MappingFrame frame) {
  for (auto& lidar : frame.lidars) {
    auto keyframes = keyframes_.find(lidar.lidar_name);
    if (keyframes == keyframes_.end())
      throw std::invalid_argument("mapping frame contains unknown LiDAR: " +
                                  lidar.lidar_name);
    SensorKeyframe keyframe;
    keyframe.frame_index = frame.frame_index;
    keyframe.timestamp = frame.timestamp;
    keyframe.world_T_reference = frame.world_T_reference;
    const auto extrinsic = frame.reference_T_lidar.find(lidar.lidar_name);
    if (extrinsic == frame.reference_T_lidar.end())
      throw std::invalid_argument("mapping frame is missing extrinsic for " +
                                  lidar.lidar_name);
    keyframe.reference_T_lidar_used = extrinsic->second;
    keyframe.points = std::move(lidar.native_points);
    keyframes->second.push_back(std::move(keyframe));
  }
}

std::vector<RgbPoint> LidarMapper::buildOnlineRgbMap() const {
  std::vector<RgbPoint> result;
  for (const auto& lidar_name : lidar_order_) {
    std::set<std::tuple<long long, long long, long long>> occupied_voxels;
    for (const auto& keyframe : keyframes_.at(lidar_name)) {
      const RigidTransform world_T_lidar =
          keyframe.world_T_reference * keyframe.reference_T_lidar_used;
      for (const auto& point : keyframe.points) {
        const Eigen::Vector3d global =
            world_T_lidar.rotation * Eigen::Vector3d(point.x, point.y, point.z) +
            world_T_lidar.translation;
        const auto voxel = std::make_tuple(
            static_cast<long long>(std::floor(global.x() / voxel_size_)),
            static_cast<long long>(std::floor(global.y() / voxel_size_)),
            static_cast<long long>(std::floor(global.z() / voxel_size_)));
        if (!occupied_voxels.insert(voxel).second) continue;
        const auto color = colors_.at(lidar_name);
        result.push_back({static_cast<float>(global.x()),
                          static_cast<float>(global.y()),
                          static_cast<float>(global.z()), color.r, color.g,
                          color.b, lidar_indices_.at(lidar_name)});
      }
    }
  }
  return result;
}

const std::vector<SensorKeyframe>& LidarMapper::keyframes(
    const std::string& lidar_name) const {
  return keyframes_.at(lidar_name);
}

std::vector<RgbPoint> LidarMapper::rebuildFinalRgbMap(
    const std::map<std::string, RigidTransform>& final_extrinsics) const {
  std::vector<RgbPoint> result;
  for (const auto& lidar_name : lidar_order_) {
    const auto extrinsic = final_extrinsics.find(lidar_name);
    if (extrinsic == final_extrinsics.end())
      throw std::invalid_argument("final extrinsic missing for " + lidar_name);
    std::set<std::tuple<long long, long long, long long>> occupied_voxels;
    for (const auto& keyframe : keyframes_.at(lidar_name)) {
      const RigidTransform world_T_lidar =
          keyframe.world_T_reference * extrinsic->second;
      for (const auto& point : keyframe.points) {
        const Eigen::Vector3d global =
            world_T_lidar.rotation * Eigen::Vector3d(point.x, point.y, point.z) +
            world_T_lidar.translation;
        const auto voxel = std::make_tuple(
            static_cast<long long>(std::floor(global.x() / voxel_size_)),
            static_cast<long long>(std::floor(global.y() / voxel_size_)),
            static_cast<long long>(std::floor(global.z() / voxel_size_)));
        if (!occupied_voxels.insert(voxel).second) continue;
        const auto color = colors_.at(lidar_name);
        result.push_back({static_cast<float>(global.x()),
                          static_cast<float>(global.y()),
                          static_cast<float>(global.z()), color.r, color.g,
                          color.b, lidar_indices_.at(lidar_name)});
      }
    }
  }
  return result;
}

RigidTransform LidarMapper::correctedReferencePose(
    std::size_t frame_index, const RigidTransform& original_pose,
    const std::map<std::size_t, RigidTransform>& optimized_keyframe_poses)
    const {
  if (optimized_keyframe_poses.empty()) return original_pose;
  const auto exact = optimized_keyframe_poses.find(frame_index);
  if (exact != optimized_keyframe_poses.end()) return exact->second;
  std::map<std::size_t, RigidTransform> original_anchors;
  for (const auto& lidar_name : lidar_order_) {
    for (const auto& keyframe : keyframes_.at(lidar_name)) {
      if (optimized_keyframe_poses.count(keyframe.frame_index) != 0)
        original_anchors[keyframe.frame_index] = keyframe.world_T_reference;
    }
    if (original_anchors.size() == optimized_keyframe_poses.size()) break;
  }
  if (original_anchors.empty()) return original_pose;
  auto upper = optimized_keyframe_poses.lower_bound(frame_index);
  auto lower = upper;
  if (lower != optimized_keyframe_poses.begin()) --lower;
  if (upper == optimized_keyframe_poses.end()) upper = lower;
  if (upper == optimized_keyframe_poses.begin() && upper->first > frame_index)
    lower = upper;

  const auto correction = [&](const auto& anchor) {
    return anchor->second * original_anchors.at(anchor->first).inverse();
  };
  const RigidTransform lower_correction = correction(lower);
  if (lower->first == upper->first)
    return lower_correction * original_pose;
  const RigidTransform upper_correction = correction(upper);
  const double fraction =
      static_cast<double>(frame_index - lower->first) /
      static_cast<double>(upper->first - lower->first);
  RigidTransform interpolated;
  interpolated.translation =
      (1.0 - fraction) * lower_correction.translation +
      fraction * upper_correction.translation;
  interpolated.rotation =
      lower_correction.rotation.slerp(fraction, upper_correction.rotation);
  interpolated.rotation.normalize();
  return interpolated * original_pose;
}

std::vector<RgbPoint> LidarMapper::rebuildCorrectedRgbMap(
    const std::map<std::string, RigidTransform>& final_extrinsics,
    const std::map<std::size_t, RigidTransform>& optimized_keyframe_poses)
    const {
  std::vector<RgbPoint> result;
  for (const auto& lidar_name : lidar_order_) {
    const auto extrinsic = final_extrinsics.find(lidar_name);
    if (extrinsic == final_extrinsics.end())
      throw std::invalid_argument("final extrinsic missing for " + lidar_name);
    std::set<std::tuple<long long, long long, long long>> occupied_voxels;
    for (const auto& keyframe : keyframes_.at(lidar_name)) {
      const RigidTransform world_T_reference = correctedReferencePose(
          keyframe.frame_index, keyframe.world_T_reference,
          optimized_keyframe_poses);
      const RigidTransform world_T_lidar =
          world_T_reference * extrinsic->second;
      for (const auto& point : keyframe.points) {
        const Eigen::Vector3d global =
            world_T_lidar.rotation * Eigen::Vector3d(point.x, point.y, point.z) +
            world_T_lidar.translation;
        const auto voxel = std::make_tuple(
            static_cast<long long>(std::floor(global.x() / voxel_size_)),
            static_cast<long long>(std::floor(global.y() / voxel_size_)),
            static_cast<long long>(std::floor(global.z() / voxel_size_)));
        if (!occupied_voxels.insert(voxel).second) continue;
        const auto color = colors_.at(lidar_name);
        result.push_back({static_cast<float>(global.x()),
                          static_cast<float>(global.y()),
                          static_cast<float>(global.z()), color.r, color.g,
                          color.b, lidar_indices_.at(lidar_name)});
      }
    }
  }
  return result;
}

}  // namespace offline
}  // namespace mloam
