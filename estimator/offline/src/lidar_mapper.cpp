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
    lidar_order_.push_back(lidar.name);
    colors_[lidar.name] = lidar.color;
    keyframes_[lidar.name] = {};
  }
}

void LidarMapper::processFrame(const MappingFrame& frame) {
  for (const auto& lidar : frame.lidars) {
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
    keyframe.points = lidar.native_points;
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
                          color.b, lidar_name});
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
                          color.b, lidar_name});
      }
    }
  }
  return result;
}

}  // namespace offline
}  // namespace mloam
