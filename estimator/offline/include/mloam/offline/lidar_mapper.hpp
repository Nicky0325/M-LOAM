#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "mloam/offline/types.hpp"

namespace mloam {
namespace offline {

struct SensorKeyframe {
  std::size_t frame_index = 0;
  double timestamp = 0.0;
  RigidTransform world_T_reference;
  RigidTransform reference_T_lidar_used;
  std::vector<NativePointXYZIRT> points;
};

struct RgbPoint {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  std::uint8_t r = 0;
  std::uint8_t g = 0;
  std::uint8_t b = 0;
  std::uint8_t lidar_index = 0;
};

class LidarMapper {
 public:
  LidarMapper(std::vector<LidarConfig> lidars, double voxel_size);

  void processFrame(MappingFrame frame);
  const std::vector<SensorKeyframe>& keyframes(
      const std::string& lidar_name) const;
  std::vector<RgbPoint> buildOnlineRgbMap() const;
  std::vector<RgbPoint> rebuildFinalRgbMap(
      const std::map<std::string, RigidTransform>& final_extrinsics) const;
  RigidTransform correctedReferencePose(
      std::size_t frame_index, const RigidTransform& original_pose,
      const std::map<std::size_t, RigidTransform>& optimized_keyframe_poses)
      const;
  std::vector<RgbPoint> rebuildCorrectedRgbMap(
      const std::map<std::string, RigidTransform>& final_extrinsics,
      const std::map<std::size_t, RigidTransform>& optimized_keyframe_poses)
      const;

 private:
  std::vector<std::string> lidar_order_;
  std::map<std::string, std::uint8_t> lidar_indices_;
  std::map<std::string, Rgb> colors_;
  std::map<std::string, std::vector<SensorKeyframe>> keyframes_;
  double voxel_size_;
};

}  // namespace offline
}  // namespace mloam
