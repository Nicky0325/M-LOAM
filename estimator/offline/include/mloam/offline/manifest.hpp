#pragma once

#include <cstddef>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include "mloam/offline/types.hpp"

namespace mloam {
namespace offline {

struct OfflineManifest {
  std::string dataset_root;
  std::string output_root;
  std::string mloam_config;
  std::string reference_lidar;
  double synchronization_threshold = 0.06;
  std::size_t frame_begin = 0;
  std::size_t frame_end = std::numeric_limits<std::size_t>::max();
  std::size_t frame_stride = 1;
  double minimum_finite_ratio = 0.95;
  JointBackendConfig joint_backend;
  std::vector<LidarConfig> lidars;
};

OfflineManifest loadManifest(const std::string& path);
void validateManifest(const OfflineManifest& manifest);
void applyLidarSelection(OfflineManifest& manifest,
                         const std::vector<std::string>& include,
                         const std::vector<std::string>& exclude);
std::map<std::string, RigidTransform> referenceRelativeExtrinsics(
    const OfflineManifest& manifest);

}  // namespace offline
}  // namespace mloam
