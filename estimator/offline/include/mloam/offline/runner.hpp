#pragma once

#include <cstddef>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "mloam/offline/artifacts.hpp"
#include "mloam/offline/joint_backend.hpp"
#include "mloam/offline/pcd_preprocessor.hpp"
#include "mloam/offline/sequence_reader.hpp"

namespace mloam {
namespace offline {

struct EstimatorInput {
  std::size_t frame_index = 0;
  double timestamp = 0.0;
  std::vector<PreparedLidarFrame> lidars;
  const ScenarioConfiguration* scenario = nullptr;
};

struct EstimatorOutput {
  RigidTransform world_T_reference;
  std::map<std::string, RigidTransform> reference_T_lidar;
  std::map<std::string, bool> observable;
  std::map<std::string, std::size_t> corner_features;
  std::map<std::string, std::size_t> surface_features;
  // M-LOAM feature points already expressed in the reference-LiDAR frame.
  std::map<std::string, std::vector<EncodedPointXYZI>> feature_points_reference;
  bool save_keyframe = true;
  double odometry_ms = -1.0;
  double mapping_ms = -1.0;
};

using EstimatorCallback =
    std::function<EstimatorOutput(const EstimatorInput& input)>;

struct RunResult {
  RunStatus status = RunStatus::kFailed;
  std::size_t processed_frames = 0;
  std::size_t dropped_frames = 0;
  std::map<std::string, RigidTransform> final_extrinsics;
  JointBackendResult backend;
};

RunResult runOffline(const OfflineManifest& manifest,
                     const ScenarioConfiguration& scenario,
                     const std::string& output_directory,
                     const EstimatorCallback& estimator);

}  // namespace offline
}  // namespace mloam
