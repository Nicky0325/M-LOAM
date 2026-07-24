#pragma once

#include <cstddef>
#include <map>
#include <string>
#include <vector>

#include "mloam/offline/lidar_mapper.hpp"
#include "mloam/offline/scenario.hpp"

namespace mloam {
namespace offline {

struct JointBackendResult;

enum class RunStatus { kSuccess, kNonConverged, kFailed };

struct SynchronizationRecord {
  std::size_t frame = 0;
  double timestamp = 0.0;
  std::string status;
  std::string reason;
  double max_raw_skew_s = 0.0;
  double max_corrected_skew_s = 0.0;
};

struct RuntimeRecord {
  std::size_t frame = 0;
  double preprocessing_ms = 0.0;
  double odometry_ms = 0.0;
  double mapping_ms = 0.0;
};

struct FeatureRecord {
  std::size_t frame = 0;
  std::string lidar_name;
  std::size_t corner_count = 0;
  std::size_t surface_count = 0;
};

struct TrajectoryRecord {
  std::size_t frame = 0;
  double timestamp = 0.0;
  RigidTransform world_T_reference;
};

struct ExtrinsicRecord {
  std::size_t frame = 0;
  double timestamp = 0.0;
  std::string lidar_name;
  RigidTransform reference_T_lidar;
  bool observable = false;
  double rotation_error_deg = 0.0;
  double translation_error_m = 0.0;
  CalibrationState state = CalibrationState::kInitializing;
};

class RunArtifacts {
 public:
  void addSynchronization(SynchronizationRecord record);
  void addRuntime(RuntimeRecord record);
  void addFeatures(FeatureRecord record);
  void addTrajectory(TrajectoryRecord record);
  void addExtrinsic(ExtrinsicRecord record);
  void addOnlineFeatures(
      const RigidTransform& world_T_reference,
      const std::map<std::string, std::vector<EncodedPointXYZI>>& features,
      const std::vector<LidarConfig>& lidars);

  void write(const std::string& output_directory,
             const OfflineManifest& manifest,
             const ScenarioConfiguration& scenario,
             const LidarMapper& mapper,
             const std::map<std::string, RigidTransform>& final_extrinsics,
             RunStatus status,
             const JointBackendResult* backend = nullptr) const;

 private:
  std::vector<SynchronizationRecord> synchronization_;
  std::vector<RuntimeRecord> runtimes_;
  std::vector<FeatureRecord> features_;
  std::vector<TrajectoryRecord> trajectory_;
  std::vector<ExtrinsicRecord> extrinsics_;
  std::vector<RgbPoint> online_features_;
};

const char* runStatusName(RunStatus status);

}  // namespace offline
}  // namespace mloam
