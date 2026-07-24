#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace mloam {
namespace offline {

struct NativePointXYZIRT {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float reflectivity = 0.0f;
  std::uint16_t ring = 0;
  double timestamp = 0.0;
  std::size_t organized_column = 0;
};

struct EncodedPointXYZI {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float intensity = 0.0f;
};

struct Rgb {
  std::uint8_t r = 255;
  std::uint8_t g = 255;
  std::uint8_t b = 255;
};

struct RigidTransform {
  Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();

  RigidTransform inverse() const {
    RigidTransform result;
    result.rotation = rotation.conjugate();
    result.translation = -(result.rotation * translation);
    return result;
  }

  RigidTransform operator*(const RigidTransform& rhs) const {
    RigidTransform result;
    result.rotation = rotation * rhs.rotation;
    result.translation = translation + rotation * rhs.translation;
    return result;
  }
};

struct LidarConfig {
  std::string name;
  bool enabled = true;
  std::string directory;
  std::size_t ring_count = 0;
  double horizontal_fov_deg = 360.0;
  std::size_t expected_width = 0;
  double timestamp_scale = 2e-6;
  double scan_period = 0.1;
  double time_offset = 0.0;
  Rgb color;
  RigidTransform vehicle_T_lidar;
  int min_cluster_size = 30;
  int segment_valid_point_num = 5;
  int segment_valid_line_num = 3;
  double segment_theta_rad = 1.047;
  bool segment_cloud = false;
};

struct ScanMatch {
  std::string lidar_name;
  std::string path;
  double raw_timestamp = 0.0;
  double corrected_timestamp = 0.0;
  double raw_skew = 0.0;
  double corrected_skew = 0.0;
};

struct SynchronizedFrame {
  std::size_t reference_index = 0;
  double reference_timestamp = 0.0;
  bool complete = false;
  std::map<std::string, ScanMatch> scans;
  std::vector<std::string> missing_lidars;
};

struct PreparedLidarFrame {
  std::string lidar_name;
  double scan_timestamp = 0.0;
  std::size_t source_width = 0;
  std::size_t source_height = 0;
  std::size_t discarded_non_finite = 0;
  std::vector<NativePointXYZIRT> native_points;
  std::vector<EncodedPointXYZI> ring_ordered_points;
  std::vector<EncodedPointXYZI> outlier_ring_ordered_points;
  std::vector<std::size_t> ring_start_indices;
  std::vector<std::size_t> ring_end_indices;
};

struct MappingFrame {
  std::size_t frame_index = 0;
  double timestamp = 0.0;
  RigidTransform world_T_reference;
  std::vector<PreparedLidarFrame> lidars;
  std::map<std::string, RigidTransform> reference_T_lidar;
};

enum class JointBackendMode {
  kDisabled,
  kPreciseRefine,
  kCoarseBootstrap,
  kCoarsePeriodic
};

struct JointBackendConfig {
  bool enabled = false;
  JointBackendMode mode = JointBackendMode::kDisabled;
  double initial_voxel_size = 4.0;
  double minimum_voxel_size = 0.5;
  double downsample_size = 0.25;
  double planarity_ratio = 20.0;
  std::size_t minimum_points_per_voxel = 20;
  std::size_t maximum_points_per_observation = 4;
  std::size_t maximum_points_per_cloud = 12000;
  std::size_t minimum_keyframes = 20;
  std::size_t maximum_keyframes = 80;
  std::size_t bootstrap_frames = 250;
  std::size_t minimum_mixed_voxels_per_lidar = 50;
  double heldout_fraction = 0.2;
  double minimum_heldout_improvement = 0.05;
  std::size_t pose_outer_iterations = 2;
  std::size_t extrinsic_outer_iterations = 4;
  std::size_t joint_outer_iterations = 8;
  std::size_t solver_iterations = 30;
  double maximum_condition_number = 1e10;
  double minimum_relative_eigenvalue = 1e-8;
  double precise_max_rotation_update_deg = 1.0;
  double precise_max_translation_update_m = 0.10;
  double coarse_max_rotation_update_deg = 12.0;
  double coarse_max_translation_update_m = 0.75;
  double maximum_pose_rotation_update_deg = 10.0;
  double maximum_pose_translation_update_m = 2.0;
  std::size_t maximum_backend_passes = 2;
  std::uint64_t partition_seed = 42;
};

enum class CalibrationScenario {
  kPrecise,
  kCalibratedInit,
  kCoarse,
  kPriorFree
};
enum class CalibrationState {
  kInitializing,
  kObservable,
  kConverged,
  kNonConverged,
  kFailed
};

}  // namespace offline
}  // namespace mloam
