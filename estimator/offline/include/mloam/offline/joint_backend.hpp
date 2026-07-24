#pragma once

#include <cstddef>
#include <map>
#include <string>
#include <vector>

#include "mloam/offline/lidar_mapper.hpp"

namespace mloam {
namespace offline {

enum class JointBackendState {
  kDisabled,
  kInsufficientData,
  kDegenerate,
  kRejected,
  kAccepted,
  kFailed
};

struct BackendLidarDiagnostics {
  std::size_t mixed_planar_voxels = 0;
  double minimum_hessian_eigenvalue = 0.0;
  double maximum_hessian_eigenvalue = 0.0;
  double condition_number = 0.0;
  double rotation_update_deg = 0.0;
  double translation_update_m = 0.0;
  bool connected = false;
  bool observable = false;
};

struct BackendIteration {
  std::string stage;
  std::size_t outer_iteration = 0;
  std::size_t planar_voxels = 0;
  std::size_t residuals = 0;
  double objective_before = 0.0;
  double objective_after = 0.0;
  bool usable = false;
};

struct JointBackendResult {
  JointBackendState state = JointBackendState::kDisabled;
  std::string reason;
  bool eligible = false;
  bool converged = false;
  bool accepted = false;
  std::size_t selected_keyframes = 0;
  std::size_t training_voxels = 0;
  std::size_t heldout_voxels = 0;
  double initial_training_objective = 0.0;
  double final_training_objective = 0.0;
  double initial_heldout_objective = 0.0;
  double final_heldout_objective = 0.0;
  double initial_reference_objective = 0.0;
  double final_reference_objective = 0.0;
  double heldout_improvement = 0.0;
  double maximum_pose_rotation_update_deg = 0.0;
  double maximum_pose_translation_update_m = 0.0;
  std::map<std::string, RigidTransform> optimized_extrinsics;
  std::map<std::size_t, RigidTransform> optimized_keyframe_poses;
  std::map<std::string, BackendLidarDiagnostics> lidar_diagnostics;
  std::vector<BackendIteration> iterations;
};

class JointCalibrationBackend {
 public:
  JointBackendResult optimize(
      const LidarMapper& mapper, const std::string& reference_lidar,
      const std::map<std::string, RigidTransform>& initial_extrinsics,
      const JointBackendConfig& config) const;
};

const char* jointBackendModeName(JointBackendMode mode);
const char* jointBackendStateName(JointBackendState state);

}  // namespace offline
}  // namespace mloam
