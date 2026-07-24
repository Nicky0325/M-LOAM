#include "mloam/offline/joint_backend.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <utility>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Eigenvalues>

namespace mloam {
namespace offline {
namespace {

using GridKey = std::tuple<long long, long long, long long>;
using ObservationKey = std::pair<std::size_t, std::size_t>;

struct PoseVariables {
  std::array<double, 4> q{{1.0, 0.0, 0.0, 0.0}};  // w, x, y, z
  std::array<double, 3> t{{0.0, 0.0, 0.0}};
};

struct Sample {
  std::size_t frame = 0;
  std::size_t lidar = 0;
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
};

struct PlaneLeaf {
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  double variance = 0.0;
  bool heldout = false;
  std::vector<std::size_t> samples;
  std::set<std::size_t> sensors;
};

struct PlaneFactor {
  std::size_t frame = 0;
  std::size_t lidar = 0;
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  double weight = 1.0;
  bool heldout = false;
};

struct VoxelModel {
  std::vector<PlaneLeaf> leaves;
  std::vector<PlaneFactor> factors;
  std::vector<std::set<std::size_t>> adjacency;
  std::vector<std::size_t> mixed_voxels;
  std::size_t training_voxels = 0;
  std::size_t heldout_voxels = 0;
  double training_objective = std::numeric_limits<double>::infinity();
  double heldout_objective = std::numeric_limits<double>::infinity();
};

struct CloudAccumulator {
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  std::size_t count = 0;
};

struct PlaneResidual {
  PlaneResidual(const PlaneFactor& factor)
      : point(factor.point),
        center(factor.center),
        normal(factor.normal),
        weight(factor.weight) {}

  template <typename T>
  bool operator()(const T* const pose_q, const T* const pose_t,
                  const T* const extrinsic_q, const T* const extrinsic_t,
                  T* residual) const {
    const T local[3] = {T(point.x()), T(point.y()), T(point.z())};
    T in_reference[3];
    ceres::QuaternionRotatePoint(extrinsic_q, local, in_reference);
    in_reference[0] += extrinsic_t[0];
    in_reference[1] += extrinsic_t[1];
    in_reference[2] += extrinsic_t[2];
    T in_world[3];
    ceres::QuaternionRotatePoint(pose_q, in_reference, in_world);
    in_world[0] += pose_t[0];
    in_world[1] += pose_t[1];
    in_world[2] += pose_t[2];
    residual[0] =
        T(weight) *
        (T(normal.x()) * (in_world[0] - T(center.x())) +
         T(normal.y()) * (in_world[1] - T(center.y())) +
         T(normal.z()) * (in_world[2] - T(center.z())));
    return true;
  }

  Eigen::Vector3d point;
  Eigen::Vector3d center;
  Eigen::Vector3d normal;
  double weight;
};

Eigen::Quaterniond quaternion(const PoseVariables& variables) {
  return Eigen::Quaterniond(variables.q[0], variables.q[1], variables.q[2],
                            variables.q[3]);
}

Eigen::Vector3d translation(const PoseVariables& variables) {
  return {variables.t[0], variables.t[1], variables.t[2]};
}

PoseVariables variables(const RigidTransform& transform) {
  PoseVariables result;
  Eigen::Quaterniond q = transform.rotation.normalized();
  result.q = {{q.w(), q.x(), q.y(), q.z()}};
  result.t = {{transform.translation.x(), transform.translation.y(),
               transform.translation.z()}};
  return result;
}

RigidTransform transform(const PoseVariables& variables) {
  RigidTransform result;
  result.rotation = quaternion(variables).normalized();
  result.translation = translation(variables);
  return result;
}

GridKey gridKey(const Eigen::Vector3d& point, double size) {
  return std::make_tuple(
      static_cast<long long>(std::floor(point.x() / size)),
      static_cast<long long>(std::floor(point.y() / size)),
      static_cast<long long>(std::floor(point.z() / size)));
}

Eigen::Matrix3d skew(const Eigen::Vector3d& point) {
  Eigen::Matrix3d result;
  result << 0.0, -point.z(), point.y(), point.z(), 0.0, -point.x(),
      -point.y(), point.x(), 0.0;
  return result;
}

double rotationDifferenceDegrees(const RigidTransform& lhs,
                                 const RigidTransform& rhs) {
  Eigen::Quaterniond difference = lhs.rotation.conjugate() * rhs.rotation;
  difference.normalize();
  return Eigen::AngleAxisd(difference).angle() * 180.0 / M_PI;
}

std::vector<Eigen::Vector3d> downsample(
    const std::vector<NativePointXYZIRT>& points,
    const JointBackendConfig& config) {
  std::map<GridKey, CloudAccumulator> voxels;
  for (const auto& point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z))
      continue;
    const Eigen::Vector3d value(point.x, point.y, point.z);
    auto& accumulator = voxels[gridKey(value, config.downsample_size)];
    accumulator.sum += value;
    ++accumulator.count;
  }
  std::vector<Eigen::Vector3d> result;
  result.reserve(std::min(voxels.size(), config.maximum_points_per_cloud));
  if (voxels.empty()) return result;
  const double stride =
      std::max(1.0, static_cast<double>(voxels.size()) /
                        static_cast<double>(config.maximum_points_per_cloud));
  double next = 0.0;
  std::size_t index = 0;
  for (const auto& item : voxels) {
    if (static_cast<double>(index++) + 1e-9 < next) continue;
    result.push_back(item.second.sum /
                     static_cast<double>(item.second.count));
    next += stride;
    if (result.size() == config.maximum_points_per_cloud) break;
  }
  return result;
}

std::vector<std::size_t> selectKeyframeIndices(std::size_t count,
                                                std::size_t maximum) {
  std::vector<std::size_t> result;
  if (count == 0) return result;
  const std::size_t selected = std::min(count, maximum);
  result.reserve(selected);
  if (selected == 1) {
    result.push_back(0);
    return result;
  }
  for (std::size_t i = 0; i < selected; ++i) {
    result.push_back(
        (i * (count - 1) + (selected - 1) / 2) / (selected - 1));
  }
  result.erase(std::unique(result.begin(), result.end()), result.end());
  return result;
}

std::uint64_t mixHash(std::uint64_t value) {
  value ^= value >> 30;
  value *= UINT64_C(0xbf58476d1ce4e5b9);
  value ^= value >> 27;
  value *= UINT64_C(0x94d049bb133111eb);
  value ^= value >> 31;
  return value;
}

class ModelBuilder {
 public:
  ModelBuilder(const std::vector<Sample>& samples,
               const std::vector<PoseVariables>& poses,
               const std::vector<PoseVariables>& extrinsics,
               const JointBackendConfig& config, std::size_t lidar_count)
      : samples_(samples),
        poses_(poses),
        extrinsics_(extrinsics),
        config_(config),
        transformed_(samples.size(), Eigen::Vector3d::Zero()),
        model_() {
    model_.adjacency.resize(lidar_count);
    model_.mixed_voxels.resize(lidar_count, 0);
    for (std::size_t i = 0; i < samples_.size(); ++i) {
      const auto& sample = samples_[i];
      transformed_[i] =
          quaternion(poses_[sample.frame]) *
              (quaternion(extrinsics_[sample.lidar]) * sample.point +
               translation(extrinsics_[sample.lidar])) +
          translation(poses_[sample.frame]);
    }
  }

  VoxelModel build(const std::vector<std::size_t>& subset) {
    std::map<GridKey, std::vector<std::size_t>> roots;
    for (const auto index : subset)
      roots[gridKey(transformed_[index], config_.initial_voxel_size)]
          .push_back(index);
    for (const auto& root : roots)
      cut(root.second, config_.initial_voxel_size, 0);
    finalize();
    return std::move(model_);
  }

 private:
  void cut(const std::vector<std::size_t>& indices, double size,
           std::size_t depth) {
    if (indices.size() < config_.minimum_points_per_voxel) return;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const auto index : indices) center += transformed_[index];
    center /= static_cast<double>(indices.size());
    for (const auto index : indices) {
      const Eigen::Vector3d centered = transformed_[index] - center;
      covariance += centered * centered.transpose();
    }
    covariance /= static_cast<double>(indices.size());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success) return;
    const auto eigenvalues = solver.eigenvalues();
    const double denominator = std::max(std::abs(eigenvalues[0]), 1e-12);
    const bool planar =
        eigenvalues[1] > 1e-10 &&
        eigenvalues[1] / denominator >= config_.planarity_ratio;
    if (planar) {
      addLeaf(indices, center, solver.eigenvectors().col(0),
              std::max(0.0, eigenvalues[0]), depth);
      return;
    }
    const double child_size = size * 0.5;
    if (child_size + 1e-12 < config_.minimum_voxel_size) return;
    std::map<GridKey, std::vector<std::size_t>> children;
    for (const auto index : indices)
      children[gridKey(transformed_[index], child_size)].push_back(index);
    if (children.size() <= 1) return;
    for (const auto& child : children) cut(child.second, child_size, depth + 1);
  }

  void addLeaf(const std::vector<std::size_t>& indices,
               const Eigen::Vector3d& center, Eigen::Vector3d normal,
               double variance, std::size_t depth) {
    std::map<ObservationKey, std::vector<std::size_t>> observations;
    std::set<std::size_t> sensors;
    for (const auto index : indices) {
      const auto& sample = samples_[index];
      observations[{sample.frame, sample.lidar}].push_back(index);
      sensors.insert(sample.lidar);
    }
    if (observations.size() < 2) return;
    PlaneLeaf leaf;
    leaf.center = center;
    if (normal.z() < 0.0) normal = -normal;
    leaf.normal = normal.normalized();
    leaf.variance = variance;
    leaf.sensors = sensors;
    std::uint64_t hash =
        mixHash(config_.partition_seed + depth + indices.size());
    for (int axis = 0; axis < 3; ++axis) {
      const auto quantized = static_cast<std::int64_t>(
          std::llround(center[axis] / config_.minimum_voxel_size));
      hash = mixHash(hash ^ static_cast<std::uint64_t>(quantized));
    }
    for (const auto sensor : sensors)
      hash = mixHash(hash ^ (sensor + UINT64_C(0x9e3779b97f4a7c15)));
    leaf.heldout =
        static_cast<double>(hash % UINT64_C(1000000)) / 1000000.0 <
        config_.heldout_fraction;
    for (const auto& observation : observations) {
      const auto& candidates = observation.second;
      const std::size_t keep =
          std::min(candidates.size(), config_.maximum_points_per_observation);
      for (std::size_t i = 0; i < keep; ++i) {
        const std::size_t candidate =
            candidates[(i * candidates.size()) / keep];
        leaf.samples.push_back(candidate);
      }
    }
    model_.leaves.push_back(std::move(leaf));
  }

  void finalize() {
    double training_sum = 0.0;
    double heldout_sum = 0.0;
    for (const auto& leaf : model_.leaves) {
      if (leaf.heldout) {
        ++model_.heldout_voxels;
        heldout_sum += leaf.variance;
      } else {
        ++model_.training_voxels;
        training_sum += leaf.variance;
      }
      if (leaf.sensors.size() > 1) {
        for (const auto lhs : leaf.sensors) {
          ++model_.mixed_voxels[lhs];
          for (const auto rhs : leaf.sensors) {
            if (lhs != rhs) model_.adjacency[lhs].insert(rhs);
          }
        }
      }
      const double weight =
          1.0 / std::sqrt(static_cast<double>(leaf.samples.size()));
      for (const auto index : leaf.samples) {
        const auto& sample = samples_[index];
        model_.factors.push_back({sample.frame, sample.lidar, sample.point,
                                  leaf.center, leaf.normal, weight,
                                  leaf.heldout});
      }
    }
    if (model_.training_voxels != 0)
      model_.training_objective =
          training_sum / static_cast<double>(model_.training_voxels);
    if (model_.heldout_voxels != 0)
      model_.heldout_objective =
          heldout_sum / static_cast<double>(model_.heldout_voxels);
  }

  const std::vector<Sample>& samples_;
  const std::vector<PoseVariables>& poses_;
  const std::vector<PoseVariables>& extrinsics_;
  const JointBackendConfig& config_;
  std::vector<Eigen::Vector3d> transformed_;
  VoxelModel model_;
};

VoxelModel buildModel(const std::vector<Sample>& samples,
                      const std::vector<PoseVariables>& poses,
                      const std::vector<PoseVariables>& extrinsics,
                      const JointBackendConfig& config,
                      std::size_t lidar_count, int only_lidar = -1) {
  std::vector<std::size_t> subset;
  subset.reserve(samples.size());
  for (std::size_t i = 0; i < samples.size(); ++i) {
    if (only_lidar >= 0 &&
        samples[i].lidar != static_cast<std::size_t>(only_lidar))
      continue;
    subset.push_back(i);
  }
  return ModelBuilder(samples, poses, extrinsics, config, lidar_count)
      .build(subset);
}

enum class SolverStage { kPoseOnly, kExtrinsicOnly, kJoint };

const char* solverStageName(SolverStage stage) {
  switch (stage) {
    case SolverStage::kPoseOnly: return "pose_only";
    case SolverStage::kExtrinsicOnly: return "extrinsic_only";
    case SolverStage::kJoint: return "joint";
  }
  return "unknown";
}

bool solveStage(const std::vector<Sample>& samples,
                std::vector<PoseVariables>* poses,
                std::vector<PoseVariables>* extrinsics,
                const JointBackendConfig& config, std::size_t reference_lidar,
                SolverStage stage, std::size_t outer_iterations,
                std::vector<BackendIteration>* history) {
  bool usable = false;
  for (std::size_t outer = 0; outer < outer_iterations; ++outer) {
    const int only_lidar =
        stage == SolverStage::kPoseOnly
            ? static_cast<int>(reference_lidar)
            : -1;
    const auto model =
        buildModel(samples, *poses, *extrinsics, config, extrinsics->size(),
                   only_lidar);
    BackendIteration iteration;
    iteration.stage = solverStageName(stage);
    iteration.outer_iteration = outer;
    iteration.planar_voxels = model.training_voxels;
    iteration.objective_before = model.training_objective;
    if (model.training_voxels == 0) {
      history->push_back(iteration);
      return false;
    }

    ceres::Problem problem;
    for (auto& pose : *poses) {
      problem.AddParameterBlock(pose.q.data(), 4,
                                new ceres::QuaternionParameterization());
      problem.AddParameterBlock(pose.t.data(), 3);
    }
    for (auto& extrinsic : *extrinsics) {
      problem.AddParameterBlock(extrinsic.q.data(), 4,
                                new ceres::QuaternionParameterization());
      problem.AddParameterBlock(extrinsic.t.data(), 3);
    }
    problem.SetParameterBlockConstant((*poses)[0].q.data());
    problem.SetParameterBlockConstant((*poses)[0].t.data());
    problem.SetParameterBlockConstant((*extrinsics)[reference_lidar].q.data());
    problem.SetParameterBlockConstant((*extrinsics)[reference_lidar].t.data());
    if (stage == SolverStage::kPoseOnly) {
      for (auto& extrinsic : *extrinsics) {
        problem.SetParameterBlockConstant(extrinsic.q.data());
        problem.SetParameterBlockConstant(extrinsic.t.data());
      }
    } else if (stage == SolverStage::kExtrinsicOnly) {
      for (auto& pose : *poses) {
        problem.SetParameterBlockConstant(pose.q.data());
        problem.SetParameterBlockConstant(pose.t.data());
      }
    }
    for (const auto& factor : model.factors) {
      if (factor.heldout) continue;
      PlaneFactor weighted_factor = factor;
      if (factor.lidar == reference_lidar &&
          stage != SolverStage::kPoseOnly)
        weighted_factor.weight *=
            std::sqrt(static_cast<double>(extrinsics->size()));
      auto* cost =
          new ceres::AutoDiffCostFunction<PlaneResidual, 1, 4, 3, 4, 3>(
              new PlaneResidual(weighted_factor));
      problem.AddResidualBlock(cost, nullptr, (*poses)[factor.frame].q.data(),
                               (*poses)[factor.frame].t.data(),
                               (*extrinsics)[factor.lidar].q.data(),
                               (*extrinsics)[factor.lidar].t.data());
      ++iteration.residuals;
    }
    if (iteration.residuals == 0) {
      history->push_back(iteration);
      return false;
    }
    ceres::Solver::Options options;
    options.max_num_iterations = static_cast<int>(config.solver_iterations);
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.num_threads = 4;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    for (auto& pose : *poses) {
      Eigen::Quaterniond normalized = quaternion(pose).normalized();
      pose.q = {{normalized.w(), normalized.x(), normalized.y(),
                 normalized.z()}};
    }
    for (auto& extrinsic : *extrinsics) {
      Eigen::Quaterniond normalized = quaternion(extrinsic).normalized();
      extrinsic.q = {{normalized.w(), normalized.x(), normalized.y(),
                      normalized.z()}};
    }
    const auto after =
        buildModel(samples, *poses, *extrinsics, config, extrinsics->size(),
                   only_lidar);
    iteration.objective_after = after.training_objective;
    iteration.usable =
        summary.IsSolutionUsable() && std::isfinite(iteration.objective_after);
    usable = usable || iteration.usable;
    history->push_back(iteration);
    if (!iteration.usable) return false;
  }
  return usable;
}

bool connectedToReference(const VoxelModel& model, std::size_t reference,
                          std::vector<bool>* connected) {
  connected->assign(model.adjacency.size(), false);
  std::vector<std::size_t> queue{reference};
  (*connected)[reference] = true;
  for (std::size_t head = 0; head < queue.size(); ++head) {
    for (const auto neighbor : model.adjacency[queue[head]]) {
      if ((*connected)[neighbor]) continue;
      (*connected)[neighbor] = true;
      queue.push_back(neighbor);
    }
  }
  return std::find(connected->begin(), connected->end(), false) ==
         connected->end();
}

double factorObjective(const VoxelModel& model,
                       const std::vector<PoseVariables>& poses,
                       const std::vector<PoseVariables>& extrinsics,
                       bool heldout) {
  double sum = 0.0;
  std::size_t count = 0;
  for (const auto& factor : model.factors) {
    if (factor.heldout != heldout) continue;
    const Eigen::Vector3d in_reference =
        quaternion(extrinsics[factor.lidar]) * factor.point +
        translation(extrinsics[factor.lidar]);
    const Eigen::Vector3d in_world =
        quaternion(poses[factor.frame]) * in_reference +
        translation(poses[factor.frame]);
    const double residual =
        factor.normal.dot(in_world - factor.center);
    sum += residual * residual;
    ++count;
  }
  return count == 0 ? std::numeric_limits<double>::infinity()
                    : sum / static_cast<double>(count);
}

void fillHessianDiagnostics(
    const VoxelModel& model, const std::vector<PoseVariables>& poses,
    const std::vector<PoseVariables>& extrinsics,
    const std::vector<std::string>& lidar_names, std::size_t reference,
    const JointBackendConfig& config, JointBackendResult* result) {
  std::vector<Eigen::Matrix<double, 6, 6>> hessians(
      extrinsics.size(), Eigen::Matrix<double, 6, 6>::Zero());
  for (const auto& factor : model.factors) {
    if (factor.heldout || factor.lidar == reference) continue;
    const Eigen::Matrix3d world_R_reference =
        quaternion(poses[factor.frame]).toRotationMatrix();
    const Eigen::Vector3d rotated =
        quaternion(extrinsics[factor.lidar]) * factor.point;
    Eigen::Matrix<double, 1, 6> jacobian;
    jacobian.block<1, 3>(0, 0) =
        -factor.weight * factor.normal.transpose() * world_R_reference *
        skew(rotated);
    jacobian.block<1, 3>(0, 3) =
        factor.weight * factor.normal.transpose() * world_R_reference;
    hessians[factor.lidar] += jacobian.transpose() * jacobian;
  }
  for (std::size_t lidar = 0; lidar < extrinsics.size(); ++lidar) {
    auto& diagnostics = result->lidar_diagnostics[lidar_names[lidar]];
    diagnostics.mixed_planar_voxels =
        lidar < model.mixed_voxels.size() ? model.mixed_voxels[lidar] : 0;
    if (lidar == reference) {
      diagnostics.connected = true;
      diagnostics.observable = true;
      diagnostics.condition_number = 1.0;
      continue;
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(
        hessians[lidar]);
    if (solver.info() != Eigen::Success) {
      diagnostics.condition_number =
          std::numeric_limits<double>::infinity();
      continue;
    }
    const auto values = solver.eigenvalues();
    diagnostics.minimum_hessian_eigenvalue = values[0];
    diagnostics.maximum_hessian_eigenvalue = values[5];
    diagnostics.condition_number =
        values[0] > 0.0
            ? values[5] / values[0]
            : std::numeric_limits<double>::infinity();
    diagnostics.observable =
        values[5] > 0.0 &&
        values[0] / values[5] >= config.minimum_relative_eigenvalue &&
        diagnostics.condition_number <= config.maximum_condition_number;
  }
}

}  // namespace

JointBackendResult JointCalibrationBackend::optimize(
    const LidarMapper& mapper, const std::string& reference_lidar,
    const std::map<std::string, RigidTransform>& initial_extrinsics,
    const JointBackendConfig& config) const {
  JointBackendResult result;
  result.optimized_extrinsics = initial_extrinsics;
  if (!config.enabled || config.mode == JointBackendMode::kDisabled) {
    result.state = JointBackendState::kDisabled;
    result.reason = "joint backend is disabled";
    return result;
  }
  if (initial_extrinsics.size() < 2) {
    result.state = JointBackendState::kInsufficientData;
    result.reason = "joint calibration requires at least two LiDARs";
    return result;
  }

  std::vector<std::string> lidar_names;
  lidar_names.reserve(initial_extrinsics.size());
  lidar_names.push_back(reference_lidar);
  for (const auto& item : initial_extrinsics) {
    if (item.first != reference_lidar) lidar_names.push_back(item.first);
  }
  const auto reference = initial_extrinsics.find(reference_lidar);
  if (reference == initial_extrinsics.end()) {
    result.state = JointBackendState::kFailed;
    result.reason = "reference LiDAR has no initial extrinsic";
    return result;
  }
  const auto& reference_keyframes = mapper.keyframes(reference_lidar);
  if (reference_keyframes.size() < config.minimum_keyframes) {
    result.state = JointBackendState::kInsufficientData;
    std::ostringstream reason;
    reason << "need at least " << config.minimum_keyframes
           << " keyframes, got " << reference_keyframes.size();
    result.reason = reason.str();
    return result;
  }
  const auto selected =
      selectKeyframeIndices(reference_keyframes.size(), config.maximum_keyframes);
  result.selected_keyframes = selected.size();

  std::map<std::size_t, std::size_t> frame_to_slot;
  std::vector<PoseVariables> poses;
  std::vector<RigidTransform> initial_poses;
  poses.reserve(selected.size());
  initial_poses.reserve(selected.size());
  for (std::size_t slot = 0; slot < selected.size(); ++slot) {
    const auto& keyframe = reference_keyframes[selected[slot]];
    frame_to_slot[keyframe.frame_index] = slot;
    poses.push_back(variables(keyframe.world_T_reference));
    initial_poses.push_back(keyframe.world_T_reference);
  }

  std::vector<PoseVariables> extrinsics;
  std::vector<RigidTransform> initial_extrinsic_vector;
  for (const auto& name : lidar_names) {
    const auto found = initial_extrinsics.find(name);
    if (found == initial_extrinsics.end()) {
      result.state = JointBackendState::kFailed;
      result.reason = "missing initial extrinsic for " + name;
      return result;
    }
    extrinsics.push_back(variables(found->second));
    initial_extrinsic_vector.push_back(found->second);
  }
  const std::size_t reference_index = 0;
  extrinsics[reference_index] = variables(RigidTransform());

  std::vector<Sample> samples;
  for (std::size_t lidar = 0; lidar < lidar_names.size(); ++lidar) {
    std::map<std::size_t, const SensorKeyframe*> by_frame;
    for (const auto& keyframe : mapper.keyframes(lidar_names[lidar]))
      by_frame[keyframe.frame_index] = &keyframe;
    for (const auto& frame : frame_to_slot) {
      const auto found = by_frame.find(frame.first);
      if (found == by_frame.end()) continue;
      const auto points = downsample(found->second->points, config);
      for (const auto& point : points)
        samples.push_back({frame.second, lidar, point});
    }
  }
  if (samples.empty()) {
    result.state = JointBackendState::kInsufficientData;
    result.reason = "selected keyframes contain no finite points";
    return result;
  }

  const auto initial_model =
      buildModel(samples, poses, extrinsics, config, lidar_names.size());
  result.training_voxels = initial_model.training_voxels;
  result.heldout_voxels = initial_model.heldout_voxels;
  result.initial_training_objective =
      factorObjective(initial_model, poses, extrinsics, false);
  result.initial_heldout_objective =
      factorObjective(initial_model, poses, extrinsics, true);
  const auto initial_reference_model =
      buildModel(samples, poses, extrinsics, config, lidar_names.size(),
                 static_cast<int>(reference_index));
  result.initial_reference_objective =
      initial_reference_model.training_objective;
  std::vector<bool> connected;
  const bool all_connected =
      connectedToReference(initial_model, reference_index, &connected);
  bool enough_overlap = all_connected;
  for (std::size_t lidar = 0; lidar < lidar_names.size(); ++lidar) {
    auto& diagnostics = result.lidar_diagnostics[lidar_names[lidar]];
    diagnostics.connected = connected[lidar];
    diagnostics.mixed_planar_voxels =
        initial_model.mixed_voxels[lidar];
    if (lidar != reference_index &&
        diagnostics.mixed_planar_voxels <
            config.minimum_mixed_voxels_per_lidar)
      enough_overlap = false;
  }
  if (initial_model.training_voxels == 0 ||
      initial_model.heldout_voxels == 0 || !enough_overlap) {
    result.state = JointBackendState::kInsufficientData;
    result.reason =
        !all_connected
            ? "mixed planar voxel graph is not connected"
            : "insufficient training/held-out mixed planar voxels";
    return result;
  }
  result.eligible = true;

  bool converged = solveStage(
      samples, &poses, &extrinsics, config, reference_index,
      SolverStage::kPoseOnly, config.pose_outer_iterations,
      &result.iterations);
  converged =
      solveStage(samples, &poses, &extrinsics, config, reference_index,
                 SolverStage::kExtrinsicOnly,
                 config.extrinsic_outer_iterations, &result.iterations) &&
      converged;
  converged =
      solveStage(samples, &poses, &extrinsics, config, reference_index,
                 SolverStage::kJoint, config.joint_outer_iterations,
                 &result.iterations) &&
      converged;
  result.converged = converged;
  if (!converged) {
    result.state = JointBackendState::kFailed;
    result.reason = "one or more optimization stages failed";
    return result;
  }

  const auto final_model =
      buildModel(samples, poses, extrinsics, config, lidar_names.size());
  result.final_training_objective =
      factorObjective(initial_model, poses, extrinsics, false);
  result.final_heldout_objective =
      factorObjective(initial_model, poses, extrinsics, true);
  const auto final_reference_model =
      buildModel(samples, poses, extrinsics, config, lidar_names.size(),
                 static_cast<int>(reference_index));
  result.final_reference_objective =
      final_reference_model.training_objective;
  if (std::isfinite(result.initial_heldout_objective) &&
      result.initial_heldout_objective > 1e-15 &&
      std::isfinite(result.final_heldout_objective)) {
    result.heldout_improvement =
        (result.initial_heldout_objective - result.final_heldout_objective) /
        result.initial_heldout_objective;
  }

  for (std::size_t lidar = 0; lidar < lidar_names.size(); ++lidar) {
    const auto optimized = transform(extrinsics[lidar]);
    result.optimized_extrinsics[lidar_names[lidar]] = optimized;
    auto& diagnostics = result.lidar_diagnostics[lidar_names[lidar]];
    diagnostics.rotation_update_deg =
        rotationDifferenceDegrees(initial_extrinsic_vector[lidar], optimized);
    diagnostics.translation_update_m =
        (initial_extrinsic_vector[lidar].translation -
         optimized.translation)
            .norm();
  }
  for (std::size_t slot = 0; slot < selected.size(); ++slot) {
    const auto optimized = transform(poses[slot]);
    result.optimized_keyframe_poses
        [reference_keyframes[selected[slot]].frame_index] = optimized;
    result.maximum_pose_rotation_update_deg =
        std::max(result.maximum_pose_rotation_update_deg,
                 rotationDifferenceDegrees(initial_poses[slot], optimized));
    result.maximum_pose_translation_update_m =
        std::max(result.maximum_pose_translation_update_m,
                 (initial_poses[slot].translation - optimized.translation)
                     .norm());
  }

  fillHessianDiagnostics(final_model, poses, extrinsics, lidar_names,
                         reference_index, config, &result);
  bool observable = true;
  double max_rotation = 0.0;
  double max_translation = 0.0;
  for (std::size_t lidar = 1; lidar < lidar_names.size(); ++lidar) {
    auto& diagnostics = result.lidar_diagnostics[lidar_names[lidar]];
    diagnostics.connected = connected[lidar];
    observable = observable && diagnostics.observable;
    max_rotation = std::max(max_rotation, diagnostics.rotation_update_deg);
    max_translation =
        std::max(max_translation, diagnostics.translation_update_m);
  }
  if (!observable) {
    result.state = JointBackendState::kDegenerate;
    result.reason = "extrinsic Hessian is rank-deficient or ill-conditioned";
    return result;
  }

  const bool coarse =
      config.mode == JointBackendMode::kCoarseBootstrap ||
      config.mode == JointBackendMode::kCoarsePeriodic;
  const double rotation_limit =
      coarse ? config.coarse_max_rotation_update_deg
             : config.precise_max_rotation_update_deg;
  const double translation_limit =
      coarse ? config.coarse_max_translation_update_m
             : config.precise_max_translation_update_m;
  const bool training_improved =
      std::isfinite(result.final_training_objective) &&
      result.final_training_objective < result.initial_training_objective;
  const bool heldout_improved =
      std::isfinite(result.final_heldout_objective) &&
      result.heldout_improvement >= config.minimum_heldout_improvement;
  const bool bounded =
      max_rotation <= rotation_limit && max_translation <= translation_limit &&
      result.maximum_pose_rotation_update_deg <=
          config.maximum_pose_rotation_update_deg &&
      result.maximum_pose_translation_update_m <=
          config.maximum_pose_translation_update_m;
  const bool reference_not_regressed =
      std::isfinite(result.initial_reference_objective) &&
      std::isfinite(result.final_reference_objective) &&
      result.final_reference_objective <=
          1.01 * result.initial_reference_objective;
  if (!training_improved || !heldout_improved || !reference_not_regressed ||
      !bounded) {
    result.state = JointBackendState::kRejected;
    if (!training_improved)
      result.reason = "training plane objective did not improve";
    else if (!heldout_improved)
      result.reason = "held-out plane objective did not improve enough";
    else if (!reference_not_regressed)
      result.reason = "reference-LiDAR map objective regressed";
    else
      result.reason = "pose or extrinsic update exceeded acceptance bounds";
    return result;
  }

  result.state = JointBackendState::kAccepted;
  result.reason = "combined overlap, validation, observability, and bounds passed";
  result.accepted = true;
  return result;
}

const char* jointBackendModeName(JointBackendMode mode) {
  switch (mode) {
    case JointBackendMode::kDisabled: return "disabled";
    case JointBackendMode::kPreciseRefine: return "precise_refine";
    case JointBackendMode::kCoarseBootstrap: return "coarse_bootstrap";
    case JointBackendMode::kCoarsePeriodic: return "coarse_periodic";
  }
  return "disabled";
}

const char* jointBackendStateName(JointBackendState state) {
  switch (state) {
    case JointBackendState::kDisabled: return "disabled";
    case JointBackendState::kInsufficientData: return "insufficient_data";
    case JointBackendState::kDegenerate: return "degenerate";
    case JointBackendState::kRejected: return "rejected";
    case JointBackendState::kAccepted: return "accepted";
    case JointBackendState::kFailed: return "failed";
  }
  return "failed";
}

}  // namespace offline
}  // namespace mloam
