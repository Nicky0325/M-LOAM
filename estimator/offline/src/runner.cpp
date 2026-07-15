#include "mloam/offline/runner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace mloam {
namespace offline {
namespace {

std::string joinPath(const std::string& root, const std::string& path) {
  if (!path.empty() && path.front() == '/') return path;
  if (root.empty() || root.back() == '/') return root + path;
  return root + '/' + path;
}

double milliseconds(std::chrono::steady_clock::time_point begin) {
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - begin)
      .count();
}

double rotationError(const RigidTransform& estimate,
                     const RigidTransform& truth) {
  Eigen::Quaterniond difference = truth.rotation.conjugate() * estimate.rotation;
  difference.normalize();
  return Eigen::AngleAxisd(difference).angle() * 180.0 / M_PI;
}

}  // namespace

RunResult runOffline(const OfflineManifest& manifest,
                     const ScenarioConfiguration& scenario,
                     const std::string& output_directory,
                     const EstimatorCallback& estimator) {
  validateManifest(manifest);
  if (!estimator) throw std::invalid_argument("offline estimator callback is empty");

  std::vector<SequenceIndex> indexes;
  std::map<std::string, LidarConfig> configurations;
  for (const auto& lidar : manifest.lidars) {
    if (!lidar.enabled) continue;
    configurations[lidar.name] = lidar;
    auto index = SequenceIndex::fromDirectory(
        lidar.name, joinPath(manifest.dataset_root, lidar.directory),
        lidar.time_offset);
    if (index.scans().empty())
      throw std::invalid_argument("LiDAR directory contains no numeric PCD scans: " +
                                  lidar.name);
    indexes.push_back(std::move(index));
  }
  FrameSynchronizer synchronizer(manifest.reference_lidar,
                                 manifest.synchronization_threshold, indexes);
  const auto synchronized = synchronizer.match(
      manifest.frame_begin, manifest.frame_end, manifest.frame_stride);

  RunArtifacts artifacts;
  LidarMapper mapper(manifest.lidars, 0.05);
  CalibrationStateTracker calibration(1.0, 0.05);
  RunResult result;
  bool estimator_failed = false;
  bool all_converged = true;
  std::map<std::string, RigidTransform> final_extrinsics =
      scenario.initial_extrinsics;
  std::map<std::string, ExtrinsicRecord> latest_extrinsic_records;

  for (const auto& synchronized_frame : synchronized) {
    double max_raw_skew = 0.0;
    double max_corrected_skew = 0.0;
    for (const auto& item : synchronized_frame.scans) {
      max_raw_skew = std::max(max_raw_skew, std::abs(item.second.raw_skew));
      max_corrected_skew =
          std::max(max_corrected_skew, std::abs(item.second.corrected_skew));
    }
    if (!synchronized_frame.complete) {
      std::string missing;
      for (const auto& name : synchronized_frame.missing_lidars) {
        if (!missing.empty()) missing += ';';
        missing += name;
      }
      artifacts.addSynchronization(
          {synchronized_frame.reference_index,
           synchronized_frame.reference_timestamp, "dropped",
           "incomplete_set:" + missing, max_raw_skew, max_corrected_skew});
      ++result.dropped_frames;
      continue;
    }

    const auto preprocessing_begin = std::chrono::steady_clock::now();
    std::vector<PreparedLidarFrame> prepared;
    try {
      for (const auto& lidar : manifest.lidars) {
        if (!lidar.enabled) continue;
        auto frame = preparePcd(synchronized_frame.scans.at(lidar.name).path,
                                configurations.at(lidar.name),
                                manifest.minimum_finite_ratio);
        frame.scan_timestamp =
            synchronized_frame.scans.at(lidar.name).corrected_timestamp;
        prepared.push_back(std::move(frame));
      }
    } catch (const std::exception& error) {
      artifacts.addSynchronization(
          {synchronized_frame.reference_index,
           synchronized_frame.reference_timestamp, "dropped",
           std::string("corrupt_pcd:") + error.what(), max_raw_skew,
           max_corrected_skew});
      ++result.dropped_frames;
      continue;
    }
    const double preprocessing_ms = milliseconds(preprocessing_begin);

    EstimatorOutput estimate;
    const auto odometry_begin = std::chrono::steady_clock::now();
    try {
      estimate = estimator({synchronized_frame.reference_index,
                            synchronized_frame.reference_timestamp,
                            prepared, &scenario});
    } catch (const std::exception& error) {
      artifacts.addSynchronization(
          {synchronized_frame.reference_index,
           synchronized_frame.reference_timestamp, "failed",
           std::string("estimator_failure:") + error.what(), max_raw_skew,
           max_corrected_skew});
      estimator_failed = true;
      break;
    }
    const double callback_ms = milliseconds(odometry_begin);
    const double odometry_ms = estimate.odometry_ms >= 0.0
                                   ? estimate.odometry_ms : callback_ms;
    if (estimate.reference_T_lidar.empty())
      estimate.reference_T_lidar = scenario.initial_extrinsics;
    final_extrinsics = estimate.reference_T_lidar;
    artifacts.addOnlineFeatures(estimate.world_T_reference,
                                estimate.feature_points_reference,
                                manifest.lidars);

    const auto mapping_begin = std::chrono::steady_clock::now();
    MappingFrame mapping_frame;
    mapping_frame.frame_index = synchronized_frame.reference_index;
    mapping_frame.timestamp = synchronized_frame.reference_timestamp;
    mapping_frame.world_T_reference = estimate.world_T_reference;
    mapping_frame.lidars = prepared;
    mapping_frame.reference_T_lidar = estimate.reference_T_lidar;
    if (estimate.save_keyframe) mapper.processFrame(mapping_frame);
    const double mapping_ms = (estimate.mapping_ms >= 0.0
                                   ? estimate.mapping_ms : 0.0) +
                              milliseconds(mapping_begin);

    artifacts.addSynchronization(
        {synchronized_frame.reference_index,
         synchronized_frame.reference_timestamp, "processed", "",
         max_raw_skew, max_corrected_skew});
    artifacts.addRuntime({synchronized_frame.reference_index,
                          preprocessing_ms, odometry_ms, mapping_ms});
    artifacts.addTrajectory({synchronized_frame.reference_index,
                             synchronized_frame.reference_timestamp,
                             estimate.world_T_reference});
    for (const auto& lidar : manifest.lidars) {
      if (!lidar.enabled) continue;
      artifacts.addFeatures(
          {synchronized_frame.reference_index, lidar.name,
           estimate.corner_features[lidar.name],
           estimate.surface_features[lidar.name]});
      if (lidar.name == manifest.reference_lidar) continue;
      const auto estimated = estimate.reference_T_lidar.find(lidar.name);
      if (estimated == estimate.reference_T_lidar.end()) {
        estimator_failed = true;
        break;
      }
      const auto truth = scenario.precise_extrinsics.at(lidar.name);
      const double rotation_error = rotationError(estimated->second, truth);
      const double translation_error =
          (estimated->second.translation - truth.translation).norm();
      const bool observable =
          scenario.scenario == CalibrationScenario::kPrecise
              ? true
              : estimate.observable[lidar.name];
      calibration.update(lidar.name, synchronized_frame.reference_index,
                         synchronized_frame.reference_timestamp, observable,
                         rotation_error, translation_error, false);
      const auto state = calibration.history(lidar.name).back().state;
      ExtrinsicRecord record{synchronized_frame.reference_index,
                             synchronized_frame.reference_timestamp,
                             lidar.name, estimated->second, observable,
                             rotation_error, translation_error, state};
      artifacts.addExtrinsic(record);
      latest_extrinsic_records[lidar.name] = record;
    }
    if (estimator_failed) break;
    ++result.processed_frames;
  }

  if (estimator_failed || result.processed_frames == 0) {
    result.status = RunStatus::kFailed;
  } else {
    for (const auto& lidar : manifest.lidars) {
      if (!lidar.enabled || lidar.name == manifest.reference_lidar) continue;
      const auto latest = latest_extrinsic_records.find(lidar.name);
      if (latest == latest_extrinsic_records.end() ||
          latest->second.state != CalibrationState::kConverged) {
        all_converged = false;
      }
    }
    result.status = all_converged ? RunStatus::kSuccess
                                  : RunStatus::kNonConverged;
  }
  if (result.status == RunStatus::kNonConverged) {
    for (auto& item : latest_extrinsic_records) {
      if (item.second.state == CalibrationState::kConverged) continue;
      item.second.state = CalibrationState::kNonConverged;
      artifacts.addExtrinsic(item.second);
    }
  }
  artifacts.write(output_directory, manifest, scenario, mapper,
                  final_extrinsics, result.status);
  return result;
}

}  // namespace offline
}  // namespace mloam
