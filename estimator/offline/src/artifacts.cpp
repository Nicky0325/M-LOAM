#include "mloam/offline/artifacts.hpp"
#include "mloam/offline/joint_backend.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>
#include <tuple>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace mloam {
namespace offline {
namespace {

void makeDirectories(const std::string& path) {
  std::string current;
  for (std::size_t i = 0; i < path.size(); ++i) {
    current.push_back(path[i]);
    if (path[i] != '/' && i + 1 != path.size()) continue;
    if (current == "/" || current.empty()) continue;
    if (mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
      throw std::runtime_error("cannot create output directory: " + current);
    }
  }
}

std::ofstream output(const std::string& path) {
  std::ofstream stream(path);
  if (!stream) throw std::runtime_error("cannot write artifact: " + path);
  stream << std::setprecision(17);
  return stream;
}

std::string csv(const std::string& value) {
  if (value.find_first_of(",\"\n") == std::string::npos) return value;
  std::string escaped = "\"";
  for (char character : value) {
    if (character == '"') escaped += '"';
    escaped += character;
  }
  return escaped + "\"";
}

void writeTransform(std::ostream& stream, const RigidTransform& transform) {
  stream << transform.translation.x() << ',' << transform.translation.y() << ','
         << transform.translation.z() << ',' << transform.rotation.x() << ','
         << transform.rotation.y() << ',' << transform.rotation.z() << ','
         << transform.rotation.w();
}

pcl::PointCloud<pcl::PointXYZRGB> coloredCloud(
    const std::vector<RgbPoint>& points, const std::uint8_t* lidar_filter) {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  for (const auto& point : points) {
    if (lidar_filter != nullptr && point.lidar_index != *lidar_filter) continue;
    pcl::PointXYZRGB output_point;
    output_point.x = point.x;
    output_point.y = point.y;
    output_point.z = point.z;
    output_point.r = point.r;
    output_point.g = point.g;
    output_point.b = point.b;
    cloud.push_back(output_point);
  }
  cloud.width = cloud.size();
  cloud.height = 1;
  return cloud;
}

void saveColored(const std::string& path, const std::vector<RgbPoint>& points,
                 const std::uint8_t* lidar_filter = nullptr) {
  const auto cloud = coloredCloud(points, lidar_filter);
  if (cloud.empty()) {
    auto stream = output(path);
    stream << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z rgb\n"
           << "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
           << "WIDTH 0\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
           << "POINTS 0\nDATA ascii\n";
    return;
  }
  if (pcl::io::savePCDFileBinaryCompressed(path, cloud) < 0)
    throw std::runtime_error("cannot write PCD artifact: " + path);
}

void saveKeyframe(const std::string& path, const SensorKeyframe& keyframe,
                  double voxel_size) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  std::set<std::tuple<long long, long long, long long>> occupied;
  for (const auto& point : keyframe.points) {
    const auto voxel = std::make_tuple(
        static_cast<long long>(std::floor(point.x / voxel_size)),
        static_cast<long long>(std::floor(point.y / voxel_size)),
        static_cast<long long>(std::floor(point.z / voxel_size)));
    if (!occupied.insert(voxel).second) continue;
    pcl::PointXYZI output_point;
    output_point.x = point.x;
    output_point.y = point.y;
    output_point.z = point.z;
    output_point.intensity = point.reflectivity;
    cloud.push_back(output_point);
  }
  cloud.width = cloud.size();
  cloud.height = 1;
  if (pcl::io::savePCDFileBinaryCompressed(path, cloud) < 0)
    throw std::runtime_error("cannot write keyframe PCD: " + path);
}

std::pair<double, double> alignmentMetrics(
    const std::vector<RgbPoint>& points) {
  using Voxel = std::tuple<long long, long long, long long>;
  std::map<Voxel, std::vector<std::size_t>> voxels;
  constexpr double kOverlapVoxelSize = 1.0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    voxels[std::make_tuple(
        static_cast<long long>(std::floor(points[i].x / kOverlapVoxelSize)),
        static_cast<long long>(std::floor(points[i].y / kOverlapVoxelSize)),
        static_cast<long long>(std::floor(points[i].z / kOverlapVoxelSize)))]
        .push_back(i);
  }
  std::vector<double> distances;
  for (const auto& voxel : voxels) {
    std::set<std::uint8_t> sensors;
    for (const auto index : voxel.second)
      sensors.insert(points[index].lidar_index);
    if (sensors.size() < 2) continue;
    for (const auto i : voxel.second) {
      double nearest = std::numeric_limits<double>::infinity();
      for (const auto j : voxel.second) {
        if (points[i].lidar_index == points[j].lidar_index) continue;
        const double dx = points[i].x - points[j].x;
        const double dy = points[i].y - points[j].y;
        const double dz = points[i].z - points[j].z;
        nearest = std::min(nearest, std::sqrt(dx * dx + dy * dy + dz * dz));
      }
      if (std::isfinite(nearest)) distances.push_back(nearest);
    }
  }
  if (distances.empty()) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    return {nan, nan};
  }
  std::sort(distances.begin(), distances.end());
  const auto percentile = [&](double p) {
    const std::size_t index = static_cast<std::size_t>(
        std::ceil(p * static_cast<double>(distances.size()))) - 1;
    return distances[std::min(index, distances.size() - 1)];
  };
  return {percentile(0.5), percentile(0.95)};
}

}  // namespace

void RunArtifacts::addSynchronization(SynchronizationRecord record) {
  synchronization_.push_back(std::move(record));
}
void RunArtifacts::addRuntime(RuntimeRecord record) {
  runtimes_.push_back(std::move(record));
}
void RunArtifacts::addFeatures(FeatureRecord record) {
  features_.push_back(std::move(record));
}
void RunArtifacts::addTrajectory(TrajectoryRecord record) {
  trajectory_.push_back(std::move(record));
}
void RunArtifacts::addExtrinsic(ExtrinsicRecord record) {
  extrinsics_.push_back(std::move(record));
}
void RunArtifacts::addOnlineFeatures(
    const RigidTransform& world_T_reference,
    const std::map<std::string, std::vector<EncodedPointXYZI>>& features,
    const std::vector<LidarConfig>& lidars) {
  std::size_t lidar_index = 0;
  for (const auto& lidar : lidars) {
    if (!lidar.enabled) continue;
    if (lidar_index >= 256)
      throw std::invalid_argument("at most 256 LiDARs are supported");
    const auto compact_index = static_cast<std::uint8_t>(lidar_index++);
    const auto sensor_features = features.find(lidar.name);
    if (sensor_features == features.end()) continue;
    for (const auto& point : sensor_features->second) {
      const Eigen::Vector3d world = world_T_reference.rotation *
                                        Eigen::Vector3d(point.x, point.y, point.z) +
                                    world_T_reference.translation;
      online_features_.push_back(
          {static_cast<float>(world.x()), static_cast<float>(world.y()),
           static_cast<float>(world.z()), lidar.color.r, lidar.color.g,
           lidar.color.b, compact_index});
    }
  }
}

void RunArtifacts::write(
    const std::string& output_directory, const OfflineManifest& manifest,
    const ScenarioConfiguration& scenario, const LidarMapper& mapper,
    const std::map<std::string, RigidTransform>& final_extrinsics,
    RunStatus status, const JointBackendResult* backend) const {
  makeDirectories(output_directory);
  makeDirectories(output_directory + "/keyframes");
  for (const auto& lidar : manifest.lidars) {
    if (lidar.enabled)
      makeDirectories(output_directory + "/keyframes/" + lidar.name);
  }

  auto resolved = output(output_directory + "/resolved_config.yaml");
  resolved << "dataset_root: " << manifest.dataset_root << '\n'
           << "output_root: " << output_directory << '\n'
           << "mloam_config: " << manifest.mloam_config << '\n'
           << "reference_lidar: " << manifest.reference_lidar << '\n'
           << "synchronization_threshold: "
           << manifest.synchronization_threshold << '\n'
           << "frame_begin: " << manifest.frame_begin << '\n'
           << "frame_end: " << manifest.frame_end << '\n'
           << "frame_stride: " << manifest.frame_stride << '\n'
           << "scenario: " << scenario.name << '\n'
           << "estimator_mode: " << scenario.estimator_mode << '\n'
           << "seed: " << scenario.seed << '\n'
           << "rotation_perturbation_deg: "
           << scenario.rotation_perturbation_deg << '\n'
           << "translation_perturbation_m: "
           << scenario.translation_perturbation_m << '\n'
           << "joint_backend:\n"
           << "  enabled: "
           << (manifest.joint_backend.enabled ? "true" : "false") << '\n'
           << "  mode: "
           << jointBackendModeName(manifest.joint_backend.mode) << '\n'
           << "  initial_voxel_size: "
           << manifest.joint_backend.initial_voxel_size << '\n'
           << "  minimum_voxel_size: "
           << manifest.joint_backend.minimum_voxel_size << '\n'
           << "  downsample_size: "
           << manifest.joint_backend.downsample_size << '\n'
           << "  planarity_ratio: "
           << manifest.joint_backend.planarity_ratio << '\n'
           << "  minimum_points_per_voxel: "
           << manifest.joint_backend.minimum_points_per_voxel << '\n'
           << "  maximum_points_per_observation: "
           << manifest.joint_backend.maximum_points_per_observation << '\n'
           << "  maximum_points_per_cloud: "
           << manifest.joint_backend.maximum_points_per_cloud << '\n'
           << "  minimum_keyframes: "
           << manifest.joint_backend.minimum_keyframes << '\n'
           << "  maximum_keyframes: "
           << manifest.joint_backend.maximum_keyframes << '\n'
           << "  bootstrap_frames: "
           << manifest.joint_backend.bootstrap_frames << '\n'
           << "  minimum_mixed_voxels_per_lidar: "
           << manifest.joint_backend.minimum_mixed_voxels_per_lidar << '\n'
           << "  heldout_fraction: "
           << manifest.joint_backend.heldout_fraction << '\n'
           << "  minimum_heldout_improvement: "
           << manifest.joint_backend.minimum_heldout_improvement << '\n'
           << "  pose_outer_iterations: "
           << manifest.joint_backend.pose_outer_iterations << '\n'
           << "  extrinsic_outer_iterations: "
           << manifest.joint_backend.extrinsic_outer_iterations << '\n'
           << "  joint_outer_iterations: "
           << manifest.joint_backend.joint_outer_iterations << '\n'
           << "  solver_iterations: "
           << manifest.joint_backend.solver_iterations << '\n'
           << "  maximum_condition_number: "
           << manifest.joint_backend.maximum_condition_number << '\n'
           << "  minimum_relative_eigenvalue: "
           << manifest.joint_backend.minimum_relative_eigenvalue << '\n'
           << "  precise_max_rotation_update_deg: "
           << manifest.joint_backend.precise_max_rotation_update_deg << '\n'
           << "  precise_max_translation_update_m: "
           << manifest.joint_backend.precise_max_translation_update_m << '\n'
           << "  coarse_max_rotation_update_deg: "
           << manifest.joint_backend.coarse_max_rotation_update_deg << '\n'
           << "  coarse_max_translation_update_m: "
           << manifest.joint_backend.coarse_max_translation_update_m << '\n'
           << "  maximum_pose_rotation_update_deg: "
           << manifest.joint_backend.maximum_pose_rotation_update_deg << '\n'
           << "  maximum_pose_translation_update_m: "
           << manifest.joint_backend.maximum_pose_translation_update_m << '\n'
           << "  maximum_backend_passes: "
           << manifest.joint_backend.maximum_backend_passes << '\n'
           << "  partition_seed: "
           << manifest.joint_backend.partition_seed << '\n'
           << "injected_perturbations:\n";
  for (const auto& item : scenario.injected_perturbations) {
    resolved << "  " << item.first << ": [";
    writeTransform(resolved, item.second);
    resolved << "]\n";
  }
  resolved << "precise_reference_T_lidar:\n";
  for (const auto& item : scenario.precise_extrinsics) {
    resolved << "  " << item.first << ": [";
    writeTransform(resolved, item.second);
    resolved << "]\n";
  }
  resolved << "initial_reference_T_lidar:\n";
  for (const auto& item : scenario.initial_extrinsics) {
    resolved << "  " << item.first << ": [";
    writeTransform(resolved, item.second);
    resolved << "]\n";
  }
  resolved
           << "lidars:\n";
  for (const auto& lidar : manifest.lidars) {
    resolved << "  - name: " << lidar.name << '\n'
             << "    enabled: " << (lidar.enabled ? "true" : "false") << '\n'
             << "    directory: " << lidar.directory << '\n'
             << "    rings: " << lidar.ring_count << '\n'
             << "    horizontal_fov_deg: " << lidar.horizontal_fov_deg << '\n'
             << "    expected_width: " << lidar.expected_width << '\n'
             << "    timestamp_scale: " << lidar.timestamp_scale << '\n'
             << "    scan_period: " << lidar.scan_period << '\n'
             << "    time_offset: " << lidar.time_offset << '\n'
             << "    color: [" << static_cast<int>(lidar.color.r) << ", "
             << static_cast<int>(lidar.color.g) << ", "
             << static_cast<int>(lidar.color.b) << "]\n"
             << "    segment_cloud: "
             << (lidar.segment_cloud ? "true" : "false") << '\n'
             << "    min_cluster_size: " << lidar.min_cluster_size << '\n'
             << "    segment_valid_point_num: "
             << lidar.segment_valid_point_num << '\n'
             << "    segment_valid_line_num: "
             << lidar.segment_valid_line_num << '\n'
             << "    segment_theta: " << lidar.segment_theta_rad << '\n'
             << "    vehicle_T_lidar: [";
    writeTransform(resolved, lidar.vehicle_T_lidar);
    resolved << "]\n";
  }

  const auto reference = std::find_if(
      manifest.lidars.begin(), manifest.lidars.end(),
      [&](const LidarConfig& lidar) {
        return lidar.name == manifest.reference_lidar;
      });
  if (reference == manifest.lidars.end())
    throw std::runtime_error("cannot resolve reference LiDAR for artifacts");
  auto optimized = output(output_directory + "/optimized_extrinsics.yaml");
  optimized << "convention: [tx, ty, tz, qx, qy, qz, qw]\n"
            << "reference_lidar: " << manifest.reference_lidar << '\n'
            << "reference_T_lidar:\n";
  for (const auto& item : final_extrinsics) {
    optimized << "  " << item.first << ": [";
    writeTransform(optimized, item.second);
    optimized << "]\n";
  }
  optimized << "vehicle_T_lidar:\n";
  for (const auto& item : final_extrinsics) {
    optimized << "  " << item.first << ": [";
    writeTransform(optimized,
                   reference->vehicle_T_lidar * item.second);
    optimized << "]\n";
  }

  auto sync = output(output_directory + "/synchronization.csv");
  sync << "frame,timestamp,status,reason,max_raw_skew_s,max_corrected_skew_s\n";
  for (const auto& record : synchronization_) {
    sync << record.frame << ',' << record.timestamp << ',' << record.status
         << ',' << csv(record.reason) << ',' << record.max_raw_skew_s << ','
         << record.max_corrected_skew_s << '\n';
  }
  auto runtime = output(output_directory + "/runtime.csv");
  runtime << "frame,preprocessing_ms,odometry_ms,mapping_ms\n";
  for (const auto& record : runtimes_)
    runtime << record.frame << ',' << record.preprocessing_ms << ','
            << record.odometry_ms << ',' << record.mapping_ms << '\n';
  auto features = output(output_directory + "/features.csv");
  features << "frame,lidar,corner_count,surface_count\n";
  for (const auto& record : features_)
    features << record.frame << ',' << record.lidar_name << ','
             << record.corner_count << ',' << record.surface_count << '\n';
  auto trajectory = output(output_directory + "/trajectory.csv");
  trajectory << "frame,timestamp,tx,ty,tz,qx,qy,qz,qw\n";
  for (const auto& record : trajectory_) {
    trajectory << record.frame << ',' << record.timestamp << ',';
    writeTransform(trajectory, record.world_T_reference);
    trajectory << '\n';
  }
  auto corrected_trajectory =
      output(output_directory + "/trajectory_corrected.csv");
  corrected_trajectory << "frame,timestamp,tx,ty,tz,qx,qy,qz,qw\n";
  for (const auto& record : trajectory_) {
    const RigidTransform corrected =
        backend != nullptr && backend->accepted
            ? mapper.correctedReferencePose(
                  record.frame, record.world_T_reference,
                  backend->optimized_keyframe_poses)
            : record.world_T_reference;
    corrected_trajectory << record.frame << ',' << record.timestamp << ',';
    writeTransform(corrected_trajectory, corrected);
    corrected_trajectory << '\n';
  }
  auto extrinsics = output(output_directory + "/extrinsics_history.csv");
  extrinsics << "frame,timestamp,lidar,tx,ty,tz,qx,qy,qz,qw,rotation_error_deg,translation_error_m,state\n";
  auto observability = output(output_directory + "/observability_history.csv");
  observability << "frame,timestamp,lidar,observable,state\n";
  for (const auto& record : extrinsics_) {
    extrinsics << record.frame << ',' << record.timestamp << ','
               << record.lidar_name << ',';
    writeTransform(extrinsics, record.reference_T_lidar);
    extrinsics << ',' << record.rotation_error_deg << ','
               << record.translation_error_m << ','
               << calibrationStateName(record.state) << '\n';
    observability << record.frame << ',' << record.timestamp << ','
                  << record.lidar_name << ','
                  << (record.observable ? "true" : "false") << ','
                  << calibrationStateName(record.state) << '\n';
  }

  const auto final_map =
      backend != nullptr && backend->accepted
          ? mapper.rebuildCorrectedRgbMap(
                final_extrinsics, backend->optimized_keyframe_poses)
          : mapper.rebuildFinalRgbMap(final_extrinsics);
  const auto online_map = mapper.buildOnlineRgbMap();
  saveColored(output_directory + "/map_online_rgb.pcd", online_map);
  saveColored(output_directory + "/map_online_features_rgb.pcd",
              online_features_);
  saveColored(output_directory + "/map_merged_rgb.pcd", online_map);
  saveColored(output_directory + "/map_final_rgb.pcd", final_map);
  saveColored(output_directory + "/map_backend_corrected_rgb.pcd", final_map);
  std::size_t lidar_index = 0;
  for (const auto& lidar : manifest.lidars) {
    if (!lidar.enabled) continue;
    const auto compact_index = static_cast<std::uint8_t>(lidar_index++);
    saveColored(output_directory + "/map_" + lidar.name + ".pcd",
                final_map, &compact_index);
    for (const auto& keyframe : mapper.keyframes(lidar.name)) {
      std::ostringstream filename;
      filename << output_directory << "/keyframes/" << lidar.name << '/'
               << std::setw(6) << std::setfill('0') << keyframe.frame_index
               << ".pcd";
      saveKeyframe(filename.str(), keyframe, 0.05);
    }
  }

  const auto metrics = alignmentMetrics(final_map);
  if (backend != nullptr) {
    auto backend_windows = output(output_directory + "/backend_windows.csv");
    backend_windows
        << "stage,outer_iteration,planar_voxels,residuals,"
           "objective_before,objective_after,usable\n";
    for (const auto& iteration : backend->iterations) {
      backend_windows << iteration.stage << ',' << iteration.outer_iteration
                      << ',' << iteration.planar_voxels << ','
                      << iteration.residuals << ','
                      << iteration.objective_before << ','
                      << iteration.objective_after << ','
                      << (iteration.usable ? "true" : "false") << '\n';
    }
    auto backend_history =
        output(output_directory + "/backend_extrinsics_history.csv");
    backend_history
        << "lidar,tx,ty,tz,qx,qy,qz,qw,rotation_update_deg,"
           "translation_update_m,mixed_planar_voxels,"
           "minimum_hessian_eigenvalue,maximum_hessian_eigenvalue,"
           "condition_number,connected,observable\n";
    for (const auto& item : backend->optimized_extrinsics) {
      const auto diagnostics = backend->lidar_diagnostics.find(item.first);
      backend_history << item.first << ',';
      writeTransform(backend_history, item.second);
      if (diagnostics == backend->lidar_diagnostics.end()) {
        backend_history << ",nan,nan,0,nan,nan,nan,false,false\n";
      } else {
        const auto& value = diagnostics->second;
        backend_history << ',' << value.rotation_update_deg << ','
                        << value.translation_update_m << ','
                        << value.mixed_planar_voxels << ','
                        << value.minimum_hessian_eigenvalue << ','
                        << value.maximum_hessian_eigenvalue << ','
                        << value.condition_number << ','
                        << (value.connected ? "true" : "false") << ','
                        << (value.observable ? "true" : "false") << '\n';
      }
    }
    auto diagnostics = output(output_directory + "/backend_diagnostics.yaml");
    diagnostics << "state: " << jointBackendStateName(backend->state) << '\n'
                << "reason: " << csv(backend->reason) << '\n'
                << "eligible: " << (backend->eligible ? "true" : "false")
                << '\n'
                << "converged: "
                << (backend->converged ? "true" : "false") << '\n'
                << "accepted: "
                << (backend->accepted ? "true" : "false") << '\n'
                << "selected_keyframes: " << backend->selected_keyframes
                << '\n'
                << "training_voxels: " << backend->training_voxels << '\n'
                << "heldout_voxels: " << backend->heldout_voxels << '\n'
                << "initial_training_objective: "
                << backend->initial_training_objective << '\n'
                << "final_training_objective: "
                << backend->final_training_objective << '\n'
                << "initial_heldout_objective: "
                << backend->initial_heldout_objective << '\n'
                << "final_heldout_objective: "
                << backend->final_heldout_objective << '\n'
                << "initial_reference_objective: "
                << backend->initial_reference_objective << '\n'
                << "final_reference_objective: "
                << backend->final_reference_objective << '\n'
                << "heldout_improvement: "
                << backend->heldout_improvement << '\n'
                << "maximum_pose_rotation_update_deg: "
                << backend->maximum_pose_rotation_update_deg << '\n'
                << "maximum_pose_translation_update_m: "
                << backend->maximum_pose_translation_update_m << '\n'
                << "lidars:\n";
    for (const auto& item : backend->lidar_diagnostics) {
      const auto& value = item.second;
      diagnostics << "  " << item.first << ":\n"
                  << "    mixed_planar_voxels: "
                  << value.mixed_planar_voxels << '\n'
                  << "    minimum_hessian_eigenvalue: "
                  << value.minimum_hessian_eigenvalue << '\n'
                  << "    maximum_hessian_eigenvalue: "
                  << value.maximum_hessian_eigenvalue << '\n'
                  << "    condition_number: " << value.condition_number
                  << '\n'
                  << "    rotation_update_deg: "
                  << value.rotation_update_deg << '\n'
                  << "    translation_update_m: "
                  << value.translation_update_m << '\n'
                  << "    connected: "
                  << (value.connected ? "true" : "false") << '\n'
                  << "    observable: "
                  << (value.observable ? "true" : "false") << '\n';
    }
  }
  auto summary = output(output_directory + "/summary.yaml");
  summary << "status: " << runStatusName(status) << '\n'
          << "scenario: " << scenario.name << '\n'
          << "processed_frames: " << trajectory_.size() << '\n'
          << "synchronization_records: " << synchronization_.size() << '\n'
          << "alignment_median_m: " << metrics.first << '\n'
          << "alignment_p95_m: " << metrics.second << '\n'
          << "backend_state: "
          << (backend == nullptr ? "disabled"
                                 : jointBackendStateName(backend->state))
          << '\n'
          << "backend_accepted: "
          << (backend != nullptr && backend->accepted ? "true" : "false")
          << '\n'
          << "extrinsic_errors:\n";
  for (const auto& lidar : manifest.lidars) {
    if (!lidar.enabled || lidar.name == manifest.reference_lidar) continue;
    const auto latest = std::find_if(extrinsics_.rbegin(), extrinsics_.rend(),
        [&](const ExtrinsicRecord& record) { return record.lidar_name == lidar.name; });
    summary << "  " << lidar.name << ":\n";
    if (latest != extrinsics_.rend()) {
      summary << "    rotation_deg: " << latest->rotation_error_deg << '\n'
              << "    translation_m: " << latest->translation_error_m << '\n'
              << "    state: " << calibrationStateName(latest->state) << '\n';
      const auto convergence = std::find_if(
          extrinsics_.begin(), extrinsics_.end(),
          [&](const ExtrinsicRecord& record) {
            return record.lidar_name == lidar.name &&
                   record.state == CalibrationState::kConverged;
          });
      if (convergence == extrinsics_.end()) {
        summary << "    convergence_frame: null\n"
                << "    convergence_time: null\n";
      } else {
        summary << "    convergence_frame: " << convergence->frame << '\n'
                << "    convergence_time: " << convergence->timestamp << '\n';
      }
    }
  }
}

const char* runStatusName(RunStatus status) {
  switch (status) {
    case RunStatus::kSuccess: return "success";
    case RunStatus::kNonConverged: return "non_converged";
    case RunStatus::kFailed: return "failed";
  }
  return "failed";
}

}  // namespace offline
}  // namespace mloam
