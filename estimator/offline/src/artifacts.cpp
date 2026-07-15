#include "mloam/offline/artifacts.hpp"

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
    const std::vector<RgbPoint>& points, const std::string* lidar_filter) {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  for (const auto& point : points) {
    if (lidar_filter != nullptr && point.lidar_name != *lidar_filter) continue;
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
                 const std::string* lidar_filter = nullptr) {
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
    std::set<std::string> sensors;
    for (const auto index : voxel.second) sensors.insert(points[index].lidar_name);
    if (sensors.size() < 2) continue;
    for (const auto i : voxel.second) {
      double nearest = std::numeric_limits<double>::infinity();
      for (const auto j : voxel.second) {
        if (points[i].lidar_name == points[j].lidar_name) continue;
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
  for (const auto& lidar : lidars) {
    if (!lidar.enabled) continue;
    const auto sensor_features = features.find(lidar.name);
    if (sensor_features == features.end()) continue;
    for (const auto& point : sensor_features->second) {
      const Eigen::Vector3d world = world_T_reference.rotation *
                                        Eigen::Vector3d(point.x, point.y, point.z) +
                                    world_T_reference.translation;
      online_features_.push_back(
          {static_cast<float>(world.x()), static_cast<float>(world.y()),
           static_cast<float>(world.z()), lidar.color.r, lidar.color.g,
           lidar.color.b, lidar.name});
    }
  }
}

void RunArtifacts::write(
    const std::string& output_directory, const OfflineManifest& manifest,
    const ScenarioConfiguration& scenario, const LidarMapper& mapper,
    const std::map<std::string, RigidTransform>& final_extrinsics,
    RunStatus status) const {
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

  const auto final_map = mapper.rebuildFinalRgbMap(final_extrinsics);
  const auto online_map = mapper.buildOnlineRgbMap();
  saveColored(output_directory + "/map_online_rgb.pcd", online_map);
  saveColored(output_directory + "/map_online_features_rgb.pcd",
              online_features_);
  saveColored(output_directory + "/map_merged_rgb.pcd", online_map);
  saveColored(output_directory + "/map_final_rgb.pcd", final_map);
  for (const auto& lidar : manifest.lidars) {
    if (!lidar.enabled) continue;
    saveColored(output_directory + "/map_" + lidar.name + ".pcd",
                final_map, &lidar.name);
    for (const auto& keyframe : mapper.keyframes(lidar.name)) {
      std::ostringstream filename;
      filename << output_directory << "/keyframes/" << lidar.name << '/'
               << std::setw(6) << std::setfill('0') << keyframe.frame_index
               << ".pcd";
      saveKeyframe(filename.str(), keyframe, 0.05);
    }
  }

  const auto metrics = alignmentMetrics(final_map);
  auto summary = output(output_directory + "/summary.yaml");
  summary << "status: " << runStatusName(status) << '\n'
          << "scenario: " << scenario.name << '\n'
          << "processed_frames: " << trajectory_.size() << '\n'
          << "synchronization_records: " << synchronization_.size() << '\n'
          << "alignment_median_m: " << metrics.first << '\n'
          << "alignment_p95_m: " << metrics.second << '\n'
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
