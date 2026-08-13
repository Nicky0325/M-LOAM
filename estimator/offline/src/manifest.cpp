#include "mloam/offline/manifest.hpp"

#include <cmath>
#include <fstream>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace mloam {
namespace offline {
namespace {

template <typename T>
T required(const YAML::Node& node, const char* key) {
  if (!node[key]) {
    throw std::invalid_argument(std::string("missing required manifest key: ") +
                                key);
  }
  return node[key].as<T>();
}

Eigen::Vector3d vector3(const YAML::Node& node, const char* description) {
  if (!node || !node.IsSequence() || node.size() != 3) {
    throw std::invalid_argument(std::string(description) +
                                " must contain exactly three values");
  }
  return {node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
}

RigidTransform parseInlineTransform(const YAML::Node& node) {
  if (!node) {
    throw std::invalid_argument("each LiDAR requires vehicle_T_lidar");
  }
  RigidTransform transform;
  transform.translation = vector3(node["translation"], "translation");
  const Eigen::Vector3d rpy_deg = vector3(node["rpy_deg"], "rpy_deg");
  const Eigen::Vector3d rpy = rpy_deg * (M_PI / 180.0);
  transform.rotation = Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX());
  transform.rotation.normalize();
  return transform;
}

double protoValue(const std::string& block, const std::string& key) {
  const std::regex expression("\\b" + key +
                              "\\s*:\\s*([-+0-9.eE]+)");
  std::smatch match;
  if (!std::regex_search(block, match, expression))
    throw std::invalid_argument("missing prototxt field: " + key);
  return std::stod(match[1].str());
}

std::string protoBlock(const std::string& text, const std::string& name) {
  const std::regex expression(name + "\\s*\\{([^}]*)\\}");
  std::smatch match;
  if (!std::regex_search(text, match, expression))
    throw std::invalid_argument("missing prototxt block: " + name);
  return match[1].str();
}

RigidTransform parsePrototxtTransform(const std::string& path) {
  std::ifstream stream(path);
  if (!stream) throw std::invalid_argument("cannot open extrinsic prototxt: " + path);
  const std::string text((std::istreambuf_iterator<char>(stream)), {});
  const std::string translation = protoBlock(text, "translation");
  RigidTransform transform;
  transform.translation = {protoValue(translation, "x"),
                           protoValue(translation, "y"),
                           protoValue(translation, "z")};
  Eigen::Vector3d rpy;
  if (text.find("rotation_rpy_deg") != std::string::npos) {
    const std::string rotation = protoBlock(text, "rotation_rpy_deg");
    rpy = Eigen::Vector3d(protoValue(rotation, "x"),
                          protoValue(rotation, "y"),
                          protoValue(rotation, "z")) * (M_PI / 180.0);
  } else {
    const std::string rotation = protoBlock(text, "rotation");
    const bool degrees = rotation.find("roll_deg") != std::string::npos;
    const std::string suffix = degrees ? "_deg" : "";
    rpy = Eigen::Vector3d(protoValue(rotation, "roll" + suffix),
                          protoValue(rotation, "pitch" + suffix),
                          protoValue(rotation, "yaw" + suffix));
    if (degrees) rpy *= M_PI / 180.0;
  }
  transform.rotation = Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX());
  transform.rotation.normalize();
  return transform;
}

std::string resolvePath(const std::string& manifest_path,
                        const std::string& referenced_path) {
  if (!referenced_path.empty() && referenced_path.front() == '/')
    return referenced_path;
  const std::size_t slash = manifest_path.find_last_of('/');
  if (slash == std::string::npos) return referenced_path;
  return manifest_path.substr(0, slash + 1) + referenced_path;
}

const LidarConfig& findLidar(const OfflineManifest& manifest,
                             const std::string& name) {
  for (const auto& lidar : manifest.lidars) {
    if (lidar.name == name) return lidar;
  }
  throw std::invalid_argument("unknown LiDAR: " + name);
}

JointBackendMode parseBackendMode(const std::string& value) {
  if (value == "disabled") return JointBackendMode::kDisabled;
  if (value == "precise_refine") return JointBackendMode::kPreciseRefine;
  if (value == "coarse_bootstrap") return JointBackendMode::kCoarseBootstrap;
  if (value == "coarse_periodic") return JointBackendMode::kCoarsePeriodic;
  throw std::invalid_argument(
      "joint_backend.mode must be disabled, precise_refine, "
      "coarse_bootstrap, or coarse_periodic");
}

void parseJointBackend(const YAML::Node& node, JointBackendConfig* config) {
  if (!node) return;
  if (!node.IsMap())
    throw std::invalid_argument("joint_backend must be a map");
  if (node["enabled"]) config->enabled = node["enabled"].as<bool>();
  if (node["mode"])
    config->mode = parseBackendMode(node["mode"].as<std::string>());
#define READ_BACKEND_VALUE(name) \
  if (node[#name]) config->name = node[#name].as<decltype(config->name)>()
  READ_BACKEND_VALUE(initial_voxel_size);
  READ_BACKEND_VALUE(optimize_extrinsic_translation);
  READ_BACKEND_VALUE(minimum_voxel_size);
  READ_BACKEND_VALUE(downsample_size);
  READ_BACKEND_VALUE(planarity_ratio);
  READ_BACKEND_VALUE(minimum_points_per_voxel);
  READ_BACKEND_VALUE(maximum_points_per_observation);
  READ_BACKEND_VALUE(maximum_points_per_cloud);
  READ_BACKEND_VALUE(minimum_keyframes);
  READ_BACKEND_VALUE(maximum_keyframes);
  READ_BACKEND_VALUE(bootstrap_frames);
  READ_BACKEND_VALUE(minimum_mixed_voxels_per_lidar);
  READ_BACKEND_VALUE(heldout_fraction);
  READ_BACKEND_VALUE(minimum_heldout_improvement);
  READ_BACKEND_VALUE(pose_outer_iterations);
  READ_BACKEND_VALUE(extrinsic_outer_iterations);
  READ_BACKEND_VALUE(joint_outer_iterations);
  READ_BACKEND_VALUE(solver_iterations);
  READ_BACKEND_VALUE(maximum_condition_number);
  READ_BACKEND_VALUE(minimum_relative_eigenvalue);
  READ_BACKEND_VALUE(precise_max_rotation_update_deg);
  READ_BACKEND_VALUE(precise_max_translation_update_m);
  READ_BACKEND_VALUE(coarse_max_rotation_update_deg);
  READ_BACKEND_VALUE(coarse_max_translation_update_m);
  READ_BACKEND_VALUE(maximum_pose_rotation_update_deg);
  READ_BACKEND_VALUE(maximum_pose_translation_update_m);
  READ_BACKEND_VALUE(maximum_backend_passes);
  READ_BACKEND_VALUE(partition_seed);
#undef READ_BACKEND_VALUE
  if (!config->enabled) config->mode = JointBackendMode::kDisabled;
  if (config->enabled && config->mode == JointBackendMode::kDisabled)
    throw std::invalid_argument(
        "joint_backend.enabled requires a non-disabled mode");
}

}  // namespace

OfflineManifest loadManifest(const std::string& path) {
  const YAML::Node root = YAML::LoadFile(path);
  OfflineManifest manifest;
  manifest.dataset_root = required<std::string>(root, "dataset_root");
  manifest.output_root = required<std::string>(root, "output_root");
  if (root["mloam_config"])
    manifest.mloam_config = root["mloam_config"].as<std::string>();
  manifest.reference_lidar = required<std::string>(root, "reference_lidar");
  if (root["synchronization_threshold"])
    manifest.synchronization_threshold =
        root["synchronization_threshold"].as<double>();
  if (root["frame_begin"])
    manifest.frame_begin = root["frame_begin"].as<std::size_t>();
  if (root["frame_end"])
    manifest.frame_end = root["frame_end"].as<std::size_t>();
  if (root["frame_stride"])
    manifest.frame_stride = root["frame_stride"].as<std::size_t>();
  if (root["minimum_finite_ratio"])
    manifest.minimum_finite_ratio = root["minimum_finite_ratio"].as<double>();
  parseJointBackend(root["joint_backend"], &manifest.joint_backend);

  const YAML::Node lidars = root["lidars"];
  if (!lidars || !lidars.IsSequence()) {
    throw std::invalid_argument("lidars must be a sequence");
  }
  for (const auto& node : lidars) {
    LidarConfig lidar;
    lidar.name = required<std::string>(node, "name");
    lidar.directory = required<std::string>(node, "directory");
    lidar.ring_count = required<std::size_t>(node, "rings");
    lidar.horizontal_fov_deg =
        required<double>(node, "horizontal_fov_deg");
    lidar.expected_width = required<std::size_t>(node, "expected_width");
    if (node["enabled"]) lidar.enabled = node["enabled"].as<bool>();
    if (node["timestamp_scale"])
      lidar.timestamp_scale = node["timestamp_scale"].as<double>();
    if (node["scan_period"])
      lidar.scan_period = node["scan_period"].as<double>();
    if (node["time_offset"])
      lidar.time_offset = node["time_offset"].as<double>();
    if (node["min_cluster_size"])
      lidar.min_cluster_size = node["min_cluster_size"].as<int>();
    if (node["segment_valid_point_num"])
      lidar.segment_valid_point_num =
          node["segment_valid_point_num"].as<int>();
    if (node["segment_valid_line_num"])
      lidar.segment_valid_line_num = node["segment_valid_line_num"].as<int>();
    if (node["segment_cloud"])
      lidar.segment_cloud = node["segment_cloud"].as<bool>();
    if (node["segment_theta"])
      lidar.segment_theta_rad = node["segment_theta"].as<double>();
    const auto color = vector3(node["color"], "color");
    for (int i = 0; i < 3; ++i) {
      if (color[i] < 0.0 || color[i] > 255.0)
        throw std::invalid_argument("color values must be in [0, 255]");
    }
    lidar.color = {static_cast<std::uint8_t>(color.x()),
                   static_cast<std::uint8_t>(color.y()),
                   static_cast<std::uint8_t>(color.z())};
    if (node["vehicle_T_lidar"])
      lidar.vehicle_T_lidar = parseInlineTransform(node["vehicle_T_lidar"]);
    else if (node["extrinsic_prototxt"])
      lidar.vehicle_T_lidar = parsePrototxtTransform(resolvePath(
          path, node["extrinsic_prototxt"].as<std::string>()));
    else
      throw std::invalid_argument(
          "each LiDAR requires vehicle_T_lidar or extrinsic_prototxt");
    manifest.lidars.push_back(std::move(lidar));
  }
  validateManifest(manifest);
  return manifest;
}

void validateManifest(const OfflineManifest& manifest) {
  if (manifest.lidars.empty())
    throw std::invalid_argument("manifest has no LiDARs");
  if (!std::isfinite(manifest.synchronization_threshold) ||
      manifest.synchronization_threshold < 0.0)
    throw std::invalid_argument("synchronization threshold must be non-negative");
  if (manifest.frame_stride == 0)
    throw std::invalid_argument("frame stride must be positive");
  if (manifest.minimum_finite_ratio < 0.0 ||
      manifest.minimum_finite_ratio > 1.0)
    throw std::invalid_argument("minimum finite ratio must be in [0, 1]");
  const auto& backend = manifest.joint_backend;
  if (backend.enabled) {
    if (backend.mode == JointBackendMode::kDisabled)
      throw std::invalid_argument(
          "enabled joint backend requires a non-disabled mode");
    if (!std::isfinite(backend.initial_voxel_size) ||
        !std::isfinite(backend.minimum_voxel_size) ||
        !std::isfinite(backend.downsample_size) ||
        backend.initial_voxel_size <= 0.0 ||
        backend.minimum_voxel_size <= 0.0 ||
        backend.minimum_voxel_size > backend.initial_voxel_size ||
        backend.downsample_size <= 0.0)
      throw std::invalid_argument("invalid joint backend voxel sizes");
    if (!std::isfinite(backend.planarity_ratio) ||
        backend.planarity_ratio <= 1.0 ||
        backend.minimum_points_per_voxel < 3 ||
        backend.maximum_points_per_observation == 0 ||
        backend.maximum_points_per_cloud == 0 ||
        backend.minimum_keyframes < 2 ||
        backend.maximum_keyframes < backend.minimum_keyframes ||
        backend.bootstrap_frames < backend.minimum_keyframes ||
        backend.minimum_mixed_voxels_per_lidar == 0)
      throw std::invalid_argument("invalid joint backend sampling limits");
    if (!std::isfinite(backend.heldout_fraction) ||
        backend.heldout_fraction <= 0.0 || backend.heldout_fraction >= 0.5 ||
        !std::isfinite(backend.minimum_heldout_improvement) ||
        backend.minimum_heldout_improvement < 0.0 ||
        backend.minimum_heldout_improvement >= 1.0)
      throw std::invalid_argument("invalid joint backend validation split");
    if (backend.pose_outer_iterations == 0 ||
        backend.extrinsic_outer_iterations == 0 ||
        backend.joint_outer_iterations == 0 ||
        backend.solver_iterations == 0 ||
        backend.maximum_backend_passes == 0 ||
        !std::isfinite(backend.maximum_condition_number) ||
        backend.maximum_condition_number <= 1.0 ||
        !std::isfinite(backend.minimum_relative_eigenvalue) ||
        backend.minimum_relative_eigenvalue <= 0.0)
      throw std::invalid_argument("invalid joint backend solver settings");
    if (!std::isfinite(backend.precise_max_rotation_update_deg) ||
        !std::isfinite(backend.precise_max_translation_update_m) ||
        !std::isfinite(backend.coarse_max_rotation_update_deg) ||
        !std::isfinite(backend.coarse_max_translation_update_m) ||
        !std::isfinite(backend.maximum_pose_rotation_update_deg) ||
        !std::isfinite(backend.maximum_pose_translation_update_m) ||
        backend.precise_max_rotation_update_deg <= 0.0 ||
        backend.precise_max_translation_update_m <= 0.0 ||
        backend.coarse_max_rotation_update_deg <= 0.0 ||
        backend.coarse_max_translation_update_m <= 0.0 ||
        backend.maximum_pose_rotation_update_deg <= 0.0 ||
        backend.maximum_pose_translation_update_m <= 0.0)
      throw std::invalid_argument(
          "invalid joint backend update acceptance bounds");
  }

  std::set<std::string> names;
  bool enabled_reference = false;
  std::size_t enabled_lidars = 0;
  for (const auto& lidar : manifest.lidars) {
    if (lidar.name.empty() || !names.insert(lidar.name).second)
      throw std::invalid_argument("LiDAR names must be non-empty and unique");
    if (lidar.ring_count == 0 || lidar.expected_width == 0)
      throw std::invalid_argument("LiDAR rings and expected width must be positive");
    if (!std::isfinite(lidar.timestamp_scale) || lidar.timestamp_scale <= 0.0)
      throw std::invalid_argument("timestamp scale must be positive");
    if (!std::isfinite(lidar.scan_period) || lidar.scan_period <= 0.0 ||
        lidar.scan_period >= 1.0 || !std::isfinite(lidar.time_offset) ||
        !std::isfinite(lidar.horizontal_fov_deg) ||
        lidar.horizontal_fov_deg <= 0.0 || lidar.horizontal_fov_deg > 360.0)
      throw std::invalid_argument("invalid LiDAR timing or horizontal FOV");
    if (lidar.min_cluster_size <= 0 || lidar.segment_valid_point_num <= 0 ||
        lidar.segment_valid_line_num <= 0 ||
        (lidar.segment_cloud &&
         lidar.segment_valid_line_num > static_cast<int>(lidar.ring_count)) ||
        !std::isfinite(lidar.segment_theta_rad) ||
        lidar.segment_theta_rad <= 0.0 || lidar.segment_theta_rad >= M_PI_2)
      throw std::invalid_argument("invalid LiDAR segmentation parameters");
    if (lidar.name == manifest.reference_lidar && lidar.enabled)
      enabled_reference = true;
    if (lidar.enabled) ++enabled_lidars;
  }
  if (!enabled_reference)
    throw std::invalid_argument("reference LiDAR must exist and be enabled");
  if (backend.enabled && enabled_lidars < 2)
    throw std::invalid_argument(
        "joint backend requires at least two enabled LiDARs");
}

void applyLidarSelection(OfflineManifest& manifest,
                         const std::vector<std::string>& include,
                         const std::vector<std::string>& exclude) {
  std::set<std::string> included(include.begin(), include.end());
  std::set<std::string> excluded(exclude.begin(), exclude.end());
  for (const auto& name : included) findLidar(manifest, name);
  for (const auto& name : excluded) findLidar(manifest, name);
  for (auto& lidar : manifest.lidars) {
    if (!included.empty()) lidar.enabled = included.count(lidar.name) != 0;
    if (excluded.count(lidar.name) != 0) lidar.enabled = false;
  }
  validateManifest(manifest);
}

std::map<std::string, RigidTransform> referenceRelativeExtrinsics(
    const OfflineManifest& manifest) {
  const auto& reference = findLidar(manifest, manifest.reference_lidar);
  const RigidTransform reference_T_vehicle =
      reference.vehicle_T_lidar.inverse();
  std::map<std::string, RigidTransform> result;
  for (const auto& lidar : manifest.lidars) {
    if (!lidar.enabled) continue;
    result.emplace(lidar.name,
                   reference_T_vehicle * lidar.vehicle_T_lidar);
  }
  return result;
}

}  // namespace offline
}  // namespace mloam
