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

  std::set<std::string> names;
  bool enabled_reference = false;
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
  }
  if (!enabled_reference)
    throw std::invalid_argument("reference LiDAR must exist and be enabled");
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
