#include "mloam/offline/pcd_preprocessor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <queue>
#include <sstream>
#include <vector>

#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLPointField.h>
#include <pcl/io/pcd_io.h>

namespace mloam {
namespace offline {
namespace {

const pcl::PCLPointField& requiredField(const pcl::PCLPointCloud2& cloud,
                                        const std::string& name) {
  const auto it = std::find_if(
      cloud.fields.begin(), cloud.fields.end(),
      [&](const pcl::PCLPointField& field) { return field.name == name; });
  if (it == cloud.fields.end() || it->count != 1) {
    throw PcdValidationError("required scalar PCD field is missing: " + name);
  }
  return *it;
}

double numericValue(const std::uint8_t* point,
                    const pcl::PCLPointField& field) {
  const std::uint8_t* data = point + field.offset;
  switch (field.datatype) {
    case pcl::PCLPointField::INT8: {
      std::int8_t value;
      std::memcpy(&value, data, sizeof(value));
      return value;
    }
    case pcl::PCLPointField::UINT8: {
      std::uint8_t value;
      std::memcpy(&value, data, sizeof(value));
      return value;
    }
    case pcl::PCLPointField::INT16: {
      std::int16_t value;
      std::memcpy(&value, data, sizeof(value));
      return value;
    }
    case pcl::PCLPointField::UINT16: {
      std::uint16_t value;
      std::memcpy(&value, data, sizeof(value));
      return value;
    }
    case pcl::PCLPointField::INT32: {
      std::int32_t value;
      std::memcpy(&value, data, sizeof(value));
      return value;
    }
    case pcl::PCLPointField::UINT32: {
      std::uint32_t value;
      std::memcpy(&value, data, sizeof(value));
      return value;
    }
    case pcl::PCLPointField::FLOAT32: {
      float value;
      std::memcpy(&value, data, sizeof(value));
      return value;
    }
    case pcl::PCLPointField::FLOAT64: {
      double value;
      std::memcpy(&value, data, sizeof(value));
      return value;
    }
    default:
      throw PcdValidationError("unsupported PCD field type for " + field.name);
  }
}

std::uint16_t ringValue(const std::uint8_t* point,
                        const pcl::PCLPointField& field,
                        std::size_t ring_count) {
  if (field.datatype != pcl::PCLPointField::UINT8 &&
      field.datatype != pcl::PCLPointField::UINT16 &&
      field.datatype != pcl::PCLPointField::UINT32) {
    throw PcdValidationError("ring must use an unsigned integer PCD type");
  }
  const double value = numericValue(point, field);
  if (value < 0.0 || value >= static_cast<double>(ring_count) ||
      value > std::numeric_limits<std::uint16_t>::max()) {
    throw PcdValidationError("ring value is outside configured bounds");
  }
  return static_cast<std::uint16_t>(value);
}

double pointRange(const NativePointXYZIRT& point) {
  return std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
}

bool connected(const NativePointXYZIRT& first,
               const NativePointXYZIRT& second,
               double fallback_alpha, double threshold) {
  const double first_range = pointRange(first);
  const double second_range = pointRange(second);
  if (first_range <= 0.0 || second_range <= 0.0) return false;
  double cosine = (first.x * second.x + first.y * second.y +
                   first.z * second.z) / (first_range * second_range);
  cosine = std::max(-1.0, std::min(1.0, cosine));
  double alpha = std::acos(cosine);
  if (alpha < 1e-6) alpha = fallback_alpha;
  const double d1 = std::max(first_range, second_range);
  const double d2 = std::min(first_range, second_range);
  const double angle = std::atan2(d2 * std::sin(alpha),
                                  d1 - d2 * std::cos(alpha));
  return angle > threshold;
}

std::vector<bool> segmentationMask(const std::vector<NativePointXYZIRT>& points,
                                   const LidarConfig& config) {
  const std::size_t missing = std::numeric_limits<std::size_t>::max();
  std::vector<std::size_t> grid(config.ring_count * config.expected_width,
                                missing);
  for (std::size_t i = 0; i < points.size(); ++i) {
    const std::size_t cell = points[i].ring * config.expected_width +
                             points[i].organized_column;
    if (grid[cell] != missing)
      throw PcdValidationError("duplicate ring/column cell in organized PCD");
    grid[cell] = i;
  }
  std::vector<bool> keep(points.size(), false);
  std::vector<bool> visited(points.size(), false);
  const double horizontal_alpha =
      config.horizontal_fov_deg * M_PI / 180.0 / config.expected_width;
  const bool wrap_columns = config.horizontal_fov_deg >= 359.0;
  for (std::size_t seed = 0; seed < points.size(); ++seed) {
    if (visited[seed]) continue;
    std::queue<std::size_t> pending;
    std::vector<std::size_t> cluster;
    std::vector<bool> rings(config.ring_count, false);
    pending.push(seed);
    visited[seed] = true;
    while (!pending.empty()) {
      const std::size_t index = pending.front();
      pending.pop();
      cluster.push_back(index);
      rings[points[index].ring] = true;
      const int row = points[index].ring;
      const int column = points[index].organized_column;
      const int dr[4] = {-1, 1, 0, 0};
      const int dc[4] = {0, 0, -1, 1};
      for (int direction = 0; direction < 4; ++direction) {
        const int next_row = row + dr[direction];
        int next_column = column + dc[direction];
        if (next_row < 0 || next_row >= static_cast<int>(config.ring_count))
          continue;
        if (next_column < 0 ||
            next_column >= static_cast<int>(config.expected_width)) {
          if (!wrap_columns) continue;
          const int width = static_cast<int>(config.expected_width);
          next_column = (next_column + width) % width;
        }
        const std::size_t neighbor =
            grid[next_row * config.expected_width + next_column];
        if (neighbor == missing || visited[neighbor]) continue;
        if (!connected(points[index], points[neighbor], horizontal_alpha,
                       config.segment_theta_rad))
          continue;
        visited[neighbor] = true;
        pending.push(neighbor);
      }
    }
    const std::size_t line_count =
        static_cast<std::size_t>(std::count(rings.begin(), rings.end(), true));
    const bool valid =
        cluster.size() >= static_cast<std::size_t>(config.min_cluster_size) ||
        (cluster.size() >=
             static_cast<std::size_t>(config.segment_valid_point_num) &&
         line_count >= static_cast<std::size_t>(config.segment_valid_line_num));
    if (valid)
      for (const auto index : cluster) keep[index] = true;
  }
  return keep;
}

}  // namespace

PreparedLidarFrame preparePcd(const std::string& path,
                              const LidarConfig& config,
                              double minimum_finite_ratio) {
  if (config.ring_count == 0 || config.expected_width == 0) {
    throw PcdValidationError("LiDAR ring count and expected width must be positive");
  }
  if (minimum_finite_ratio < 0.0 || minimum_finite_ratio > 1.0) {
    throw PcdValidationError("minimum finite ratio must be in [0, 1]");
  }

  pcl::PCLPointCloud2 cloud;
  if (pcl::io::loadPCDFile(path, cloud) < 0) {
    throw PcdValidationError("cannot load PCD: " + path);
  }
  if (cloud.width != config.expected_width ||
      cloud.height != config.ring_count ||
      cloud.point_step == 0 || cloud.row_step < cloud.point_step * cloud.width) {
    throw PcdValidationError("PCD dimensions do not match configured geometry");
  }

  const auto& x_field = requiredField(cloud, "x");
  const auto& y_field = requiredField(cloud, "y");
  const auto& z_field = requiredField(cloud, "z");
  const auto& reflectivity_field = requiredField(cloud, "reflectivity");
  const auto& ring_field = requiredField(cloud, "ring");
  const auto& timestamp_field = requiredField(cloud, "timestamp");

  PreparedLidarFrame result;
  result.lidar_name = config.name;
  result.source_width = cloud.width;
  result.source_height = cloud.height;
  const std::size_t total = static_cast<std::size_t>(cloud.width) * cloud.height;
  result.native_points.reserve(total);
  for (std::size_t row = 0; row < cloud.height; ++row) {
    for (std::size_t column = 0; column < cloud.width; ++column) {
      const std::size_t offset = row * cloud.row_step + column * cloud.point_step;
      if (offset + cloud.point_step > cloud.data.size()) {
        throw PcdValidationError("PCD data is shorter than its dimensions");
      }
      const auto* point = cloud.data.data() + offset;
      NativePointXYZIRT native;
      native.x = static_cast<float>(numericValue(point, x_field));
      native.y = static_cast<float>(numericValue(point, y_field));
      native.z = static_cast<float>(numericValue(point, z_field));
      native.reflectivity =
          static_cast<float>(numericValue(point, reflectivity_field));
      native.timestamp = numericValue(point, timestamp_field) *
                         config.timestamp_scale;
      native.organized_column = column;
      if (!std::isfinite(native.x) || !std::isfinite(native.y) ||
          !std::isfinite(native.z) || !std::isfinite(native.reflectivity) ||
          !std::isfinite(native.timestamp)) {
        ++result.discarded_non_finite;
        continue;
      }
      if (native.timestamp < 0.0 || native.timestamp > config.scan_period) {
        throw PcdValidationError(
            "scaled timestamp is outside the configured scan period");
      }
      native.ring = ringValue(point, ring_field, config.ring_count);
      result.native_points.push_back(native);
    }
  }

  const double finite_ratio = total == 0
                                  ? 0.0
                                  : static_cast<double>(result.native_points.size()) /
                                        static_cast<double>(total);
  if (finite_ratio < minimum_finite_ratio) {
    std::ostringstream message;
    message << "finite-point ratio " << finite_ratio << " is below "
            << minimum_finite_ratio;
    throw PcdValidationError(message.str());
  }

  result.ring_start_indices.assign(config.ring_count,
                                   std::numeric_limits<std::size_t>::max());
  result.ring_end_indices.assign(config.ring_count,
                                 std::numeric_limits<std::size_t>::max());
  result.ring_ordered_points.reserve(result.native_points.size());
  const std::vector<bool> keep = config.segment_cloud
                                     ? segmentationMask(result.native_points,
                                                        config)
                                     : std::vector<bool>(result.native_points.size(),
                                                         true);
  for (std::size_t ring = 0; ring < config.ring_count; ++ring) {
    for (std::size_t native_index = 0;
         native_index < result.native_points.size(); ++native_index) {
      const auto& native = result.native_points[native_index];
      if (native.ring != ring) continue;
      if (!keep[native_index]) {
        result.outlier_ring_ordered_points.push_back(
            {native.x, native.y, native.z,
             static_cast<float>(native.ring + native.timestamp)});
        continue;
      }
      const std::size_t encoded_index = result.ring_ordered_points.size();
      if (result.ring_start_indices[ring] ==
          std::numeric_limits<std::size_t>::max()) {
        result.ring_start_indices[ring] = encoded_index;
      }
      result.ring_end_indices[ring] = encoded_index;
      result.ring_ordered_points.push_back(
          {native.x, native.y, native.z,
           static_cast<float>(native.ring + native.timestamp)});
    }
  }
  return result;
}

}  // namespace offline
}  // namespace mloam
