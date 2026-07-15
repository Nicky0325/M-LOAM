#include "mloam/offline/sequence_reader.hpp"

#include <algorithm>
#include <cmath>
#include <dirent.h>
#include <limits>
#include <set>
#include <stdexcept>

namespace mloam {
namespace offline {

SequenceIndex SequenceIndex::fromDirectory(const std::string& name,
                                           const std::string& directory,
                                           double time_offset) {
  DIR* handle = opendir(directory.c_str());
  if (handle == nullptr) {
    throw std::invalid_argument("cannot open LiDAR directory: " + directory);
  }
  std::vector<std::pair<double, std::string>> scans;
  while (dirent* entry = readdir(handle)) {
    const std::string filename(entry->d_name);
    if (filename.size() <= 4 ||
        filename.compare(filename.size() - 4, 4, ".pcd") != 0) {
      continue;
    }
    const std::string stem = filename.substr(0, filename.size() - 4);
    try {
      std::size_t consumed = 0;
      const double timestamp = std::stod(stem, &consumed);
      if (consumed != stem.size() || !std::isfinite(timestamp)) continue;
      const std::string separator =
          !directory.empty() && directory.back() == '/' ? "" : "/";
      scans.emplace_back(timestamp, directory + separator + filename);
    } catch (const std::exception&) {
      continue;
    }
  }
  closedir(handle);
  return SequenceIndex(name, time_offset, std::move(scans));
}

SequenceIndex::SequenceIndex(
    std::string name, double time_offset,
    std::vector<std::pair<double, std::string>> scans)
    : name_(std::move(name)), time_offset_(time_offset) {
  scans_.reserve(scans.size());
  for (const auto& scan : scans) {
    scans_.push_back({scan.first, scan.second});
  }
  std::sort(scans_.begin(), scans_.end(),
            [](const IndexedScan& lhs, const IndexedScan& rhs) {
              return lhs.timestamp < rhs.timestamp;
            });
}

FrameSynchronizer::FrameSynchronizer(std::string reference_name,
                                     double threshold_seconds,
                                     std::vector<SequenceIndex> indexes)
    : reference_name_(std::move(reference_name)),
      threshold_seconds_(threshold_seconds),
      indexes_(std::move(indexes)) {
  if (threshold_seconds_ < 0.0) {
    throw std::invalid_argument("synchronization threshold must be non-negative");
  }
  const auto reference = std::find_if(
      indexes_.begin(), indexes_.end(), [&](const SequenceIndex& index) {
        return index.name() == reference_name_;
      });
  if (reference == indexes_.end()) {
    throw std::invalid_argument("reference LiDAR is not indexed: " +
                                reference_name_);
  }
}

std::vector<SynchronizedFrame> FrameSynchronizer::match(
    std::size_t begin, std::size_t end_exclusive, std::size_t stride) const {
  if (stride == 0) {
    throw std::invalid_argument("frame stride must be positive");
  }
  const auto reference_it = std::find_if(
      indexes_.begin(), indexes_.end(), [&](const SequenceIndex& index) {
        return index.name() == reference_name_;
      });
  const auto& reference_scans = reference_it->scans();
  end_exclusive = std::min(end_exclusive, reference_scans.size());

  std::vector<std::set<std::size_t>> used(indexes_.size());
  std::vector<SynchronizedFrame> result;
  for (std::size_t ref_index = begin; ref_index < end_exclusive;
       ref_index += stride) {
    const auto& reference_scan = reference_scans[ref_index];
    const double reference_time =
        reference_scan.timestamp + reference_it->timeOffset();
    SynchronizedFrame frame;
    frame.reference_index = ref_index;
    frame.reference_timestamp = reference_time;

    for (std::size_t lidar_index = 0; lidar_index < indexes_.size();
         ++lidar_index) {
      const auto& index = indexes_[lidar_index];
      std::size_t best = std::numeric_limits<std::size_t>::max();
      double best_skew = std::numeric_limits<double>::infinity();
      for (std::size_t scan_index = 0; scan_index < index.scans().size();
           ++scan_index) {
        if (used[lidar_index].count(scan_index) != 0) continue;
        const double corrected = index.scans()[scan_index].timestamp +
                                 index.timeOffset();
        const double absolute_skew = std::abs(corrected - reference_time);
        if (absolute_skew < best_skew) {
          best = scan_index;
          best_skew = absolute_skew;
        }
      }

      if (best == std::numeric_limits<std::size_t>::max() ||
          best_skew > threshold_seconds_) {
        frame.missing_lidars.push_back(index.name());
        continue;
      }
      used[lidar_index].insert(best);
      const auto& scan = index.scans()[best];
      ScanMatch match;
      match.lidar_name = index.name();
      match.path = scan.path;
      match.raw_timestamp = scan.timestamp;
      match.corrected_timestamp = scan.timestamp + index.timeOffset();
      match.raw_skew = scan.timestamp - reference_scan.timestamp;
      match.corrected_skew = match.corrected_timestamp - reference_time;
      frame.scans.emplace(index.name(), std::move(match));
    }
    frame.complete = frame.missing_lidars.empty();
    result.push_back(std::move(frame));
  }
  return result;
}

}  // namespace offline
}  // namespace mloam
