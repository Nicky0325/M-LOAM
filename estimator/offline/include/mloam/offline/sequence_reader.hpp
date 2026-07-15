#pragma once

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "mloam/offline/types.hpp"

namespace mloam {
namespace offline {

struct IndexedScan {
  double timestamp = 0.0;
  std::string path;
};

class SequenceIndex {
 public:
  static SequenceIndex fromDirectory(const std::string& name,
                                     const std::string& directory,
                                     double time_offset);

  SequenceIndex(std::string name, double time_offset,
                std::vector<std::pair<double, std::string>> scans);

  const std::string& name() const { return name_; }
  double timeOffset() const { return time_offset_; }
  const std::vector<IndexedScan>& scans() const { return scans_; }

 private:
  std::string name_;
  double time_offset_;
  std::vector<IndexedScan> scans_;
};

class FrameSynchronizer {
 public:
  FrameSynchronizer(std::string reference_name, double threshold_seconds,
                    std::vector<SequenceIndex> indexes);

  std::vector<SynchronizedFrame> match(std::size_t begin,
                                       std::size_t end_exclusive,
                                       std::size_t stride) const;

 private:
  std::string reference_name_;
  double threshold_seconds_;
  std::vector<SequenceIndex> indexes_;
};

}  // namespace offline
}  // namespace mloam
