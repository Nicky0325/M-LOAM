#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "mloam/offline/manifest.hpp"

namespace mloam {
namespace offline {

struct ScenarioConfiguration {
  std::string name;
  std::uint64_t seed = 42;
  CalibrationScenario scenario = CalibrationScenario::kPrecise;
  int estimator_mode = 0;
  double rotation_perturbation_deg = 0.0;
  double translation_perturbation_m = 0.0;
  std::map<std::string, RigidTransform> precise_extrinsics;
  std::map<std::string, RigidTransform> initial_extrinsics;
  std::map<std::string, RigidTransform> injected_perturbations;
};

ScenarioConfiguration makeScenario(const OfflineManifest& manifest,
                                   CalibrationScenario scenario,
                                   std::uint64_t seed,
                                   std::size_t coarse_level);

ScenarioConfiguration makePerturbedCalibrationScenario(
    const OfflineManifest& manifest, std::uint64_t seed,
    double rotation_perturbation_deg,
    double translation_perturbation_m = 0.0);

struct CalibrationHistoryEntry {
  std::size_t frame = 0;
  double timestamp = 0.0;
  CalibrationState state = CalibrationState::kInitializing;
  bool observable = false;
  double rotation_error_deg = 0.0;
  double translation_error_m = 0.0;
};

class CalibrationStateTracker {
 public:
  CalibrationStateTracker(double rotation_threshold_deg,
                          double translation_threshold_m)
      : rotation_threshold_deg_(rotation_threshold_deg),
        translation_threshold_m_(translation_threshold_m) {}

  void update(const std::string& lidar_name, std::size_t frame,
              double timestamp, bool observable, double rotation_error_deg,
              double translation_error_m, bool failed);
  void finalize(std::size_t frame, double timestamp);
  const std::vector<CalibrationHistoryEntry>& history(
      const std::string& lidar_name) const;

 private:
  double rotation_threshold_deg_;
  double translation_threshold_m_;
  std::map<std::string, std::vector<CalibrationHistoryEntry>> history_;
};

const char* calibrationStateName(CalibrationState state);

}  // namespace offline
}  // namespace mloam
