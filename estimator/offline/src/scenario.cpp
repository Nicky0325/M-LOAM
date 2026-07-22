#include "mloam/offline/scenario.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iomanip>
#include <random>
#include <sstream>
#include <stdexcept>

namespace mloam {
namespace offline {
namespace {

Eigen::Vector3d randomUnitVector(std::mt19937_64& generator) {
  std::normal_distribution<double> normal(0.0, 1.0);
  Eigen::Vector3d direction;
  do {
    direction = {normal(generator), normal(generator), normal(generator)};
  } while (direction.squaredNorm() < 1e-18);
  return direction.normalized();
}

std::uint64_t nameSeed(std::uint64_t seed, const std::string& name) {
  std::uint64_t value = 1469598103934665603ULL ^ seed;
  for (unsigned char character : name) {
    value ^= character;
    value *= 1099511628211ULL;
  }
  return value;
}

std::string magnitudeName(double value) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << value;
  std::string result = stream.str();
  while (result.size() > 1 && result.back() == '0') result.pop_back();
  if (!result.empty() && result.back() == '.') result.pop_back();
  std::replace(result.begin(), result.end(), '.', 'p');
  return result;
}

void perturbNonReferenceExtrinsics(const OfflineManifest& manifest,
                                   ScenarioConfiguration& scenario) {
  for (auto& item : scenario.initial_extrinsics) {
    if (item.first == manifest.reference_lidar) continue;
    std::mt19937_64 generator(nameSeed(scenario.seed, item.first));
    const Eigen::Vector3d rotation_axis = randomUnitVector(generator);
    const Eigen::Vector3d translation_direction = randomUnitVector(generator);
    RigidTransform perturbation;
    perturbation.rotation = Eigen::AngleAxisd(
        scenario.rotation_perturbation_deg * M_PI / 180.0, rotation_axis);
    perturbation.translation =
        translation_direction * scenario.translation_perturbation_m;
    item.second.rotation = item.second.rotation * perturbation.rotation;
    item.second.rotation.normalize();
    item.second.translation += perturbation.translation;
    scenario.injected_perturbations[item.first] = perturbation;
  }
}

}  // namespace

ScenarioConfiguration makeScenario(const OfflineManifest& manifest,
                                   CalibrationScenario scenario,
                                   std::uint64_t seed,
                                   std::size_t coarse_level) {
  static const std::array<double, 3> kRotationDegrees{{5.0, 15.0, 30.0}};
  static const std::array<double, 3> kTranslationMeters{{0.25, 0.75, 1.5}};
  ScenarioConfiguration result;
  result.seed = seed;
  result.scenario = scenario;
  result.precise_extrinsics = referenceRelativeExtrinsics(manifest);
  result.initial_extrinsics = result.precise_extrinsics;
  if (scenario == CalibrationScenario::kPrecise) {
    result.name = "precise";
    result.estimator_mode = 0;
    return result;
  }
  if (scenario == CalibrationScenario::kCalibratedInit) {
    result.name = "calibrated_init";
    result.estimator_mode = 1;
    return result;
  }
  if (scenario == CalibrationScenario::kPriorFree) {
    result.name = "prior_free";
    result.estimator_mode = 2;
    for (auto& item : result.initial_extrinsics) {
      if (item.first != manifest.reference_lidar) item.second = RigidTransform();
    }
    return result;
  }
  if (coarse_level >= kRotationDegrees.size()) {
    throw std::invalid_argument("coarse level must be 0, 1, or 2");
  }
  result.estimator_mode = 1;
  result.rotation_perturbation_deg = kRotationDegrees[coarse_level];
  result.translation_perturbation_m = kTranslationMeters[coarse_level];
  result.name = "coarse_" +
                std::to_string(static_cast<int>(result.rotation_perturbation_deg)) +
                "deg_" +
                (coarse_level == 0 ? "0.25" : coarse_level == 1 ? "0.75" : "1.5") +
                "m";
  perturbNonReferenceExtrinsics(manifest, result);
  return result;
}

ScenarioConfiguration makePerturbedCalibrationScenario(
    const OfflineManifest& manifest, std::uint64_t seed,
    double rotation_perturbation_deg, double translation_perturbation_m) {
  if (!std::isfinite(rotation_perturbation_deg) ||
      rotation_perturbation_deg < 0.0 || rotation_perturbation_deg > 180.0) {
    throw std::invalid_argument(
        "rotation perturbation must be finite and in [0, 180] degrees");
  }
  if (!std::isfinite(translation_perturbation_m) ||
      translation_perturbation_m < 0.0) {
    throw std::invalid_argument(
        "translation perturbation must be finite and non-negative");
  }
  ScenarioConfiguration result;
  result.seed = seed;
  result.scenario = CalibrationScenario::kCoarse;
  result.estimator_mode = 1;
  result.rotation_perturbation_deg = rotation_perturbation_deg;
  result.translation_perturbation_m = translation_perturbation_m;
  result.name = "calibrated_init_" + magnitudeName(rotation_perturbation_deg) +
                "deg_" + magnitudeName(translation_perturbation_m) + "m";
  result.precise_extrinsics = referenceRelativeExtrinsics(manifest);
  result.initial_extrinsics = result.precise_extrinsics;
  perturbNonReferenceExtrinsics(manifest, result);
  return result;
}

void CalibrationStateTracker::update(
    const std::string& lidar_name, std::size_t frame, double timestamp,
    bool observable, double rotation_error_deg, double translation_error_m,
    bool failed) {
  CalibrationHistoryEntry entry;
  entry.frame = frame;
  entry.timestamp = timestamp;
  entry.observable = observable;
  entry.rotation_error_deg = rotation_error_deg;
  entry.translation_error_m = translation_error_m;
  if (failed) {
    entry.state = CalibrationState::kFailed;
  } else if (!observable) {
    entry.state = CalibrationState::kInitializing;
  } else if (rotation_error_deg <= rotation_threshold_deg_ &&
             translation_error_m <= translation_threshold_m_) {
    entry.state = CalibrationState::kConverged;
  } else {
    entry.state = CalibrationState::kObservable;
  }
  history_[lidar_name].push_back(entry);
}

void CalibrationStateTracker::finalize(std::size_t frame, double timestamp) {
  for (auto& item : history_) {
    if (item.second.empty()) continue;
    const CalibrationState state = item.second.back().state;
    if (state == CalibrationState::kConverged ||
        state == CalibrationState::kFailed) {
      continue;
    }
    CalibrationHistoryEntry entry = item.second.back();
    entry.frame = frame;
    entry.timestamp = timestamp;
    entry.state = CalibrationState::kNonConverged;
    item.second.push_back(entry);
  }
}

const std::vector<CalibrationHistoryEntry>& CalibrationStateTracker::history(
    const std::string& lidar_name) const {
  const auto it = history_.find(lidar_name);
  if (it == history_.end()) throw std::out_of_range("unknown calibration history");
  return it->second;
}

const char* calibrationStateName(CalibrationState state) {
  switch (state) {
    case CalibrationState::kInitializing: return "initializing";
    case CalibrationState::kObservable: return "observable";
    case CalibrationState::kConverged: return "converged";
    case CalibrationState::kNonConverged: return "non_converged";
    case CalibrationState::kFailed: return "failed";
  }
  return "failed";
}

}  // namespace offline
}  // namespace mloam
