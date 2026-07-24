#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <chrono>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <ros/ros.h>

#include "mloam/offline/runner.hpp"
#include "../../src/estimator/estimator.h"
#include "../../src/estimator/parameters.h"
#include "../../src/lidarMapper/lidar_mapper_core.h"

namespace offline = mloam::offline;

namespace {

struct Options {
  std::string manifest;
  std::string mloam_config;
  std::string scenario = "precise";
  std::string coarse_level = "all";
  std::vector<double> rotation_perturbations;
  double translation_perturbation = 0.0;
  std::vector<std::string> include;
  std::vector<std::string> exclude;
  std::string reference;
  std::string backend_mode;
  std::string output_root;
  std::size_t begin = std::numeric_limits<std::size_t>::max();
  std::size_t end = std::numeric_limits<std::size_t>::max();
  std::size_t stride = 0;
  std::uint64_t seed = 42;
  bool publish_ros = false;
};

std::vector<std::string> split(const std::string& value) {
  std::vector<std::string> result;
  std::size_t begin = 0;
  while (begin <= value.size()) {
    const std::size_t comma = value.find(',', begin);
    const std::string item = value.substr(
        begin, comma == std::string::npos ? std::string::npos : comma - begin);
    if (!item.empty()) result.push_back(item);
    if (comma == std::string::npos) break;
    begin = comma + 1;
  }
  return result;
}

void usage(std::ostream& stream) {
  stream << "Usage: mloam_offline_runner --manifest FILE [options]\n"
         << "  --mloam-config FILE       legacy estimator algorithm config\n"
         << "  --scenario NAME           precise|calibrated_init|calibration_sweep|coarse|prior_free|all\n"
         << "  --coarse-level LEVEL      0|1|2|all (default all)\n"
         << "  --rotation-perturbations D1,D2,...  calibration_sweep angles (degrees)\n"
         << "  --translation-perturbation M       optional sweep translation magnitude (default 0)\n"
         << "  --include a,b --exclude c select LiDAR names\n"
         << "  --reference NAME          override reference LiDAR\n"
         << "  --backend-mode MODE       disabled|precise_refine|coarse_bootstrap|coarse_periodic\n"
         << "  --begin N --end N --stride N\n"
         << "  --seed N --output-root DIR --ros-publish\n";
}

Options parseOptions(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--help" || argument == "-h") {
      usage(std::cout);
      std::exit(0);
    }
    if (argument == "--ros-publish") {
      options.publish_ros = true;
      continue;
    }
    if (i + 1 >= argc) throw std::invalid_argument("missing value for " + argument);
    const std::string value = argv[++i];
    if (argument == "--manifest") options.manifest = value;
    else if (argument == "--mloam-config") options.mloam_config = value;
    else if (argument == "--scenario") options.scenario = value;
    else if (argument == "--coarse-level") options.coarse_level = value;
    else if (argument == "--rotation-perturbations") {
      for (const auto& item : split(value))
        options.rotation_perturbations.push_back(std::stod(item));
    }
    else if (argument == "--translation-perturbation")
      options.translation_perturbation = std::stod(value);
    else if (argument == "--include") options.include = split(value);
    else if (argument == "--exclude") options.exclude = split(value);
    else if (argument == "--reference") options.reference = value;
    else if (argument == "--backend-mode") options.backend_mode = value;
    else if (argument == "--output-root") options.output_root = value;
    else if (argument == "--begin") options.begin = std::stoull(value);
    else if (argument == "--end") options.end = std::stoull(value);
    else if (argument == "--stride") options.stride = std::stoull(value);
    else if (argument == "--seed") options.seed = std::stoull(value);
    else throw std::invalid_argument("unknown option: " + argument);
  }
  if (options.manifest.empty()) throw std::invalid_argument("--manifest is required");
  return options;
}

std::vector<std::size_t> coarseLevels(const std::string& value) {
  if (value == "all") return {0, 1, 2};
  const std::size_t level = std::stoull(value);
  if (level > 2) throw std::invalid_argument("coarse level must be 0, 1, 2, or all");
  return {level};
}

offline::JointBackendMode backendMode(const std::string& value) {
  if (value == "disabled") return offline::JointBackendMode::kDisabled;
  if (value == "precise_refine")
    return offline::JointBackendMode::kPreciseRefine;
  if (value == "coarse_bootstrap")
    return offline::JointBackendMode::kCoarseBootstrap;
  if (value == "coarse_periodic")
    return offline::JointBackendMode::kCoarsePeriodic;
  throw std::invalid_argument("unknown backend mode: " + value);
}

std::vector<offline::ScenarioConfiguration> scenarios(
    const offline::OfflineManifest& manifest, const Options& options) {
  std::vector<offline::ScenarioConfiguration> result;
  if (options.scenario == "precise" || options.scenario == "all")
    result.push_back(offline::makeScenario(
        manifest, offline::CalibrationScenario::kPrecise, options.seed, 0));
  if (options.scenario == "calibrated_init" || options.scenario == "all")
    result.push_back(offline::makeScenario(
        manifest, offline::CalibrationScenario::kCalibratedInit,
        options.seed, 0));
  if (options.scenario == "calibration_sweep") {
    if (options.rotation_perturbations.empty())
      throw std::invalid_argument(
          "calibration_sweep requires --rotation-perturbations");
    for (const double degrees : options.rotation_perturbations)
      result.push_back(offline::makePerturbedCalibrationScenario(
          manifest, options.seed, degrees,
          options.translation_perturbation));
  }
  if (options.scenario == "coarse" || options.scenario == "all") {
    for (const auto level : coarseLevels(options.coarse_level))
      result.push_back(offline::makeScenario(
          manifest, offline::CalibrationScenario::kCoarse, options.seed, level));
  }
  if (options.scenario == "prior_free" || options.scenario == "all")
    result.push_back(offline::makeScenario(
        manifest, offline::CalibrationScenario::kPriorFree, options.seed, 0));
  if (result.empty())
    throw std::invalid_argument(
        "scenario must be precise, calibrated_init, calibration_sweep, coarse, prior_free, or all");
  return result;
}

std::vector<const offline::LidarConfig*> enabledLidars(
    const offline::OfflineManifest& manifest) {
  std::vector<const offline::LidarConfig*> result;
  for (const auto& lidar : manifest.lidars)
    if (lidar.enabled) result.push_back(&lidar);
  return result;
}

void configureEstimator(const offline::OfflineManifest& manifest,
                        const offline::ScenarioConfiguration& scenario,
                        const std::string& config_file,
                        bool reference_only = false) {
  readParameters(config_file);
  auto lidars = enabledLidars(manifest);
  if (reference_only) {
    lidars.erase(
        std::remove_if(lidars.begin(), lidars.end(),
                       [&](const offline::LidarConfig* lidar) {
                         return lidar->name != manifest.reference_lidar;
                       }),
        lidars.end());
  }
  for (const auto* lidar : lidars) {
    if (std::abs(lidar->scan_period - SCAN_PERIOD) > 1e-6)
      throw std::invalid_argument(
          "LiDAR scan_period must match the M-LOAM algorithm config: " +
          lidar->name);
  }
  NUM_OF_LASER = lidars.size();
  IDX_REF = 0;
  QBL.resize(NUM_OF_LASER);
  TBL.resize(NUM_OF_LASER);
  TDBL.resize(NUM_OF_LASER);
  CLOUD_TOPIC.resize(NUM_OF_LASER);
  COV_EXT.assign(NUM_OF_LASER, Eigen::Matrix<double, 6, 6>::Zero());
  for (std::size_t i = 0; i < lidars.size(); ++i) {
    if (lidars[i]->name == manifest.reference_lidar) IDX_REF = i;
    const auto& transform = scenario.initial_extrinsics.at(lidars[i]->name);
    QBL[i] = transform.rotation;
    TBL[i] = transform.translation;
    TDBL[i] = lidars[i]->time_offset;
  }
  N_SCANS = lidars[IDX_REF]->ring_count;
  HORIZON_SCAN = lidars[IDX_REF]->expected_width;
  ESTIMATE_EXTRINSIC = scenario.estimator_mode;
  ESTIMATE_TD = 0;
  MULTIPLE_THREAD = 0;
  // Prepared frames have already undergone each sensor's native ring/column
  // segmentation. Disable the legacy angle-derived global segmenter.
  SEGMENT_CLOUD = 0;
}

offline::EstimatorCallback estimatorCallback(
    Estimator& estimator, const offline::OfflineManifest& manifest,
    bool publish_ros, bool reference_only = false) {
  const auto all_lidars = enabledLidars(manifest);
  auto estimator_lidars = all_lidars;
  if (reference_only) {
    estimator_lidars.erase(
        std::remove_if(estimator_lidars.begin(), estimator_lidars.end(),
                       [&](const offline::LidarConfig* lidar) {
                         return lidar->name != manifest.reference_lidar;
                       }),
        estimator_lidars.end());
  }
  return [&estimator, all_lidars, estimator_lidars, publish_ros,
          reference_only](const offline::EstimatorInput& input) {
    std::vector<offline::PreparedLidarFrame> prepared_for_estimator;
    const std::vector<offline::PreparedLidarFrame>* prepared = &input.lidars;
    if (reference_only) {
      for (const auto& frame : input.lidars) {
        if (frame.lidar_name == estimator_lidars.front()->name)
          prepared_for_estimator.push_back(frame);
      }
      prepared = &prepared_for_estimator;
    }
    const auto odometry_begin = std::chrono::steady_clock::now();
    const auto estimate = estimator.processPreparedFrame(
        input.timestamp, *prepared, publish_ros);
    const double odometry_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - odometry_begin).count();
    LidarMapperFrameInput mapping_input;
    mapping_input.timestamp = input.timestamp;
    mapping_input.full_cloud = estimate.full_cloud;
    mapping_input.outlier_cloud = estimate.outlier_cloud;
    mapping_input.corner_cloud = estimate.corner_cloud;
    mapping_input.surface_cloud = estimate.surface_cloud;
    mapping_input.odometry = Pose(estimate.world_R_reference,
                                  estimate.world_t_reference);
    mapping_input.extrinsics.reserve(estimator_lidars.size());
    for (std::size_t i = 0; i < estimator_lidars.size(); ++i) {
      Pose extrinsic(estimate.reference_R_lidar[i],
                     estimate.reference_t_lidar[i]);
      extrinsic.cov_ = estimator.covbl_[i];
      mapping_input.extrinsics.push_back(extrinsic);
    }
    const auto mapping_begin = std::chrono::steady_clock::now();
    const auto mapped = processLidarMapperFrame(mapping_input);
    const double mapping_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - mapping_begin).count();
    offline::EstimatorOutput output;
    output.world_T_reference.rotation = mapped.pose.q_;
    output.world_T_reference.translation = mapped.pose.t_;
    output.save_keyframe = mapped.saved_keyframe;
    output.odometry_ms = odometry_ms;
    output.mapping_ms = mapping_ms;
    const auto collect_features = [&](const common::PointICloud& cloud) {
      for (const auto& point : cloud) {
        const int sensor = static_cast<int>(std::lround(point.intensity));
        if (sensor < 0 ||
            sensor >= static_cast<int>(estimator_lidars.size()))
          continue;
        output.feature_points_reference[estimator_lidars[sensor]->name]
            .push_back(
            {point.x, point.y, point.z, point.intensity});
      }
    };
    collect_features(estimate.corner_cloud);
    collect_features(estimate.surface_cloud);
    for (const auto* lidar : all_lidars) {
      const auto estimated = std::find_if(
          estimator_lidars.begin(), estimator_lidars.end(),
          [&](const offline::LidarConfig* candidate) {
            return candidate->name == lidar->name;
          });
      if (estimated == estimator_lidars.end()) {
        output.reference_T_lidar[lidar->name] =
            input.scenario->initial_extrinsics.at(lidar->name);
        output.observable[lidar->name] = false;
        output.corner_features[lidar->name] = 0;
        output.surface_features[lidar->name] = 0;
        continue;
      }
      const std::size_t i =
          static_cast<std::size_t>(estimated - estimator_lidars.begin());
      offline::RigidTransform transform;
      transform.rotation = estimate.reference_R_lidar[i];
      transform.translation = estimate.reference_t_lidar[i];
      output.reference_T_lidar[lidar->name] = transform;
      output.observable[lidar->name] = estimate.observable[i];
      output.corner_features[lidar->name] = estimate.corner_features[i];
      output.surface_features[lidar->name] = estimate.surface_features[i];
    }
    return output;
  };
}

std::string join(const std::string& root, const std::string& child) {
  return !root.empty() && root.back() == '/' ? root + child
                                             : root + '/' + child;
}

offline::RunResult runPass(
    const offline::OfflineManifest& manifest,
    const offline::ScenarioConfiguration& scenario,
    const std::string& algorithm_config, const std::string& output_directory,
    bool publish_ros, bool reference_only) {
  configureEstimator(manifest, scenario, algorithm_config, reference_only);
  Estimator estimator;
  estimator.setParameter();
  initializeLidarMapperCore();
  return offline::runOffline(
      manifest, scenario, output_directory,
      estimatorCallback(estimator, manifest, publish_ros, reference_only));
}

}  // namespace

int main(int argc, char** argv) {
  try {
    ros::init(argc, argv, "mloam_offline_runner",
              ros::init_options::AnonymousName |
                  ros::init_options::NoSigintHandler);
    const Options options = parseOptions(argc, argv);
    auto manifest = offline::loadManifest(options.manifest);
    if (!options.reference.empty()) manifest.reference_lidar = options.reference;
    if (!options.backend_mode.empty()) {
      manifest.joint_backend.mode = backendMode(options.backend_mode);
      manifest.joint_backend.enabled =
          manifest.joint_backend.mode !=
          offline::JointBackendMode::kDisabled;
    }
    if (options.begin != std::numeric_limits<std::size_t>::max())
      manifest.frame_begin = options.begin;
    if (options.end != std::numeric_limits<std::size_t>::max())
      manifest.frame_end = options.end;
    if (options.stride != 0) manifest.frame_stride = options.stride;
    if (!options.output_root.empty()) manifest.output_root = options.output_root;
    offline::applyLidarSelection(manifest, options.include, options.exclude);
    const std::string algorithm_config = options.mloam_config.empty()
                                             ? manifest.mloam_config
                                             : options.mloam_config;
    if (algorithm_config.empty())
      throw std::invalid_argument(
          "mloam_config in the manifest or --mloam-config is required");

    const auto configured_scenarios = scenarios(manifest, options);
    if (manifest.joint_backend.enabled) {
      for (const auto& scenario : configured_scenarios) {
        if (scenario.scenario == offline::CalibrationScenario::kPriorFree)
          throw std::invalid_argument(
              "the joint backend requires an extrinsic prior; "
              "prior_free is intentionally unsupported");
      }
    }

    int exit_code = 0;
    for (const auto& scenario : configured_scenarios) {
      offline::RunResult result;
      const bool coarse_bootstrap =
          manifest.joint_backend.enabled &&
          manifest.joint_backend.mode ==
              offline::JointBackendMode::kCoarseBootstrap;
      const bool coarse_retroactive =
          manifest.joint_backend.enabled &&
          manifest.joint_backend.mode ==
              offline::JointBackendMode::kCoarsePeriodic;
      if (coarse_bootstrap || coarse_retroactive) {
        auto initialization_manifest = manifest;
        if (coarse_bootstrap) {
          const std::size_t remaining =
              std::numeric_limits<std::size_t>::max() -
              initialization_manifest.frame_begin;
          const std::size_t bootstrap_end =
              initialization_manifest.joint_backend.bootstrap_frames >
                      remaining
                  ? std::numeric_limits<std::size_t>::max()
                  : initialization_manifest.frame_begin +
                        initialization_manifest.joint_backend.bootstrap_frames;
          initialization_manifest.frame_end =
              std::min(initialization_manifest.frame_end, bootstrap_end);
        }
        auto initialization_scenario = scenario;
        if (coarse_bootstrap) initialization_scenario.estimator_mode = 0;
        const std::string scenario_directory =
            join(manifest.output_root, scenario.name);
        const std::string initialization_name =
            coarse_bootstrap ? "bootstrap" : "coarse_pass";
        const auto initialization = runPass(
            initialization_manifest, initialization_scenario,
            algorithm_config,
            join(scenario_directory, initialization_name),
            options.publish_ros, coarse_bootstrap);
        std::cout << scenario.name << '/' << initialization_name << ": "
                  << offline::runStatusName(initialization.status)
                  << ", backend="
                  << offline::jointBackendStateName(
                         initialization.backend.state)
                  << ", processed=" << initialization.processed_frames
                  << ", dropped=" << initialization.dropped_frames
                  << std::endl;
        if (!initialization.backend.accepted) {
          result = initialization;
        } else {
          auto replay_manifest = manifest;
          if (replay_manifest.joint_backend.maximum_backend_passes > 1) {
            replay_manifest.joint_backend.mode =
                offline::JointBackendMode::kPreciseRefine;
          } else {
            replay_manifest.joint_backend.enabled = false;
            replay_manifest.joint_backend.mode =
                offline::JointBackendMode::kDisabled;
          }
          auto replay_scenario = scenario;
          replay_scenario.initial_extrinsics =
              initialization.final_extrinsics;
          replay_scenario.estimator_mode = 0;
          replay_scenario.scenario =
              offline::CalibrationScenario::kPrecise;
          result = runPass(replay_manifest, replay_scenario, algorithm_config,
                           scenario_directory, options.publish_ros, false);
        }
      } else {
        result = runPass(
            manifest, scenario, algorithm_config,
            join(manifest.output_root, scenario.name), options.publish_ros,
            false);
      }
      std::cout << scenario.name << ": " << offline::runStatusName(result.status)
                << ", backend="
                << offline::jointBackendStateName(result.backend.state)
                << ", processed=" << result.processed_frames
                << ", dropped=" << result.dropped_frames << std::endl;
      if (result.status == offline::RunStatus::kFailed) exit_code = 1;
      else if (result.status == offline::RunStatus::kNonConverged && exit_code == 0)
        exit_code = 2;
    }
    return exit_code;
  } catch (const std::exception& error) {
    std::cerr << "mloam_offline_runner: " << error.what() << std::endl;
    usage(std::cerr);
    return 1;
  }
}
