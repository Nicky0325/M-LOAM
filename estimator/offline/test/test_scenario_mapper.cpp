#include <gtest/gtest.h>

#include "mloam/offline/lidar_mapper.hpp"
#include "mloam/offline/scenario.hpp"

namespace offline = mloam::offline;

namespace {

offline::OfflineManifest manifest() {
  offline::OfflineManifest result;
  result.dataset_root = "/data";
  result.output_root = "/tmp";
  result.reference_lidar = "top";
  offline::LidarConfig top;
  top.name = "top";
  top.directory = "top";
  top.ring_count = 128;
  top.expected_width = 2;
  top.color = {255, 0, 0};
  offline::LidarConfig side;
  side.name = "side";
  side.directory = "side";
  side.ring_count = 32;
  side.expected_width = 2;
  side.color = {0, 255, 0};
  side.vehicle_T_lidar.translation = Eigen::Vector3d(1.0, 0.0, 0.0);
  result.lidars = {top, side};
  return result;
}

double rotationErrorDegrees(const offline::RigidTransform& lhs,
                            const offline::RigidTransform& rhs) {
  return Eigen::AngleAxisd(lhs.rotation.conjugate() * rhs.rotation).angle() *
         180.0 / M_PI;
}

offline::PreparedLidarFrame onePoint(const std::string& name, float x) {
  offline::PreparedLidarFrame result;
  result.lidar_name = name;
  result.native_points.push_back({x, 0.0f, 0.0f, 1.0f, 0, 0.0});
  result.ring_ordered_points.push_back({x, 0.0f, 0.0f, 0.0f});
  return result;
}

}  // namespace

TEST(Scenario, ProducesDeterministicExactMagnitudeCoarsePerturbations) {
  const auto first = offline::makeScenario(manifest(),
      offline::CalibrationScenario::kCoarse, 42, 1);
  const auto second = offline::makeScenario(manifest(),
      offline::CalibrationScenario::kCoarse, 42, 1);
  const auto precise = offline::referenceRelativeExtrinsics(manifest());

  EXPECT_EQ(1, first.estimator_mode);
  EXPECT_EQ("coarse_15deg_0.75m", first.name);
  EXPECT_TRUE(first.initial_extrinsics.at("side").translation.isApprox(
      second.initial_extrinsics.at("side").translation, 0.0));
  EXPECT_DOUBLE_EQ(0.75,
      (first.initial_extrinsics.at("side").translation -
       precise.at("side").translation).norm());
  EXPECT_NEAR(15.0,
      rotationErrorDegrees(first.initial_extrinsics.at("side"),
                           precise.at("side")), 1e-10);
  EXPECT_TRUE(first.initial_extrinsics.at("top").translation.isZero());
}

TEST(Scenario, PreciseAndPriorFreeSelectExpectedEstimatorModes) {
  const auto precise = offline::makeScenario(manifest(),
      offline::CalibrationScenario::kPrecise, 42, 0);
  const auto prior = offline::makeScenario(manifest(),
      offline::CalibrationScenario::kPriorFree, 42, 0);
  EXPECT_EQ(0, precise.estimator_mode);
  EXPECT_EQ(2, prior.estimator_mode);
  EXPECT_DOUBLE_EQ(1.0,
      precise.initial_extrinsics.at("side").translation.x());
  EXPECT_TRUE(prior.initial_extrinsics.at("side").translation.isZero());
  EXPECT_TRUE(prior.initial_extrinsics.at("side").rotation.isApprox(
      Eigen::Quaterniond::Identity()));
}

TEST(Scenario, TracksObservableConvergedAndNonConvergedStates) {
  offline::CalibrationStateTracker tracker(1.0, 0.05);
  tracker.update("side", 0, 0.0, false, 10.0, 1.0, false);
  EXPECT_EQ(offline::CalibrationState::kInitializing,
            tracker.history("side").back().state);
  tracker.update("side", 1, 0.1, true, 5.0, 0.5, false);
  EXPECT_EQ(offline::CalibrationState::kObservable,
            tracker.history("side").back().state);
  tracker.update("side", 2, 0.2, true, 0.5, 0.01, false);
  EXPECT_EQ(offline::CalibrationState::kConverged,
            tracker.history("side").back().state);

  offline::CalibrationStateTracker never_converges(1.0, 0.05);
  never_converges.update("side", 0, 0.0, true, 5.0, 0.5, false);
  never_converges.finalize(1, 0.1);
  EXPECT_EQ(offline::CalibrationState::kNonConverged,
            never_converges.history("side").back().state);
}

TEST(LidarMapper, PreservesProvenanceAndRebuildsWithFinalExtrinsics) {
  const auto config = manifest();
  offline::LidarMapper mapper(config.lidars, 0.01);
  offline::MappingFrame frame;
  frame.frame_index = 7;
  frame.timestamp = 1.0;
  frame.lidars = {onePoint("top", 0.0f), onePoint("side", 0.0f)};
  frame.reference_T_lidar["top"] = offline::RigidTransform();
  frame.reference_T_lidar["side"].translation = Eigen::Vector3d(5, 0, 0);
  mapper.processFrame(frame);

  EXPECT_EQ(1u, mapper.keyframes("top").size());
  EXPECT_EQ(1u, mapper.keyframes("side").size());
  const auto online_map = mapper.buildOnlineRgbMap();
  ASSERT_EQ(2u, online_map.size());
  EXPECT_FLOAT_EQ(5.0f, online_map[1].x);
  auto final_ext = offline::referenceRelativeExtrinsics(config);
  const auto map = mapper.rebuildFinalRgbMap(final_ext);
  ASSERT_EQ(2u, map.size());
  EXPECT_FLOAT_EQ(0.0f, map[0].x);
  EXPECT_FLOAT_EQ(1.0f, map[1].x);
  EXPECT_EQ(255, map[0].r);
  EXPECT_EQ(255, map[1].g);
}
