#include <gtest/gtest.h>

#include <fstream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

#include "mloam/offline/runner.hpp"

namespace offline = mloam::offline;

namespace {

void makeDir(const std::string& path) {
  ASSERT_TRUE(mkdir(path.c_str(), 0755) == 0 || errno == EEXIST);
}

void writePcd(const std::string& path, bool corrupt = false) {
  std::ofstream stream(path);
  stream << "VERSION 0.7\nFIELDS x y z reflectivity ring timestamp\n"
         << "SIZE 4 4 4 4 2 4\nTYPE F F F F U U\n"
         << "COUNT 1 1 1 1 1 1\nWIDTH 2\nHEIGHT 2\n"
         << "VIEWPOINT 0 0 0 1 0 0 0\nPOINTS 4\nDATA ascii\n"
         << "0 0 0 1 " << (corrupt ? 9 : 0) << " 1\n"
         << "1 0 0 2 0 2\n"
         << "0 1 0 3 1 3\n"
         << "1 1 0 4 1 4\n";
}

offline::OfflineManifest fixture(std::string* root_out) {
  const std::string root = "/tmp/mloam_fixture_" +
                           std::to_string(static_cast<long long>(getpid()));
  makeDir(root);
  makeDir(root + "/top");
  makeDir(root + "/side");
  for (int frame = 1; frame <= 4; ++frame)
    writePcd(root + "/top/" + std::to_string(frame) + ".pcd");
  writePcd(root + "/side/1.pcd");
  writePcd(root + "/side/2.pcd", true);
  writePcd(root + "/side/3.pcd");

  offline::OfflineManifest result;
  result.dataset_root = root;
  result.output_root = root + "/output";
  result.reference_lidar = "top";
  result.minimum_finite_ratio = 1.0;
  offline::LidarConfig top;
  top.name = "top";
  top.directory = "top";
  top.ring_count = 2;
  top.expected_width = 2;
  top.color = {255, 0, 0};
  offline::LidarConfig side = top;
  side.name = "side";
  side.directory = "side";
  side.color = {0, 255, 0};
  side.vehicle_T_lidar.translation.x() = 1.0;
  result.lidars = {top, side};
  *root_out = root;
  return result;
}

}  // namespace

TEST(OfflineIntegration, DropsMissingAndCorruptSetsAndContinuesStreaming) {
  std::string root;
  const auto config = fixture(&root);
  const auto scenario = offline::makeScenario(
      config, offline::CalibrationScenario::kPrecise, 42, 0);
  const auto precise = offline::referenceRelativeExtrinsics(config);
  auto estimator = [precise](const offline::EstimatorInput& input) {
    offline::EstimatorOutput output;
    output.world_T_reference.translation.x() = input.frame_index;
    output.reference_T_lidar = precise;
    return output;
  };

  const auto result = offline::runOffline(config, scenario,
                                           root + "/precise", estimator);
  EXPECT_EQ(2u, result.processed_frames);
  EXPECT_EQ(2u, result.dropped_frames);
  EXPECT_EQ(offline::RunStatus::kSuccess, result.status);
  std::ifstream sync(root + "/precise/synchronization.csv");
  const std::string contents((std::istreambuf_iterator<char>(sync)), {});
  EXPECT_NE(std::string::npos, contents.find("corrupt_pcd"));
  EXPECT_NE(std::string::npos, contents.find("incomplete_set"));
}

TEST(OfflineIntegration, RetainsArtifactsAndReturnsDistinctNonConvergence) {
  std::string root;
  const auto config = fixture(&root);
  const auto scenario = offline::makeScenario(
      config, offline::CalibrationScenario::kPriorFree, 42, 0);
  auto estimator = [](const offline::EstimatorInput&) {
    offline::EstimatorOutput output;
    output.reference_T_lidar["top"] = offline::RigidTransform();
    output.reference_T_lidar["side"] = offline::RigidTransform();
    output.observable["side"] = false;
    return output;
  };

  const auto result = offline::runOffline(config, scenario,
                                           root + "/prior_free", estimator);
  EXPECT_EQ(offline::RunStatus::kNonConverged, result.status);
  EXPECT_TRUE(std::ifstream(root + "/prior_free/map_final_rgb.pcd").good());
  EXPECT_TRUE(std::ifstream(root + "/prior_free/summary.yaml").good());
}
