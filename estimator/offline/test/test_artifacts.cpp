#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <utility>

#include "mloam/offline/artifacts.hpp"

namespace offline = mloam::offline;

namespace {

std::string readFile(const std::string& path) {
  std::ifstream stream(path);
  std::ostringstream contents;
  contents << stream.rdbuf();
  return contents.str();
}

bool exists(const std::string& path) {
  struct stat info {};
  return stat(path.c_str(), &info) == 0;
}

offline::OfflineManifest manifest() {
  offline::OfflineManifest result;
  result.dataset_root = "/fixture";
  result.output_root = "/tmp/results";
  result.reference_lidar = "top";
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
  return result;
}

offline::PreparedLidarFrame point(const std::string& name) {
  offline::PreparedLidarFrame frame;
  frame.lidar_name = name;
  frame.native_points.push_back({0, 0, 0, 1, 0, 0});
  return frame;
}

}  // namespace

TEST(Artifacts, WritesStableSchemasMapsAndPerSensorKeyframes) {
  const auto config = manifest();
  const auto scenario = offline::makeScenario(
      config, offline::CalibrationScenario::kCoarse, 42, 0);
  offline::LidarMapper mapper(config.lidars, 0.05);
  offline::MappingFrame frame;
  frame.frame_index = 3;
  frame.timestamp = 12.5;
  frame.lidars = {point("top"), point("side")};
  frame.reference_T_lidar = scenario.initial_extrinsics;
  mapper.processFrame(std::move(frame));

  offline::RunArtifacts artifacts;
  artifacts.addSynchronization({3, 12.5, "processed", "", 0.01, 0.02});
  artifacts.addRuntime({3, 1.0, 2.0, 3.0});
  artifacts.addFeatures({3, "top", 10, 20});
  artifacts.addTrajectory({3, 12.5, offline::RigidTransform()});
  artifacts.addExtrinsic({3, 12.5, "side",
                          offline::RigidTransform(), true, 1.0, 0.1,
                          offline::CalibrationState::kObservable});
  artifacts.addOnlineFeatures(
      offline::RigidTransform(),
      {{"top", {{1.0f, 2.0f, 3.0f, 0.0f}}}}, config.lidars);

  const std::string output = "/tmp/mloam_artifacts_" +
                             std::to_string(static_cast<long long>(getpid()));
  artifacts.write(output, config, scenario, mapper,
                  offline::referenceRelativeExtrinsics(config),
                  offline::RunStatus::kNonConverged);

  EXPECT_TRUE(exists(output + "/resolved_config.yaml"));
  EXPECT_TRUE(exists(output + "/summary.yaml"));
  EXPECT_TRUE(exists(output + "/synchronization.csv"));
  EXPECT_TRUE(exists(output + "/runtime.csv"));
  EXPECT_TRUE(exists(output + "/features.csv"));
  EXPECT_TRUE(exists(output + "/trajectory.csv"));
  EXPECT_TRUE(exists(output + "/extrinsics_history.csv"));
  EXPECT_TRUE(exists(output + "/observability_history.csv"));
  EXPECT_TRUE(exists(output + "/optimized_extrinsics.yaml"));
  EXPECT_TRUE(exists(output + "/map_online_rgb.pcd"));
  EXPECT_TRUE(exists(output + "/map_online_features_rgb.pcd"));
  EXPECT_TRUE(exists(output + "/map_top.pcd"));
  EXPECT_TRUE(exists(output + "/map_side.pcd"));
  EXPECT_TRUE(exists(output + "/map_merged_rgb.pcd"));
  EXPECT_TRUE(exists(output + "/map_final_rgb.pcd"));
  EXPECT_TRUE(exists(output + "/keyframes/top/000003.pcd"));
  EXPECT_TRUE(exists(output + "/keyframes/side/000003.pcd"));
  EXPECT_NE(std::string::npos,
            readFile(output + "/summary.yaml").find("status: non_converged"));
  EXPECT_NE(std::string::npos,
            readFile(output + "/summary.yaml").find("alignment_median_m:"));
  EXPECT_NE(std::string::npos,
            readFile(output + "/resolved_config.yaml").find("seed: 42"));
  EXPECT_NE(std::string::npos,
            readFile(output + "/resolved_config.yaml")
                .find("injected_perturbations:"));
  EXPECT_NE(std::string::npos,
            readFile(output + "/summary.yaml").find("convergence_frame:"));
  EXPECT_NE(std::string::npos,
            readFile(output + "/optimized_extrinsics.yaml")
                .find("vehicle_T_lidar:"));
  const std::string synchronization_header =
      "frame,timestamp,status,reason,max_raw_skew_s,max_corrected_skew_s\n";
  EXPECT_EQ(synchronization_header,
            readFile(output + "/synchronization.csv")
                .substr(0, synchronization_header.size()));
}
