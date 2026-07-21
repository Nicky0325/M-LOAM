#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <string>

#include "mloam/offline/manifest.hpp"

namespace offline = mloam::offline;

namespace {

std::string writeManifest(const std::string& name, const std::string& text) {
  const std::string path = "/tmp/" + name;
  std::ofstream stream(path);
  stream << text;
  return path;
}

const char* kTwoLidarManifest = R"yaml(
dataset_root: /data/run
output_root: /tmp/results
reference_lidar: top
lidars:
  - name: top
    directory: top
    rings: 128
    horizontal_fov_deg: 360
    expected_width: 4
    color: [255, 0, 0]
    vehicle_T_lidar:
      translation: [1, 2, 3]
      rpy_deg: [10, 20, 30]
  - name: side
    directory: side
    rings: 32
    horizontal_fov_deg: 120
    expected_width: 8
    time_offset: -0.02
    color: [0, 255, 0]
    vehicle_T_lidar:
      translation: [2, 2, 3]
      rpy_deg: [10, 20, 30]
)yaml";

}  // namespace

TEST(Manifest, AppliesDefaultsAndParsesZyxExtrinsics) {
  const auto path = writeManifest("mloam_manifest.yaml", kTwoLidarManifest);
  const auto manifest = offline::loadManifest(path);

  EXPECT_DOUBLE_EQ(0.06, manifest.synchronization_threshold);
  EXPECT_EQ(1u, manifest.frame_stride);
  ASSERT_EQ(2u, manifest.lidars.size());
  EXPECT_DOUBLE_EQ(2e-6, manifest.lidars[0].timestamp_scale);

  const Eigen::Matrix3d expected =
      (Eigen::AngleAxisd(M_PI / 6.0, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(M_PI / 9.0, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(M_PI / 18.0, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  EXPECT_TRUE(manifest.lidars[0].vehicle_T_lidar.rotation.toRotationMatrix()
                  .isApprox(expected, 1e-12));
}

TEST(Manifest, ComputesReferenceRelativeTransforms) {
  const auto manifest =
      offline::loadManifest(writeManifest("mloam_relative.yaml", kTwoLidarManifest));
  const auto relative = offline::referenceRelativeExtrinsics(manifest);

  ASSERT_EQ(2u, relative.size());
  EXPECT_TRUE(relative.at("top").translation.isZero(1e-12));
  const Eigen::Quaterniond vehicle_R_top =
      Eigen::AngleAxisd(M_PI / 6.0, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(M_PI / 9.0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(M_PI / 18.0, Eigen::Vector3d::UnitX());
  EXPECT_TRUE(relative.at("side").translation.isApprox(
      vehicle_R_top.conjugate() * Eigen::Vector3d::UnitX(), 1e-12));
}

TEST(Manifest, RejectsDuplicateNamesAndMissingReference) {
  std::string duplicate = kTwoLidarManifest;
  const auto side = duplicate.find("name: side");
  duplicate.replace(side, std::string("name: side").size(), "name: top");
  EXPECT_THROW(offline::loadManifest(
                   writeManifest("mloam_duplicate.yaml", duplicate)),
               std::invalid_argument);

  std::string missing = kTwoLidarManifest;
  const auto reference = missing.find("reference_lidar: top");
  missing.replace(reference, std::string("reference_lidar: top").size(),
                  "reference_lidar: absent");
  EXPECT_THROW(offline::loadManifest(
                   writeManifest("mloam_missing_ref.yaml", missing)),
               std::invalid_argument);
}

TEST(Manifest, IncludeExcludeOverridesKeepReferenceAndValidateNames) {
  auto manifest =
      offline::loadManifest(writeManifest("mloam_override.yaml", kTwoLidarManifest));
  offline::applyLidarSelection(manifest, {"top", "side"}, {"side"});
  EXPECT_TRUE(manifest.lidars[0].enabled);
  EXPECT_FALSE(manifest.lidars[1].enabled);
  EXPECT_THROW(offline::applyLidarSelection(manifest, {"unknown"}, {}),
               std::invalid_argument);
  EXPECT_THROW(offline::applyLidarSelection(manifest, {"side"}, {}),
               std::invalid_argument);
}

TEST(Manifest, LoadsExistingStylePrototxtExtrinsicWithZyxComposition) {
  const std::string proto = "/tmp/mloam_extrinsic.prototxt";
  std::ofstream(proto) <<
      "translation { x: 1 y: 2 z: 3 }\n"
      "rotation { roll_deg: 10 pitch_deg: 20 yaw_deg: 30 }\n";
  const std::string yaml = R"yaml(
dataset_root: /data
output_root: /tmp/out
reference_lidar: top
lidars:
  - name: top
    directory: top
    rings: 32
    horizontal_fov_deg: 360
    expected_width: 4
    color: [1, 2, 3]
    extrinsic_prototxt: )yaml" + proto + "\n";
  const auto parsed = offline::loadManifest(
      writeManifest("mloam_prototxt_manifest.yaml", yaml));
  EXPECT_TRUE(parsed.lidars[0].vehicle_T_lidar.translation.isApprox(
      Eigen::Vector3d(1, 2, 3)));
  const Eigen::Quaterniond expected =
      Eigen::AngleAxisd(M_PI / 6.0, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(M_PI / 9.0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(M_PI / 18.0, Eigen::Vector3d::UnitX());
  EXPECT_TRUE(parsed.lidars[0].vehicle_T_lidar.rotation.isApprox(expected));
}

TEST(Manifest, LoadsAivRotationRpyDegPrototxtExtrinsic) {
  const std::string proto = "/tmp/mloam_aiv_extrinsic.prototxt";
  std::ofstream(proto) <<
      "vehicle_type: AIV\n"
      "rotation_rpy_deg { x: 10 y: 20 z: 30 }\n"
      "translation { x: 1 y: 2 z: 3 }\n";
  const std::string yaml = R"yaml(
dataset_root: /data
output_root: /tmp/out
reference_lidar: top
lidars:
  - name: top
    directory: top
    rings: 128
    horizontal_fov_deg: 120
    expected_width: 1300
    color: [1, 2, 3]
    extrinsic_prototxt: )yaml" + proto + "\n";
  const auto parsed = offline::loadManifest(
      writeManifest("mloam_aiv_prototxt_manifest.yaml", yaml));
  const Eigen::Quaterniond expected =
      Eigen::AngleAxisd(M_PI / 6.0, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(M_PI / 9.0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(M_PI / 18.0, Eigen::Vector3d::UnitX());
  EXPECT_TRUE(parsed.lidars[0].vehicle_T_lidar.translation.isApprox(
      Eigen::Vector3d(1, 2, 3)));
  EXPECT_TRUE(parsed.lidars[0].vehicle_T_lidar.rotation.isApprox(expected));
}
