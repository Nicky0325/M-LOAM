#include <gtest/gtest.h>

#include <fstream>
#include <string>

#include "mloam/offline/pcd_preprocessor.hpp"

namespace offline = mloam::offline;

namespace {

std::string writePcd(const std::string& name, std::size_t width,
                     std::size_t height, const std::string& points,
                     const std::string& fields =
                         "x y z reflectivity ring timestamp") {
  const std::string path = "/tmp/" + name;
  std::ofstream stream(path);
  stream << "# .PCD v0.7\nVERSION 0.7\nFIELDS " << fields << "\n";
  if (fields == "x y z reflectivity ring timestamp") {
    stream << "SIZE 4 4 4 4 2 4\nTYPE F F F F U U\nCOUNT 1 1 1 1 1 1\n";
  } else {
    stream << "SIZE 4 4 4 4 4\nTYPE F F F F U\nCOUNT 1 1 1 1 1\n";
  }
  stream << "WIDTH " << width << "\nHEIGHT " << height << "\n"
         << "VIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << width * height
         << "\nDATA ascii\n" << points;
  return path;
}

offline::LidarConfig config(std::size_t rings = 2,
                            std::size_t width = 4) {
  offline::LidarConfig result;
  result.name = "lidar";
  result.ring_count = rings;
  result.expected_width = width;
  result.timestamp_scale = 2e-6;
  return result;
}

const char* kOrganizedPoints =
    "0 0 0 10 1 3\n"
    "1 0 0 11 0 2\n"
    "2 0 0 12 1 1\n"
    "3 0 0 13 0 4\n"
    "4 0 0 14 1 5\n"
    "5 0 0 15 0 6\n"
    "6 0 0 16 1 7\n"
    "7 0 0 17 0 8\n";

}  // namespace

TEST(PcdPreprocessor, PreservesReflectivityTimeRingAndOrganizedOrder) {
  const auto path = writePcd("native_organized.pcd", 4, 2,
                             kOrganizedPoints);
  const auto prepared = offline::preparePcd(path, config(), 1.0);

  ASSERT_EQ(8u, prepared.native_points.size());
  EXPECT_FLOAT_EQ(10.0f, prepared.native_points[0].reflectivity);
  EXPECT_DOUBLE_EQ(6e-6, prepared.native_points[0].timestamp);
  ASSERT_EQ(8u, prepared.ring_ordered_points.size());
  EXPECT_FLOAT_EQ(1.0f, prepared.ring_ordered_points[0].x);
  EXPECT_FLOAT_EQ(3.0f, prepared.ring_ordered_points[1].x);
  EXPECT_FLOAT_EQ(0.0f, prepared.ring_ordered_points[4].x);
  EXPECT_NEAR(4e-6f, prepared.ring_ordered_points[0].intensity, 1e-9f);
  EXPECT_NEAR(1.0f + 6e-6f, prepared.ring_ordered_points[4].intensity,
              1e-7f);
  EXPECT_EQ((std::vector<std::size_t>{0, 4}), prepared.ring_start_indices);
  EXPECT_EQ((std::vector<std::size_t>{3, 7}), prepared.ring_end_indices);
}

TEST(PcdPreprocessor, FiltersNanButEnforcesFiniteRatio) {
  std::string points = kOrganizedPoints;
  points.replace(points.find("7 0 0"), 5, "nan 0 0");
  const auto path = writePcd("native_nan.pcd", 4, 2, points);

  const auto prepared = offline::preparePcd(path, config(), 0.8);
  EXPECT_EQ(7u, prepared.native_points.size());
  EXPECT_EQ(1u, prepared.discarded_non_finite);
  EXPECT_THROW(offline::preparePcd(path, config(), 0.9),
               offline::PcdValidationError);
}

TEST(PcdPreprocessor, RejectsRingDimensionsAndRequiredFieldErrors) {
  std::string bad_ring = kOrganizedPoints;
  bad_ring.replace(bad_ring.find("10 1 3"), 6, "10 2 3");
  EXPECT_THROW(offline::preparePcd(
                   writePcd("native_bad_ring.pcd", 4, 2, bad_ring),
                   config(), 1.0),
               offline::PcdValidationError);
  EXPECT_THROW(offline::preparePcd(
                   writePcd("native_bad_width.pcd", 8, 1,
                            kOrganizedPoints),
                   config(), 1.0),
               offline::PcdValidationError);

  std::string without_ring;
  for (int i = 0; i < 8; ++i) without_ring += "0 0 0 1 2\n";
  EXPECT_THROW(offline::preparePcd(
                   writePcd("native_missing_ring.pcd", 4, 2, without_ring,
                            "x y z reflectivity timestamp"),
                   config(), 1.0),
               offline::PcdValidationError);
}

TEST(PcdPreprocessor, SupportsMixed128And32RingGeometries) {
  std::string points128;
  for (int ring = 0; ring < 128; ++ring)
    points128 += "1 0 0 1 " + std::to_string(ring) + " 1\n";
  std::string points32;
  for (int ring = 0; ring < 32; ++ring)
    points32 += "1 0 0 1 " + std::to_string(ring) + " 1\n";
  const auto lidar128 = writePcd("native_128.pcd", 1, 128, points128);
  const auto lidar32 = writePcd("native_32.pcd", 1, 32, points32);
  EXPECT_EQ(128u, offline::preparePcd(lidar128, config(128, 1), 1.0)
                    .ring_ordered_points.size());
  EXPECT_EQ(32u, offline::preparePcd(lidar32, config(32, 1), 1.0)
                    .ring_ordered_points.size());
}

TEST(PcdPreprocessor, RejectsUnorganizedHeightAndUnsafeRelativeTimes) {
  EXPECT_THROW(offline::preparePcd(
                   writePcd("native_unorganized.pcd", 8, 1,
                            kOrganizedPoints), config(2, 8), 1.0),
               offline::PcdValidationError);

  std::string negative = kOrganizedPoints;
  negative.replace(negative.find("10 1 3"), 6, "10 1 -1");
  EXPECT_THROW(offline::preparePcd(
                   writePcd("native_negative_time.pcd", 4, 2, negative),
                   config(), 1.0),
               offline::PcdValidationError);

  std::string overflow = kOrganizedPoints;
  overflow.replace(overflow.find("10 1 3"), 6, "10 1 500000");
  EXPECT_THROW(offline::preparePcd(
                   writePcd("native_overflow_time.pcd", 4, 2, overflow),
                   config(), 1.0),
               offline::PcdValidationError);
}

TEST(PcdPreprocessor, AppliesPerLidarNativeRangeImageSegmentation) {
  const char* points =
      "1 0 0 1 0 1\n"
      "0 1 0 1 0 2\n"
      "nan 0 0 1 0 3\n"
      "100 0 0 1 0 4\n"
      "1 0 0.1 1 1 5\n"
      "0 1 0.1 1 1 6\n"
      "nan 0 0 1 1 7\n"
      "nan 0 0 1 1 8\n";
  auto lidar = config();
  lidar.horizontal_fov_deg = 120.0;
  lidar.segment_cloud = true;
  lidar.min_cluster_size = 5;
  lidar.segment_valid_point_num = 4;
  lidar.segment_valid_line_num = 2;
  lidar.segment_theta_rad = 0.1;
  const auto prepared = offline::preparePcd(
      writePcd("native_segmented.pcd", 4, 2, points), lidar, 0.6);
  EXPECT_EQ(4u, prepared.ring_ordered_points.size());
  ASSERT_EQ(1u, prepared.outlier_ring_ordered_points.size());
  EXPECT_FLOAT_EQ(100.0f, prepared.outlier_ring_ordered_points[0].x);
}
