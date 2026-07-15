#include <gtest/gtest.h>

#include <cerrno>
#include <fstream>
#include <sys/stat.h>
#include <unistd.h>

#include "mloam/offline/sequence_reader.hpp"

namespace offline = mloam::offline;

TEST(SequenceReader, IndexesOnlyNumericPcdFilenamesInTimestampOrder) {
  const std::string directory =
      "/tmp/mloam_index_" + std::to_string(static_cast<long long>(getpid()));
  ASSERT_TRUE(mkdir(directory.c_str(), 0755) == 0 || errno == EEXIST);
  std::ofstream(directory + "/10.200.pcd");
  std::ofstream(directory + "/9.900.pcd");
  std::ofstream(directory + "/notes.pcd");
  std::ofstream(directory + "/10.000.txt");

  const auto index = offline::SequenceIndex::fromDirectory("top", directory,
                                                            0.0);
  ASSERT_EQ(2u, index.scans().size());
  EXPECT_DOUBLE_EQ(9.9, index.scans()[0].timestamp);
  EXPECT_DOUBLE_EQ(10.2, index.scans()[1].timestamp);
}

TEST(SequenceReader, MatchesNearestUnusedScansUsingCorrectedTime) {
  offline::SequenceIndex reference("top", 0.0,
      {{10.000, "top/10.000.pcd"}, {10.100, "top/10.100.pcd"}});
  offline::SequenceIndex side("side", -0.020,
      {{10.015, "side/10.015.pcd"}, {10.025, "side/10.025.pcd"},
       {10.115, "side/10.115.pcd"}});

  offline::FrameSynchronizer synchronizer("top", 0.060,
                                           {reference, side});
  const auto frames = synchronizer.match(0, 2, 1);

  ASSERT_EQ(2u, frames.size());
  ASSERT_EQ(2u, frames[0].scans.size());
  EXPECT_DOUBLE_EQ(10.015, frames[0].scans.at("side").raw_timestamp);
  EXPECT_NEAR(-0.005, frames[0].scans.at("side").corrected_skew, 1e-12);
  EXPECT_DOUBLE_EQ(10.115, frames[1].scans.at("side").raw_timestamp);
}

TEST(SequenceReader, DoesNotReuseAScanAcrossReferenceFrames) {
  offline::SequenceIndex reference("top", 0.0,
      {{1.000, "top/1.000.pcd"}, {1.040, "top/1.040.pcd"}});
  offline::SequenceIndex side("side", 0.0, {{1.020, "side/1.020.pcd"}});

  offline::FrameSynchronizer synchronizer("top", 0.030,
                                           {reference, side});
  const auto frames = synchronizer.match(0, 2, 1);

  ASSERT_EQ(2u, frames.size());
  EXPECT_TRUE(frames[0].complete);
  EXPECT_FALSE(frames[1].complete);
  ASSERT_EQ(1u, frames[1].missing_lidars.size());
  EXPECT_EQ("side", frames[1].missing_lidars[0]);
}
