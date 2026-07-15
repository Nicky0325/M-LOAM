#pragma once

#include <vector>

#include "common/types/type.h"
#include "../estimator/pose.h"

struct LidarMapperFrameInput
{
    double timestamp = 0.0;
    common::PointICloud full_cloud;
    common::PointICloud outlier_cloud;
    common::PointICloud surface_cloud;
    common::PointICloud corner_cloud;
    Pose odometry;
    std::vector<Pose> extrinsics;
};

// Reset and configure the state shared by the legacy ROS wrapper and the
// synchronous offline entrypoint. readParameters() must have been called.
void initializeLidarMapperCore(bool with_uncertainty = true,
                               double good_feature_ratio = 1.0);

// Run one frame through the original M-LOAM scan-to-map optimization core.
// The input clouds are in the reference-LiDAR frame and integer intensity
// carries the source LiDAR index, matching visualization.cpp's ROS contract.
struct LidarMapperFrameResult { Pose pose; bool saved_keyframe = false; };
LidarMapperFrameResult processLidarMapperFrame(const LidarMapperFrameInput &input);

int runLegacyLidarMapper(int argc, char **argv);
