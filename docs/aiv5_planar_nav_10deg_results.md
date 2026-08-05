# Superseded AIV5 navigation-aided result

This report previously described an experiment that used a LiDAR-localizer
vehicle pose and a downstream INS text result. That experiment is withdrawn as
calibration evidence because the pose source depends on precise LiDAR
extrinsics and the INS text is not guaranteed to be a raw, available sensor
input.

Do not use the old result to evaluate calibration robustness or global map
consistency. The implementation and experiment were replaced by a run that
uses only raw IMU samples and raw dual-antenna GNSS records:

- [Current AIV5 raw-IMU/GNSS 10-degree report](aiv5_raw_imu_gnss_10deg_results.md)
- [Current raw-sensor methodology](methodology/fixed_translation_planar_multi_lidar_calibration.md)

The obsolete output directory `data/aiv5_planar_nav_10deg/`, if present in a
working copy, is historical and is not consumed by the current workflow.
