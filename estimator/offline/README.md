# Offline multi-LiDAR runner

`mloam_offline_runner` streams numeric-timestamp PCD files directly into the
M-LOAM estimator. It does not replay ROS topics. ROS initialization remains in
the executable only because the legacy estimator library uses ROS time and can
optionally publish its existing visualization topics.

Each PCD must be organized and contain scalar `x`, `y`, `z`, `reflectivity`,
`ring`, and `timestamp` fields. `ring` must be unsigned; timestamps are scaled
by each LiDAR's `timestamp_scale` (default `2e-6` seconds) and must not exceed
its `scan_period` (default `0.1` seconds). Points are kept in
native organized-column order inside each supplied ring. Non-finite or corrupt
synchronized sets are logged, dropped as a unit, and processing continues.
Optional per-LiDAR `segment_cloud` processing uses that native ring/column
grid (never angle-derived rings) with `min_cluster_size`,
`segment_valid_point_num`, `segment_valid_line_num`, and `segment_theta`.
Rejected clusters are passed to M-LOAM's mapper as sensor-provenance-preserving
outliers.

Build with the normal catkin/colcon workspace, then run:

```bash
rosrun mloam mloam_offline_runner \
  --manifest estimator/config/offline/example_six_lidar.yaml \
  --scenario all --seed 42
```

Useful overrides are `--include`, `--exclude`, `--reference`, `--begin`,
`--end`, `--stride`, `--output-root`, `--coarse-level`, and `--ros-publish`.
Exit status `0` means convergence/success, `2` means artifacts were retained
but at least one run did not converge, and `1` indicates failure.
