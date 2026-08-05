# AIV5 raw-IMU/GNSS multi-LiDAR 10-degree recovery

This is the dataset-specific evidence report for the
[fixed-translation raw-sensor methodology](methodology/fixed_translation_planar_multi_lidar_calibration.md).

## Outcome

The six-LiDAR rig recovered successfully from a deterministic 10-degree
rotation perturbation on every LiDAR using only raw IMU and raw dual-antenna
GNSS for rig motion. All six solutions passed independent held-out motion and
rotation-Hessian gates. No LiDAR translation changed.

Final per-LiDAR rotation error against the supplied reference was
0.632--1.130 degrees. Across all 15 LiDAR pairs, relative rotation error was
0.644 degrees mean, 1.233 degrees p95, and 1.381 degrees maximum.

The recovered ENU colored map is
[`map_optimized_rgb.pcd`](../data/aiv5_raw_imu_gnss_10deg/map_optimized_rgb.pcd).
The complete machine-readable result is
[`summary.yaml`](../data/aiv5_raw_imu_gnss_10deg/summary.yaml).

## Inputs and leakage check

The run consumed:

- 7,489 raw IMU records from `imu.txt`;
- 375 raw GNSS position records from `gnss/*.prototxt`;
- 375 raw, usable dual-antenna GNSS heading records from those files;
- timestamped PCD scans from all six LiDAR directories; and
- the supplied vehicle-frame LiDAR translations and coarse rotations.

It did not consume the LiDAR-localizer vehicle-pose topic/files, the final INS
text result, or wheel data. The CLI exposes raw `--imu-file` and `--gnss-dir`
overrides and no path for any of the excluded sources. `summary.yaml` records
the three non-use flags explicitly.

The corrected manifest also disables the MLCC-style joint backend. The result
does not include an MLCC update.

## Raw-navigation quality and excitation

Raw LLA was converted to WGS84 ENU, with measured ENU velocity used for cubic
Hermite interpolation to IMU rate. Gyro-z was initialized during a stationary
interval, integrated, and robustly anchored to the dual-antenna headings. A
single shared planar GNSS antenna lever arm was then alternated with the six
LiDAR rotation solves.

| Quantity | Measured value |
|---|---:|
| Dense raw-navigation poses | 7,446 |
| Covered duration | 74.48 s |
| Raw-GNSS path length | 152.89 m |
| Accumulated absolute yaw | 115.65 deg |
| Net yaw | 89.03 deg |
| Maximum absolute corrected gyro-z | 10.08 deg/s |
| GNSS position/heading status | 375 / 375 RTK_FIXED |
| Maximum position/heading gap | 0.40 / 0.40 s |
| Stationary IMU samples | 936 |
| Fixed gravity roll/pitch | +0.275 / +0.375 deg |
| IMU-integrated yaw vs GNSS-heading RMSE / p95 | 0.062 / 0.116 deg |

The dual-antenna heading baseline is not aligned with vehicle x. The fallback
straight-course estimate found a heading-to-vehicle yaw of +80.049 degrees
from 91 samples, with 2.541 degrees p95 scatter. This is the run's only
vehicle-motion assumption: on low-yaw-rate segments above 2 m/s, sideslip is
small enough that GNSS velocity course approximates vehicle x. No wheel speed,
steering angle, wheelbase, or Ackermann model was used. A surveyed mounting
yaw can be passed instead to remove that assumption.

The shared planar antenna lever-arm nuisance estimate was
`[7.3484, 1.3503, 0]` m in the vehicle frame. Its 2D Hessian condition number
was 1.013. The vertical component was fixed because it is unobservable under
the planar model. This estimate does not alter any LiDAR translation and should
not be treated as a surveyed antenna location.

## Ten-degree recovery

The reference rotations were used only to generate a deterministic
sensor-specific 10-degree perturbation and to score the answer after solving.
They were not used as optimizer priors or residuals.

| LiDAR | Initial error (deg) | Final error (deg) | Hessian condition | Held-out translation RMSE, before -> after (m) |
|---|---:|---:|---:|---:|
| `lidar_fll_at128` | 10.000 | 0.678 | 15.55 | 0.455 -> 0.037 |
| `lidar_frf_at128` | 10.000 | 0.786 | 21.30 | 0.484 -> 0.045 |
| `lidar_brr_at128` | 10.000 | 0.675 | 30.63 | 0.500 -> 0.065 |
| `lidar_blb_at128` | 10.000 | 0.632 | 21.26 | 0.417 -> 0.061 |
| `lidar_fr_xt32` | 10.000 | 1.130 | 31.55 | 0.457 -> 0.076 |
| `lidar_bl_xt32` | 10.000 | 0.662 | 26.47 | 0.484 -> 0.110 |

Five LiDARs accepted all 40 attempted ICP constraints; `lidar_fr_xt32`
accepted 37. The deterministic 80/20 split left eight held-out constraints per
LiDAR. All fixed translations in
[`corrected_manifest.yaml`](../data/aiv5_raw_imu_gnss_10deg/corrected_manifest.yaml)
are numerically exact copies of the manifest values.

The complete relative transforms are in
[`relative_extrinsics.yaml`](../data/aiv5_raw_imu_gnss_10deg/relative_extrinsics.yaml),
and individual scan-pair diagnostics are in
[`pair_constraints.csv`](../data/aiv5_raw_imu_gnss_10deg/pair_constraints.csv).

## Globally referenced map

The before and after maps use the identical raw-GNSS/IMU trajectory, estimated
GNSS antenna lever arm, 75 scans per LiDAR, and 0.35 m voxels. Only the injected
versus recovered LiDAR rotations differ.

| Map statistic | Injected 10 deg | Recovered |
|---|---:|---:|
| Colored points | 1,193,964 | 497,688 |
| Occupied voxels | 955,115 | 286,107 |
| Cross-LiDAR overlap within 1 m | 62.24% | 93.83% |
| Matched nearest-neighbour median | 0.287 m | 0.173 m |
| Matched nearest-neighbour p95 | 0.872 m | 0.508 m |

Occupied voxels fell by 70.04%, and cross-LiDAR overlap improved by 31.59
percentage points. The reduction in duplicated/thickened geometry is strong
evidence of multi-LiDAR consistency.

The map is globally referenced in a local ENU frame because its positions come
directly from raw RTK GNSS and its yaw is anchored to raw dual-antenna heading.
These overlap statistics are not surveyed absolute map error, and this run is
not evidence of GNSS-denied loop closure.

## Reproduce

From the repository root:

```bash
python3 estimator/offline/tools/planar_nav_rotation_calibrator.py \
  --manifest estimator/config/offline/aiv5_sequence.yaml \
  --output-dir data/aiv5_raw_imu_gnss_10deg \
  --inject-rotation-error-deg 10 --seed 42 \
  --map-stride 10 --map-voxel-size 0.35
```

For an actual coarse calibration, omit `--inject-rotation-error-deg`. If the
dual-antenna baseline mounting yaw relative to vehicle x is known, add:

```bash
  --gnss-heading-to-vehicle-yaw-deg <measured-yaw>
```

The generated fixed-extrinsic M-LOAM manifest is
[`corrected_manifest.yaml`](../data/aiv5_raw_imu_gnss_10deg/corrected_manifest.yaml).

## Is wheel or a vehicle model required?

Not for this sequence. Continuous RTK-fixed position and dual-antenna heading,
high-rate IMU, a stationary initialization interval, straight segments, and
substantial yaw excitation are sufficient.

Additional wheel/steering/kinematic information becomes necessary or valuable
when dual-antenna heading is absent, GNSS has long outages, straight-line
mounting-yaw initialization is unavailable or invalid because of sideslip, or
GNSS-denied global consistency is required. If the heading mounting yaw can be
measured, no vehicle-kinematics fallback is needed for the current data.

## Limitations

- This is one deterministic 10-degree perturbation seed, not a Monte Carlo
  recovery probability.
- The supplied rotations are a reference rather than independently surveyed
  ground truth.
- Roll/pitch are fixed from stationary gravity; full 3D IMU preintegration is
  not implemented.
- IMU-to-vehicle roll/pitch alignment is assumed.
- Time-offset estimation and point-level LiDAR deskew are not implemented.
- The planar GNSS antenna lever arm's z component is not estimated.
- ICP measurements are not recomputed after calibration updates.
- GNSS-denied operation still needs loop closure and pose-graph optimization.
