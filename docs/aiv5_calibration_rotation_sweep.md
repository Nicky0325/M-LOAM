# AIV5 calibrated-initialization rotation recovery sweep

## Experiment

- Dataset: `/home/biaoding2/datasets/aiv5_sequence_extraction`
- Reference LiDAR: `lidar_fll_at128`
- Sensors: all six configured LiDARs
- Estimator mode: online extrinsic calibration (`ESTIMATE_EXTRINSIC=1`)
- Perturbations: rotation only, exactly 1, 3, 5, and 10 degrees
- Translation perturbation: 0 m
- Seed: 42
- Each non-reference LiDAR has its own deterministic random axis. The axis is
  held fixed across all four magnitudes.
- Reference trajectory: the supplied DR `vehicle_pose` stream, rigidly aligned
  with SE(3) and no fitted scale. It is not surveyed ground truth.
- Sequence result: 756/760 synchronized frames processed and 743 frames
  overlap the DR reference, covering 151.1 m.

All four runs completed, but every run is classified `non_converged` because
not all five non-reference extrinsics became observable and converged.

## Odometry and map summary

| Initial rotation | ATE RMSE (m) | ATE p95 (m) | RPE 1 s (m) | RPE 10 m (m) | Final-map points | Final map (MiB) |
|---:|---:|---:|---:|---:|---:|---:|
| 1 deg | 0.499 | 0.835 | 0.103 | 0.333 | 20,157,019 | 236.4 |
| 3 deg | 0.458 | 0.683 | 0.097 | 0.333 | 21,293,740 | 249.6 |
| 5 deg | 0.459 | 0.848 | 0.100 | 0.334 | 21,561,891 | 252.8 |
| 10 deg | 0.533 | 0.754 | 0.108 | 0.411 | 23,287,999 | 273.1 |

Trajectory metrics remain good even when calibration is wrong. They must not
be used alone to claim extrinsic recovery. The growing final-map point count is
consistent with more duplicated/misaligned geometry, but it is a supporting
indicator rather than a standalone map-quality metric.

## Final angular recovery by LiDAR

Each cell is `final angular error (recovery from injected error)`. Negative
recovery means the estimate diverged beyond its starting error.

| LiDAR | 1 deg | 3 deg | 5 deg | 10 deg |
|---|---:|---:|---:|---:|
| `lidar_frf_at128` | 0.506 deg (49%) | 2.955 deg (2%) | 3.407 deg (32%) | 10.984 deg (-10%) |
| `lidar_brr_at128` | 1.000 deg (0%) | 3.000 deg (0%) | 5.000 deg (0%) | 10.000 deg (0%) |
| `lidar_blb_at128` | 0.352 deg (65%) | 0.972 deg (68%) | 1.892 deg (62%) | 13.173 deg (-32%) |
| `lidar_fr_xt32` | 0.426 deg (57%) | 1.881 deg (37%) | 1.394 deg (72%) | 10.000 deg (0%) |
| `lidar_bl_xt32` | 2.418 deg (-142%) | 3.269 deg (-9%) | 6.524 deg (-30%) | 10.505 deg (-5%) |

No perturbation level recovered all sensors. The best-observed individual
sensors can partially recover from 1--5 degrees, but other sensors remain
unchanged or diverge.

## Translation drift caused by optimization

Translation was not perturbed. Therefore all final translation errors below
were introduced by online calibration:

| LiDAR | 1 deg run (m) | 3 deg run (m) | 5 deg run (m) | 10 deg run (m) |
|---|---:|---:|---:|---:|
| `lidar_frf_at128` | 0.102 | 0.616 | 0.705 | 0.813 |
| `lidar_brr_at128` | 0.000 | 0.000 | 0.000 | 0.000 |
| `lidar_blb_at128` | 0.124 | 0.137 | 0.253 | 0.644 |
| `lidar_fr_xt32` | 0.070 | 0.080 | 0.025 | 0.000 |
| `lidar_bl_xt32` | 0.756 | 0.909 | 0.825 | 0.585 |

The translation drift is additional evidence that weakly observable parameters
can compensate for rotation error rather than converge to the trusted
calibration.

## Observability and recovery basin

- At 1 degree, the same main sensor as the unperturbed run accumulated 25
  eligible windows; the remaining sensor counts were sparse (0, 1, 0, and 5).
- At 3 degrees, the formerly strong sensor lost eligibility; only sparse
  windows remained on two other sensors.
- At 5 and 10 degrees, all five non-reference sensors had zero eligible
  calibration windows in the later route segment.

For this seed and route, the calibration-recovery basin is therefore narrower
than 3 degrees for the full six-sensor system. Even 1 degree is only partially
recoverable because the vehicle motion does not excite every sensor/extrinsic.
This is a single deterministic-axis experiment, not a probability-of-success
estimate over random seeds.

## Coarse-to-fine map artifacts

Open `map_final_rgb.pcd` for the accumulated colored map and
`map_online_features_rgb.pcd` for the denser feature map. Each directory also
contains `map_online_rgb.pcd`, `map_merged_rgb.pcd`, and six per-LiDAR maps.

From coarse to fine:

1. `calibrated_init_10deg_0m/` -- unrecovered coarse endpoint
2. `calibrated_init_5deg_0m/` -- unrecovered/partially compensated
3. `calibrated_init_3deg_0m/` -- weak partial correction
4. `calibrated_init_1deg_0m/` -- best partial correction

The trusted/unperturbed calibrated-init result from the earlier session is the
fine reference for visual comparison:

`/home/biaoding2/datasets/aiv5_sequence_extraction/mloam_results/session_20260720_170045/full_complete/calibrated_init/map_final_rgb.pcd`

## Recommendation

Use the calibration folder as a trusted initial guess, and do not expect this
route to refine all six extrinsics reliably. Do not replace calibration files
with these optimized values automatically. For a genuine calibration
experiment, collect a route with stronger roll, pitch, yaw, and parallax for
every sensor, freeze unobservable parameters, and repeat multiple random axes
and seeds. A conservative operational policy is to reject online updates for a
LiDAR unless it has sufficient eligible windows and both rotation and
translation remain within bounded change from the trusted calibration.

