# AIV5 planar multi-LiDAR rotation-calibration prototype

## Outcome

The original M-LOAM 6-DoF online calibration formulation is a poor match for
this rig and route. A rotation-only, known-translation hand-eye formulation
does tolerate a deterministic 10-degree error on every one of the six LiDARs
in this extraction.

With seed 42, all six estimates passed independent held-out consistency and
rotation-Hessian observability gates. Final errors against the supplied
calibration rotations are 0.29--0.82 degrees. Across all 15 LiDAR-to-LiDAR
pairs, relative rotation error is 0.43 degrees mean, 0.83 degrees p95, and 0.98
degrees maximum. Vehicle-frame translations are copied unchanged and never
enter an optimizer parameter block.

The resulting GNSS-anchored colored map is
[`map_optimized_rgb.pcd`](../data/aiv5_planar_nav_10deg/map_optimized_rgb.pcd).
The before/after bird's-eye comparison is
[`map_comparison_birdeye.png`](../data/aiv5_planar_nav_10deg/map_comparison_birdeye.png).

## Why this formulation

For vehicle motion `A_ij`, measured LiDAR motion `B_ij`, and the constant
vehicle-to-LiDAR transform `X`, the estimator minimizes the robust hand-eye
error

```text
B_ij = X^-1 A_ij X.
```

Only the rotation of `X` is variable. The supplied vehicle-frame translation
is fixed exactly. The implementation:

1. interpolates the dense `vehicle_pose` DR trajectory;
2. rigidly anchors it to local ENU using `ins.txt` positions;
3. chooses 40 motion pairs per LiDAR at a 1.5 s separation, prioritizing yaw
   excitation while retaining 0.8--6 m displacement;
4. estimates each `B_ij` with coarse-to-fine point-to-plane ICP;
5. robustly solves the three rotational degrees of freedom with a Cauchy loss;
6. uses a deterministic 80/20 train/held-out split and rejects updates that do
   not improve held-out motion consistency or have a degenerate Hessian; and
7. rebuilds a globally anchored map and emits a fixed-extrinsic M-LOAM
   manifest.

Known lever arms of roughly 7 m and the route's yaw turns make yaw observable
through the translational hand-eye equation. The measured LiDAR rotations and
scene geometry constrain the other two axes. This removes the translation/
rotation compensation seen in the unconstrained M-LOAM calibration.

## Dataset excitation

| Quantity | Measured value |
|---|---:|
| Dense DR poses | 6,594 |
| INS samples | 7,488 (all status 4) |
| Duration | 74.94 s |
| Path length | 153.29 m |
| Accumulated absolute yaw | 115.57 deg |
| Net yaw | 89.08 deg |
| Maximum angular rate | 28.19 deg/s |
| Roll span | 1.16 deg |
| Pitch span | 0.88 deg |
| DR-to-INS position alignment RMSE / p95 | 0.129 / 0.242 m |

This is enough excitation for the constrained problem. It is not enough to
justify unconstrained 6-DoF extrinsic calibration: roll/pitch motion is weak,
and translation is allowed to compensate rotation in the original objective.

## Ten-degree recovery

The trusted rotations are used only to inject and score the test. Each sensor
gets a different deterministic rotation axis; all translations remain at the
calibration-file values.

| LiDAR | Initial error (deg) | Final error (deg) | Hessian condition | Held-out translation RMSE, before -> after (m) |
|---|---:|---:|---:|---:|
| `lidar_fll_at128` | 10.000 | 0.543 | 14.7 | 0.443 -> 0.048 |
| `lidar_frf_at128` | 10.000 | 0.286 | 21.1 | 0.484 -> 0.051 |
| `lidar_brr_at128` | 10.000 | 0.822 | 29.4 | 0.511 -> 0.062 |
| `lidar_blb_at128` | 10.000 | 0.464 | 21.2 | 0.406 -> 0.049 |
| `lidar_fr_xt32` | 10.000 | 0.299 | 30.6 | 0.461 -> 0.074 |
| `lidar_bl_xt32` | 10.000 | 0.578 | 26.3 | 0.447 -> 0.109 |

The complete per-pair result is in
[`relative_extrinsics.yaml`](../data/aiv5_planar_nav_10deg/relative_extrinsics.yaml),
and full train/held-out diagnostics are in
[`summary.yaml`](../data/aiv5_planar_nav_10deg/summary.yaml).

## Baseline comparison

The existing MLCC-style batch backend was run on the same deterministic 10
degree perturbation using 299 synchronized frames (stride 2). It rejected its
update and left every non-reference rotation at exactly 10 degrees. Its plane
objective regressed:

| Backend objective | Initial | Final |
|---|---:|---:|
| Training | 0.03081 | 0.05867 |
| Held-out | 0.03229 | 0.05923 |

It also proposed 0.257--0.424 m translation changes even though the test did
not perturb translation. The baseline artifacts are under
`/tmp/mloam_backend_baseline/calibrated_init_10deg_0m/bootstrap` for this
session.

## Map result

The comparison map uses 75 scans from each LiDAR over the entire route (450
scans total) and a 0.35 m voxel size. Both maps use the identical
GNSS-anchored trajectory; only rotations differ.

| Map statistic | Injected 10 deg | Recovered |
|---|---:|---:|
| Colored points | 1,149,047 | 471,427 |
| Occupied voxels | 941,375 | 268,680 |
| Cross-LiDAR overlap within 1 m | 61.9% | 93.2% |
| Matched nearest-neighbor median / p95 | 0.294 / 0.875 m | 0.166 / 0.509 m |

The 71.5% occupied-voxel reduction and stronger cross-LiDAR overlap are
consistent with removal of duplicated and thickened geometry. These are
map-consistency indicators, not surveyed map ground truth. The colored
bird's-eye plot should also be inspected: repeated facades and road boundaries
collapse visibly after recovery.

The generated fixed-extrinsic manifest is
[`corrected_manifest.yaml`](../data/aiv5_planar_nav_10deg/corrected_manifest.yaml).
It completed a 20-frame, all-six-LiDAR M-LOAM smoke run with status `success`,
19 processed frames, one synchronization drop, and 0.098 m median cross-LiDAR
alignment distance.

A full fixed-extrinsic integration run at stride 2 also completed successfully:
379 frames were processed over 74.4 s, with one synchronization drop. Against
the dense vehicle-pose DR reference, after one rigid SE(3) alignment and no
scale fit, the run achieved 0.692 m ATE RMSE (1.384 m p95), 0.077 m / 0.167
degree 1-second RPE RMSE, and 0.292 m / 0.466 degree 10-metre RPE RMSE. Median /
p95 cross-LiDAR alignment distance was 0.061 / 0.359 m. The median runtime was
2.81 s per processed frame. The detailed report is
[`mloam_fixed_stride2/session_report.md`](../data/aiv5_planar_nav_10deg/mloam_fixed_stride2/session_report.md).
The runner logged 50 non-converged individual Ceres solves but continued and
produced a complete trajectory and 214 MiB map, so this remains a prototype
result rather than a real-time or production-readiness claim.

## Reproduce

```bash
python3 estimator/offline/tools/planar_nav_rotation_calibrator.py \
  --manifest estimator/config/offline/aiv5_sequence.yaml \
  --output-dir data/aiv5_planar_nav_10deg \
  --inject-rotation-error-deg 10 --seed 42 \
  --map-stride 10 --map-voxel-size 0.35
```

For a real coarse input rather than a scored perturbation test, omit
`--inject-rotation-error-deg`. Then run M-LOAM with the accepted fixed result:

```bash
rosrun mloam mloam_offline_runner \
  --manifest data/aiv5_planar_nav_10deg/corrected_manifest.yaml \
  --scenario precise
```

## Limitations and next data to collect

- This is navigation-aided mapping, not a pure LiDAR-only proof. Global
  consistency comes from anchoring the smooth DR trajectory to GNSS/INS. A
  GNSS-denied deployment still needs loop closure and pose-graph optimization.
- The supplied calibration is a reference, not independently surveyed ground
  truth. The reported angular values measure recovery to that reference.
- The recovery-basin experiment uses one deterministic seed (six different
  sensor-specific axes), not a Monte Carlo success-rate study. Repeat several
  seeds before treating 10 degrees as a production guarantee.
- Point-level motion compensation and LiDAR/pose time-offset calibration are
  not included. At the measured vehicle speed, scan distortion is a plausible
  contributor to the remaining sub-degree biases, especially on the XT32s.
- This route has little roll/pitch excitation. It is enough only because
  translation is fixed and navigation is available. For a navigation-free
  calibration sequence, collect banked/sloped motion and deliberate pitch/
  roll changes while keeping static structure in every sensor's field of view.
- A short lever arm, a stationary route, motion along only one direction, poor
  scan overlap, or GNSS/DR disagreement should be rejected as unobservable.
