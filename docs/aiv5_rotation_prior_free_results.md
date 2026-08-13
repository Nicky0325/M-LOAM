# AIV5 rotation-prior-free calibration and mapping result

Date: 2026-08-13

## Result

The six-LiDAR AIV5 sequence was calibrated without using any manifest LiDAR
rotation for initialization, correspondence prediction, residuals, or update
bounds. The only LiDAR extrinsic quantities admitted to calibration were the
translations in the vehicle frame; dataset rotations were revealed only for
post-run scoring. Raw `imu.txt`, raw dual-antenna GNSS prototxt records, and the
timestamped LiDAR PCDs were used; `vehicle_pose`, `ins.txt`, and wheel data were
not read.

All six initializations passed the independent-motion, signed-axis,
fixed-translation yaw, held-out residual, and Hessian gates. Compared only
after optimization with the rotations in the dataset calibration files, the
absolute rotation differences were 0.095--1.175 degrees. Across all 15 LiDAR
pairs, the relative rotation difference had mean 1.048 degrees, p95 1.534
degrees, and maximum 1.794 degrees.

The raw-GNSS-anchored map had cross-LiDAR nearest-neighbour median 0.204 m,
p95 0.591 m, and mean overlap within 1 m of 0.939. These are internal map
consistency measurements, not surveyed point accuracy.

## Data boundary and excitation

| Item | Measured value |
|---|---:|
| Raw IMU samples | 7,489 |
| Raw GNSS position / heading samples | 375 / 375 |
| Navigation duration | 74.48 s |
| Raw-GNSS path length | 152.89 m |
| Accumulated absolute yaw | 115.65 deg |
| Net yaw | 89.03 deg |
| Maximum angular rate | 10.08 deg/s |
| Maximum GNSS position / heading gap | 0.40 / 0.40 s |
| Stationary IMU samples | 936 |
| Straight samples for heading-frame alignment | 91 |

Every GNSS position and heading record used by the run reported `RTK_FIXED`.
The raw gyro/dual-heading fit had 0.062-degree RMSE and 0.116-degree p95. The
estimated constant heading-frame-to-vehicle yaw was 80.049 degrees; its
straight-course validation p95 was 2.541 degrees.

This sequence has enough turning and lever-arm motion for the constrained
problem. That conclusion does not extend to a straight or stationary
sequence, nor to a solve in which LiDAR translations are also unknown.

## Rotation-prior-free initialization

Each LiDAR pair was registered from the identity transform. The raw navigation
rotation magnitude, which is invariant under hand-eye conjugation, was used
only to reject implausible ICP hypotheses. Signed ICP rotation axes recovered
the vehicle planar axis in each LiDAR frame. A global solve then estimated one
yaw per LiDAR and a shared planar GNSS-antenna lever arm using the known LiDAR
translations.

The selected eight-state solve comprised six LiDAR yaws and two planar antenna
lever components. Its Hessian eigenvalues ranged from 11.79 to 3098.57, for a
condition number of 262.83. The estimated vehicle-to-GNSS-antenna translation
was `[7.3086, 1.3053, 0.0]` m; the vertical component is fixed because it is
unobservable under planar motion.

| LiDAR | Accepted motions | Axis p95 | Held-out translation RMSE | Held-out rotation RMSE | Difference from manifest after solve |
|---|---:|---:|---:|---:|---:|
| `lidar_fll_at128` | 35 / 40 | 3.477 deg | 0.040 m | 0.217 deg | 0.655 deg |
| `lidar_frf_at128` | 33 / 40 | 3.456 deg | 0.043 m | 0.170 deg | 0.095 deg |
| `lidar_brr_at128` | 23 / 40 | 3.309 deg | 0.115 m | 0.191 deg | 0.769 deg |
| `lidar_blb_at128` | 31 / 40 | 3.560 deg | 0.037 m | 0.148 deg | 0.802 deg |
| `lidar_fr_xt32` | 32 / 40 | 3.758 deg | 0.088 m | 0.218 deg | 0.721 deg |
| `lidar_bl_xt32` | 31 / 40 | 3.660 deg | 0.277 m | 0.642 deg | 1.175 deg |

The local three-degree-of-freedom hand-eye refinement proposed after the
global initializer was not selected for any LiDAR because its combined
held-out translation/rotation score was worse. Retaining the global
initializer avoided overfitting the training motions.

## Map consistency

The map used 75 scans per LiDAR (`--map-stride 10`) and a 0.35 m output voxel.

| Metric | Prior-free result | Earlier injected-10-degree initial map | Earlier injected-10-degree optimized map |
|---|---:|---:|---:|
| Mean overlap fraction below 1 m | 0.939 | not reported here | not reported here |
| Matched nearest-neighbour median | 0.204 m | 0.287 m | 0.173 m |
| Matched nearest-neighbour p95 | 0.591 m | 0.872 m | 0.508 m |
| Occupied 0.35 m voxels | 314,595 | not reported here | not reported here |

The prior-free result is close to the earlier locally initialized result and
substantially better than that experiment's injected-10-degree initial map.
Because the trajectory is anchored directly by raw RTK GNSS and dual-antenna
heading, this is a globally referenced navigation-aided map. It is not proof
of GNSS-denied loop closure or a surveyed absolute map-error result.

## Rotation-only MLCC replay

The corrected manifest was also replayed through the optional MLCC-style
batch backend at stride four with `optimize_extrinsic_translation: false`.
The runner processed 189 synchronized frames and dropped one. The accepted
M-LOAM output reported alignment median 0.068 m and p95 0.414 m.

All backend translation updates were exactly zero. The proposed rotation
updates were 0.277, 0.803, 0.529, 0.929, and 1.192 degrees for the five
non-reference LiDARs. The backend correctly rejected the proposal: its
training plane objective changed from 0.03897 to 0.04354 and its held-out
objective changed from 0.03690 to 0.03903, a 5.77% regression. The retained
extrinsics and map are therefore the prior-free initializer result.

This experiment supports using MLCC as a guarded local refinement, not as the
global no-prior initializer. Rejection is a valid result: a backend proposal
must not be applied merely because its nonlinear solver converged.

## Reproduction

Run the no-prior calibration and build the ENU map:

```bash
python3 estimator/offline/tools/planar_nav_rotation_calibrator.py \
  --manifest estimator/config/offline/aiv5_sequence.yaml \
  --output-dir data/aiv5_rotation_prior_free \
  --rotation-initialization prior-free \
  --score-against-manifest \
  --pair-count 40 --minimum-constraints 12 \
  --map-stride 10 --map-voxel-size 0.35
```

`--score-against-manifest` is evaluation-only. Omit it for a real calibration;
in that form, a `vehicle_T_lidar` entry may contain only `translation`.

After building the ROS Noetic runner, replay the optional rotation-only MLCC
experiment:

```bash
source /opt/ros/noetic/setup.bash
source build/mloam/devel/setup.bash
build/mloam/devel/lib/mloam/mloam_offline_runner \
  --manifest data/aiv5_rotation_prior_free/corrected_manifest.yaml \
  --scenario precise \
  --backend-mode precise_refine \
  --stride 4 \
  --output-root data/aiv5_rotation_prior_free/mlcc_stride4
```

The primary artifacts are `summary.yaml`, `relative_extrinsics.yaml`,
`corrected_manifest.yaml`, `pair_constraints.csv`,
`trajectory_navigation.csv`, and the colored PCD maps. MLCC diagnostics are
under `mlcc_stride4/precise/`.

## Verification

- The full six-LiDAR, 40-pair AIV5 run passed and built the map reported above.
- A separate 12-pair AIV5 smoke run omitted `--score-against-manifest`; its
  summary recorded `score_against_manifest_after_optimization: false` and all
  six LiDARs passed. This exercises the path that does not parse or use
  rotations.
- The standalone C++ offline suite passed all seven tests.
- The Python calibration suite passed all ten tests, including an exact
  synthetic arbitrary-$SO(3)$ recovery with deliberately unrelated manifest
  rotations and a translation-only manifest parser test.
- The rotation-only MLCC test holds translation bit-exact while permitting a
  rotation proposal, and the AIV5 replay reported zero translation update for
  every LiDAR.

## Limitations

- Dataset rotations are a convenient scoring reference, not independently
  surveyed ground truth.
- Identity-seeded local ICP is still scene-dependent. Repetitive geometry,
  little overlap, motion distortion, or time offset can create wrong motions.
- The current navigation model fixes roll/pitch after stationary gravity
  initialization and does not jointly optimize IMU bias or time offset.
- Estimating heading-frame mounting yaw from GNSS course assumes small
  sideslip on selected straight segments. A surveyed mounting yaw is better.
- The 32-ring rear-left LiDAR is the weakest result and sets the observed
  1.175-degree absolute / 1.794-degree relative maximum.
- One successful sequence is not a production capture-rate guarantee.
- Wheel data is not required on this sequence. Raw encoder increments or raw
  speed could improve robustness during GNSS gaps, but the existing final
  wheel text result remains unusable.

The canonical algorithm description is in the
[methodology](methodology/fixed_translation_planar_multi_lidar_calibration.md),
with the derivation in the
[mathematical establishment](methodology/fixed_translation_planar_multi_lidar_calibration_math.md).
