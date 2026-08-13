# Running M-LOAM on multi-LiDAR PCD sequences

`mloam_offline_runner` streams organized PCD scans directly from disk into
M-LOAM odometry and mapping. It does not replay bags or transport point clouds
through ROS topics. ROS is still initialized because the existing estimator
uses ROS time, and `--ros-publish` can enable the existing visualization
publishers.

This file is the operational guide. The maintained problem formulation,
coordinate conventions, optimization residuals, validation gates, and
limitations are documented in
[`docs/methodology/fixed_translation_planar_multi_lidar_calibration.md`](../../docs/methodology/fixed_translation_planar_multi_lidar_calibration.md).
The corresponding equations and observability derivation are in the
[`mathematical establishment`](../../docs/methodology/fixed_translation_planar_multi_lidar_calibration_math.md).
The measured arbitrary-orientation result is in the
[`AIV5 rotation-prior-free report`](../../docs/aiv5_rotation_prior_free_results.md).

## Planar-motion, known-translation calibration

`planar_nav_rotation_calibrator.py` is the recommended prototype when the
vehicle-frame LiDAR translations are trusted and no usable rotation prior is
available. It uses only raw IMU and raw dual-antenna GNSS navigation, plus the
LiDAR PCD scans:

1. Convert raw GNSS fixes to ENU and interpolate position with measured ENU
   velocity.
2. Initialize gyro bias and fixed planar roll/pitch from stationary raw IMU.
3. Integrate gyro-z and robustly anchor yaw to raw dual-antenna GNSS heading.
4. Estimate the heading-baseline mounting yaw from straight GNSS velocity, or
   accept its measured value from the command line.
5. Select 0.8--6 m motion pairs and measure each LiDAR motion with
   identity-seeded multiscale point-to-plane ICP.
6. Recover a signed planar rotation axis for each LiDAR, reducing arbitrary
   orientation to one yaw per sensor.
7. Jointly solve all LiDAR yaws and one shared planar GNSS-antenna lever from
   the translational hand-eye equation and known LiDAR translations.
8. Retain a subsequent local rotation-only proposal only when held-out
   validation improves; every `vehicle_T_lidar` translation remains exact.
9. Publish a corrected manifest and raw-GNSS-anchored colored map only when
   all axis, held-out, and Hessian gates pass.

The LiDAR-localizer pose stream, final INS text output, and wheel result are
not accepted as inputs. The global initializer does not run the MLCC-style
backend. Its generated manifest disables MLCC by default and records a
rotation-only policy for an optional guarded replay.

Run the rotation-prior-free calibration on AIV5:

```bash
python3 estimator/offline/tools/planar_nav_rotation_calibrator.py \
  --manifest estimator/config/offline/aiv5_sequence.yaml \
  --output-dir data/aiv5_rotation_prior_free \
  --rotation-initialization prior-free \
  --pair-count 40 --minimum-constraints 12 \
  --map-stride 10 --map-voxel-size 0.35
```

In this mode a manifest `vehicle_T_lidar` may contain only `translation`; its
`rpy_deg` is neither required nor parsed. For a controlled dataset evaluation,
add `--score-against-manifest` to reveal manifest rotations only after the
solve.

Validate the 10-degree recovery basin deterministically on all enabled LiDARs:

```bash
python3 estimator/offline/tools/planar_nav_rotation_calibrator.py \
  --manifest estimator/config/offline/aiv5_sequence.yaml \
  --output-dir data/aiv5_raw_imu_gnss_10deg \
  --rotation-initialization manifest \
  --inject-rotation-error-deg 10 --seed 42 \
  --map-stride 10 --map-voxel-size 0.35
```

The tool requires Python 3, NumPy, SciPy, PyYAML, and Open3D. It produces:

- `summary.yaml`: raw-input quality, IMU/GNSS heading fit, GNSS lever arm,
  excitation, held-out residuals, observability, uncertainty, and (in
  injection mode) angular recovery error.
- `pair_constraints.csv`: every ICP motion pair and its train/held-out status.
- `corrected_manifest.yaml`: accepted rotations with the original translations
  copied unchanged, the joint backend disabled, and
  `optimize_extrinsic_translation: false`.
- `trajectory_navigation.csv`: GNSS-anchored vehicle and reference-LiDAR poses.
- `map_initial_rgb.pcd` and `map_optimized_rgb.pcd`: before/after colored maps,
  plus one optimized map per LiDAR.

Run fixed-extrinsic M-LOAM with the accepted result:

```bash
rosrun mloam mloam_offline_runner \
  --manifest data/aiv5_rotation_prior_free/corrected_manifest.yaml \
  --scenario precise
```

Planar motion is not sufficient for unconstrained 6-DoF hand-eye calibration.
It is sufficient here because translation is known, the vehicle translates in
the plane, and the route contains substantial yaw. A stationary sequence, a
near-zero baseline, failed ICP overlap, or motion along only one unchanging
direction should be treated as unobservable; the tool reports this instead of
publishing an update. Absolute accuracy remains limited by raw GNSS/IMU
quality, GNSS/IMU mounting assumptions, timestamp alignment, and scan motion
distortion. When the dual-antenna heading baseline's yaw relative to vehicle x
is surveyed, pass it with `--gnss-heading-to-vehicle-yaw-deg`; otherwise the
fallback assumes negligible sideslip on selected straight segments.

## Joint MLCC-style backend prototype

The runner includes an opt-in, batch joint pose/extrinsic backend based on the
adaptive planar-voxel objective described in MLCC. It keeps the native
per-LiDAR keyframe clouds, refines reference-LiDAR poses and all constant
relative extrinsics, validates updates on held-out voxels, and rebuilds the
entire historical map from native points after an accepted update.

Set `joint_backend.optimize_extrinsic_translation: false` when translations
are surveyed. Ceres then fixes every translation parameter block and the
observability check uses only the three-dimensional rotation Hessian. The
prior-free calibrator writes this setting automatically.

Run mapping with trusted/precise extrinsics and allow a bounded consistency
refinement:

```bash
rosrun mloam mloam_offline_runner \
  --manifest data/aiv5_rotation_prior_free/corrected_manifest.yaml \
  --scenario precise \
  --backend-mode precise_refine
```

Run the coarse workflow. The first 600 synchronized frames use only
`lidar_fll_at128` for frontend motion while retaining all six native clouds.
If the backend accepts the calibration, the runner automatically restarts the
full sequence with all six LiDARs and fixed accepted extrinsics:

```bash
rosrun mloam mloam_offline_runner \
  --manifest estimator/config/offline/aiv5_sequence.yaml \
  --scenario calibration_sweep \
  --rotation-perturbations 3,5 \
  --translation-perturbation 0.3 \
  --backend-mode coarse_bootstrap
```

The backend never applies an update unless the mixed planar-voxel graph is
connected, every auxiliary LiDAR has sufficient overlap, the held-out
objective improves, the extrinsic Hessian is observable, and pose/extrinsic
changes remain inside the configured bounds. Missing priors are intentionally
unsupported by MLCC itself; use the prior-free calibrator to enter its local
capture basin.

Additional artifacts are:

- `trajectory_corrected.csv`
- `backend_extrinsics_history.csv`
- `backend_windows.csv`
- `backend_diagnostics.yaml`
- `map_backend_corrected_rgb.pcd`

In `coarse_bootstrap`, initialization artifacts are stored in the
`bootstrap/` subdirectory and the replayed full-sensor result is stored in the
scenario directory. This prototype pauses for batch optimization and replay;
it does not mutate M-LOAM's live marginalized window.

`coarse_periodic` is the experimental retroactive alternative: it first lets
all LiDARs run with the coarse prior, optimizes the retained history, and then
replays the sequence with the accepted fixed extrinsics. It is an offline
feedback cycle, not a real-time optimizer.

This guide covers dataset preparation, manifest configuration, benchmark runs,
and the generated results. The six-LiDAR template is
[`../config/offline/example_six_lidar.yaml`](../config/offline/example_six_lidar.yaml).
The checked AIV5 sequence configuration is
[`../config/offline/aiv5_sequence.yaml`](../config/offline/aiv5_sequence.yaml).

## 1. Build and set up the environment

From the repository root, build the required packages in a ROS Noetic shell:

```bash
source /opt/ros/noetic/setup.bash
colcon build --packages-select mloam_common mloam_pcl mloam \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source build/mloam/devel/setup.bash
```

Check that the runner is available:

```bash
rosrun mloam mloam_offline_runner --help
```

Run all commands below from the repository root. In every new terminal, source
both `/opt/ros/noetic/setup.bash` and `build/mloam/devel/setup.bash` first. This
package currently exposes the runner from its catkin devel space rather than
installing it into `install/`. The equivalent direct executable is
`build/mloam/devel/lib/mloam/mloam_offline_runner`. If the runner reports a
missing ROS library such as `libxmlrpcpp.so`, the ROS environment has not been
sourced.

## 2. Prepare the sequence dataset

Create one directory per LiDAR under a common dataset root:

```text
/data/my_sequence/
├── top/
│   ├── 1700000000.000000.pcd
│   ├── 1700000000.100000.pcd
│   └── ...
├── front/
│   ├── 1700000000.002000.pcd
│   └── ...
└── rear/
    └── ...
```

The filename stem must be a finite numeric scan timestamp. Files whose stems
are not numeric are ignored. Each LiDAR is indexed independently; for every
selected reference scan, the runner chooses the nearest unused scan from each
other enabled LiDAR within `synchronization_threshold`.

Each PCD must:

- Be organized with `WIDTH == expected_width` and `HEIGHT == rings`.
- Contain scalar fields named `x`, `y`, `z`, `ring`, and `timestamp`, plus
  either `reflectivity` or its accepted alias `intensity`.
- Store `ring` as an unsigned integer in the range `[0, rings)`.
- Preserve the sensor's native organized column order. M-LOAM uses the supplied
  ring and point time; it does not derive them from angles.
- Have at least `minimum_finite_ratio` finite XYZ points.
- Have point timestamps which, after multiplication by `timestamp_scale`, lie
  in `[0, scan_period]` seconds. Values no more than 2 ms past the configured
  period are clamped to the period to tolerate sensor quantization.

For sensors whose point timestamp unit is 2 microseconds, use
`timestamp_scale: 2.0e-6`. The runner supports mixed geometries, such as a
128-ring 360-degree LiDAR together with 32-ring limited-FOV LiDARs.

An invalid cloud or an incomplete synchronized set is recorded in
`synchronization.csv`, dropped as a complete multi-LiDAR frame, and does not
stop the sequence.

## 3. Create a manifest

Copy the example and edit it for the sequence:

```bash
cp estimator/config/offline/example_six_lidar.yaml \
  estimator/config/offline/my_sequence.yaml
```

A minimal two-LiDAR manifest looks like this:

```yaml
dataset_root: /data/my_sequence
output_root: /data/mloam_results/my_sequence
mloam_config: estimator/config/config_realvehicle_mltest.yaml
reference_lidar: top
synchronization_threshold: 0.06
frame_begin: 0
frame_end: 760
frame_stride: 1
minimum_finite_ratio: 0.95

lidars:
  - name: top
    enabled: true
    directory: top
    rings: 128
    horizontal_fov_deg: 360
    expected_width: 1800
    timestamp_scale: 2.0e-6
    scan_period: 0.1
    time_offset: 0.0
    segment_cloud: true
    min_cluster_size: 30
    segment_valid_point_num: 5
    segment_valid_line_num: 3
    segment_theta: 1.047
    color: [230, 25, 75]
    vehicle_T_lidar:
      translation: [0.0, 0.0, 0.0]
      rpy_deg: [0.0, 0.0, 0.0]

  - name: front
    enabled: true
    directory: front
    rings: 32
    horizontal_fov_deg: 120
    expected_width: 1800
    timestamp_scale: 2.0e-6
    scan_period: 0.1
    time_offset: 0.0
    segment_cloud: true
    min_cluster_size: 30
    segment_valid_point_num: 5
    segment_valid_line_num: 3
    segment_theta: 1.047
    color: [60, 180, 75]
    vehicle_T_lidar:
      translation: [1.0, 0.0, 0.0]
      rpy_deg: [0.0, 0.0, 0.0]
```

Important manifest rules:

- `dataset_root` and `output_root` should be absolute paths. A LiDAR's
  `directory` is joined to `dataset_root`.
- `frame_begin` is inclusive and `frame_end` is exclusive in the sorted
  reference-LiDAR index. `frame_stride` must be positive.
- LiDAR names must be unique. The reference LiDAR must be enabled and included
  in the run.
- `time_offset` is a fixed offset in seconds. Synchronization compares
  `filename_timestamp + time_offset`; both raw and corrected skew are logged.
- `scan_period` must agree with `SCAN_PERIOD` in the legacy `mloam_config`.
- `color` is the stable RGB identity used in merged maps.
- Set `segment_cloud: false` to bypass range-image segmentation for a LiDAR.
  When enabled, the four segmentation parameters are applied to that LiDAR's
  native ring/column grid.
- Under `joint_backend`, set `optimize_extrinsic_translation: false` whenever
  vehicle-frame translations are surveyed. The default remains `true` for
  compatibility with earlier full-extrinsic experiments.

### Extrinsic convention

The M-LOAM runner requires a complete calibrated `vehicle_T_lidar`. Inline
`rpy_deg` is `[roll, pitch, yaw]` in degrees and is composed in Z-Y-X order.
The runner converts the vehicle-frame calibrations to `reference_T_lidar`
internally. The separate prior-free calibrator is the exception: before it
writes the complete corrected manifest, its input may contain only the
vehicle-frame `translation`.

Instead of an inline transform, a LiDAR may point to the existing calibration
prototxt format:

```yaml
    extrinsic_prototxt: ../../calibration/front_lidar.prototxt
```

Relative `extrinsic_prototxt` paths are resolved relative to the manifest file.
The prototxt translation and rotation are also interpreted as
`vehicle_T_lidar`. Both the legacy `rotation { roll, pitch, yaw }` form and the
AIV `rotation_rpy_deg { x, y, z }` form are supported.

## 4. Run the sequence

The first run should be a short fixed-extrinsic smoke test:

```bash
rosrun mloam mloam_offline_runner \
  --manifest estimator/config/offline/my_sequence.yaml \
  --scenario precise \
  --begin 0 --end 20 \
  --output-root /data/mloam_results/my_sequence_smoke
```

Inspect `precise/synchronization.csv`, `features.csv`, `runtime.csv`, and
`map_final_rgb.pcd` before starting a full run.

Run the full precise baseline:

```bash
rosrun mloam mloam_offline_runner \
  --manifest estimator/config/offline/my_sequence.yaml \
  --scenario precise
```

To trust the calibration folder as the initial estimate and then optimize the
non-reference extrinsics online, run:

```bash
rosrun mloam mloam_offline_runner \
  --manifest estimator/config/offline/my_sequence.yaml \
  --scenario calibrated_init
```

To measure angular recovery around the trusted calibration without coupling in
translation error, run a deterministic rotation-only sweep:

```bash
rosrun mloam mloam_offline_runner \
  --manifest estimator/config/offline/aiv5_sequence.yaml \
  --scenario calibration_sweep \
  --rotation-perturbations 1,3,5,10 --seed 42
```

Every non-reference LiDAR receives its own deterministic random rotation axis;
the axis stays fixed across sweep levels so increasing angles are directly
comparable. Translation remains unchanged unless
`--translation-perturbation` is supplied. Outputs use directories such as
`calibrated_init_3deg_0m/` and include the injected transform, extrinsic
history, optimized extrinsics, odometry, and local/final maps.

Run all deterministic coarse-initialization levels:

```bash
rosrun mloam mloam_offline_runner \
  --manifest estimator/config/offline/my_sequence.yaml \
  --scenario coarse --coarse-level all --seed 42
```

The levels are:

| Level | Rotation perturbation | Translation perturbation | Output directory |
|---:|---:|---:|---|
| `0` | 5 degrees | 0.25 m | `coarse_5deg_0.25m/` |
| `1` | 15 degrees | 0.75 m | `coarse_15deg_0.75m/` |
| `2` | 30 degrees | 1.5 m | `coarse_30deg_1.5m/` |

The seed and LiDAR name determine stable rotation and translation directions.
The exact injected transforms are saved in `resolved_config.yaml`.

Run without an extrinsic prior for non-reference LiDARs:

```bash
rosrun mloam mloam_offline_runner \
  --manifest estimator/config/offline/my_sequence.yaml \
  --scenario prior_free --seed 42
```

Or run precise, calibrated-init, all three coarse, and prior-free scenarios
together:

```bash
rosrun mloam mloam_offline_runner \
  --manifest estimator/config/offline/my_sequence.yaml \
  --scenario all --coarse-level all --seed 42
```

Useful command-line overrides are:

```text
--mloam-config FILE       override the legacy estimator configuration
--include top,front       run only the named LiDARs
--exclude rear            disable the named LiDARs
--reference NAME          override the reference LiDAR
--begin N --end N         override the inclusive/exclusive reference range
--stride N                process every Nth reference scan
--seed N                  choose deterministic coarse perturbations
--output-root DIR         override the result root
--ros-publish             publish the optional ROS/RViz visualization
```

Do not specify the same LiDAR in both `--include` and `--exclude`. If selection
removes the configured reference, also supply an included `--reference`.

## 5. Find and interpret the results

Results are written below the manifest's `output_root`, or the path supplied by
`--output-root`:

```text
<output_root>/
├── precise/
├── calibrated_init/
├── coarse_5deg_0.25m/
├── coarse_15deg_0.75m/
├── coarse_30deg_1.5m/
└── prior_free/
```

Each scenario directory contains:

| Artifact | Purpose |
|---|---|
| `resolved_config.yaml` | Complete resolved selection, scenario, seed, precise and initial transforms, and injected perturbations. |
| `summary.yaml` | Run status, frame counts, final calibration errors and states, convergence frame/time, and alignment median/P95. |
| `synchronization.csv` | One record per attempted set, including drop reason and maximum raw/corrected skew. |
| `runtime.csv` | Per-frame preprocessing, odometry, and mapping time in milliseconds. |
| `features.csv` | Per-frame, per-LiDAR corner and surface feature counts. |
| `trajectory.csv` | Estimated world pose of the reference LiDAR as translation plus XYZW quaternion. |
| `optimized_extrinsics.yaml` | Final transforms in both `reference_T_lidar` and `vehicle_T_lidar` convention. |
| `extrinsics_history.csv` | Per-frame `reference_T_lidar`, rotation/translation error, and calibration state. |
| `observability_history.csv` | Per-frame observability flag and calibration state for each non-reference LiDAR. |
| `map_online_rgb.pcd` | Map accumulated with the extrinsics available online at each keyframe. |
| `map_online_features_rgb.pcd` | Online feature map used by the mapper. |
| `map_merged_rgb.pcd` | Merged online RGB map; currently equivalent to `map_online_rgb.pcd`. |
| `map_final_rgb.pcd` | Preferred comparison map, rebuilt from saved keyframes using final extrinsics. |
| `map_<lidar-name>.pcd` | Final-map points belonging to one LiDAR. |
| `keyframes/<lidar-name>/<frame>.pcd` | Binary-compressed, 0.05 m voxel-filtered per-sensor keyframes. |

RGB colors always identify the configured source LiDAR. `map_final_rgb.pcd` is
the best structural-alignment artifact because early calibration error does not
remain baked into it. Compare it with `map_online_rgb.pcd` to see the effect of
online calibration history.

The process exit status is:

- `0`: every requested scenario succeeded or converged.
- `2`: at least one scenario did not converge, but valid artifacts were kept.
- `1`: configuration or execution failure.

The following evaluator is a legacy, optional odometry benchmark only. If an
AIV5 session contains a LiDAR-localizer `vehicle_pose` stream, it can generate
ATE and RPE against that derived reference:

```bash
python3 estimator/offline/tools/evaluate_aiv5_session.py \
  --manifest estimator/config/offline/aiv5_sequence.yaml \
  --session-root /data/mloam_results/my_session \
  --scenarios precise calibrated_init prior_free
```

This writes `odometry_metrics.yaml` and
`trajectory_vehicle_aligned.csv` in each scenario plus
`session_metrics.yaml` and `session_report.md` at the session root. The
evaluator applies a rigid SE(3), no-scale alignment and labels the supplied DR
vehicle pose as a reference rather than surveyed ground truth. That reference
is never an input to the raw-IMU/GNSS calibration workflow above and must not
be used as independent calibration evidence.

## 6. Benchmark scenarios consistently

For a fair benchmark, keep the following identical across precise,
calibrated-init, coarse, and prior-free runs: manifest, included LiDARs,
reference LiDAR, frame range, stride, synchronization threshold, fixed time
offsets, algorithm config, and seed. Use a separate output root if an earlier
run must be preserved.

Recommended comparison order:

1. Verify comparable input in `resolved_config.yaml` and
   `synchronization.csv`. Compare processed/dropped sets and corrected skew.
2. Use `summary.yaml` for final per-LiDAR rotation/translation error,
   convergence frame/time, and cross-LiDAR alignment median/P95. Lower
   alignment distances indicate better agreement in overlapping voxels.
3. Inspect `extrinsics_history.csv` and `observability_history.csv` for
   initialization, observability, convergence, non-convergence, or failure.
4. Compute median and P95 stage time from `runtime.csv`, and compare feature
   counts from `features.csv`.
5. Visually inspect `map_final_rgb.pcd`; differently colored surfaces should
   overlap. Use the per-LiDAR maps to isolate a sensor that disagrees.

The `trajectory.csv` output can be aligned to an external vehicle-pose source
for an optional diagnostic. Vehicle-pose ATE/RPE is not used as a calibration
pass criterion because the benchmark's primary target is inter-LiDAR
extrinsic and structural alignment.

## 7. Troubleshooting

**A LiDAR directory cannot be opened or has no usable scans**

Check `dataset_root`, each `directory`, filename extensions, and numeric
filename stems. Only lowercase `.pcd` files with fully numeric stems are
indexed.

**Cloud validation reports missing fields, dimensions, ring, or timestamps**

Compare the PCD header with the manifest. Confirm the exact field names and
scalar types, organized width/height, unsigned ring values, and
`timestamp * timestamp_scale <= scan_period`. Do not substitute `intensity`
for `reflectivity`.

**Many synchronized sets are dropped**

Open `synchronization.csv` and group by `status` and `reason`. Compare
`max_raw_skew_s` with `max_corrected_skew_s`; if the corrected skew is still
larger than the threshold, verify timestamp units and the sign/value of each
`time_offset`. A scan selected for an attempted set is never reused by a later
set, even when the attempted set is incomplete.

**Features are empty or strongly imbalanced between LiDARs**

Inspect `features.csv`, verify `rings`, `expected_width`, FOV, and scan period,
then tune the per-LiDAR segmentation parameters. A short precise smoke test is
faster than diagnosing these settings in a complete run.

**The run exits with status 2**

This is a completed non-converged benchmark, not an artifact-write failure.
Keep the output and inspect `summary.yaml`, `extrinsics_history.csv`, and
`observability_history.csv`. Poor scene overlap, weak motion excitation, too
few valid synchronized frames, or a large initial perturbation can prevent
convergence.

**The output directory already contains results**

Choose a new `--output-root` before rerunning if the existing benchmark must be
retained. Scenario artifacts are written to deterministic directory names.
