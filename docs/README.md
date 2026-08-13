# Documentation

The repository keeps reusable algorithm descriptions separate from
dataset-specific evidence and command-oriented run guides.

## Methodology

- [Raw-IMU/GNSS fixed-translation multi-LiDAR rotation calibration](methodology/fixed_translation_planar_multi_lidar_calibration.md): canonical data boundary, problem formulation, frame conventions, fusion, optimization, validation gates, mapping, and limitations.
- [Mathematical establishment](methodology/fixed_translation_planar_multi_lidar_calibration_math.md): WGS84/ENU construction, IMU-heading fusion, antenna-motion conjugation, expanded hand-eye residuals, Jacobians, Hessians, and planar observability.

## Offline operation

- [Multi-LiDAR PCD runner](../estimator/offline/README.md): build, manifest, execution, and artifact instructions.

## Evaluation reports

- [AIV5 rotation-prior-free feasibility study](aiv5_rotation_prior_free_feasibility_report.md): observability analysis, no-prior initialization strategy, raw IMU/GNSS/wheel fusion assessment, dataset excitation, and proposed validation protocol.
- [AIV5 raw-IMU/GNSS 10-degree recovery](aiv5_raw_imu_gnss_10deg_results.md): current successful six-LiDAR calibration and ENU mapping experiment.
- [Superseded AIV5 navigation-aided report](aiv5_planar_nav_10deg_results.md): withdrawal notice for the earlier leakage-prone experiment.
- [AIV5 legacy calibration rotation sweep](aiv5_calibration_rotation_sweep.md): behavior of the original calibration path under injected rotation errors.

The files under `docs/superpowers/` are historical implementation plans and
specifications. They are not the current user or algorithm documentation.
