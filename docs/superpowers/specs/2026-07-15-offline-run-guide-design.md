# Offline Multi-LiDAR Run Guide Design

## Goal

Expand `estimator/offline/README.md` into the authoritative operator guide for
running M-LOAM on sequence datasets and locating comparable benchmark results.

## Documentation Structure

The guide will contain:

1. Input PCD and directory requirements, including numeric scan timestamps,
   organized dimensions, required fields, ring bounds, and point-time units.
2. A manifest walkthrough explaining dataset/output roots, reference LiDAR,
   synchronization, frame selection, per-LiDAR geometry, segmentation,
   time offsets, colors, and extrinsics.
3. Exact ROS Noetic/colcon build and environment setup commands.
4. A short smoke-test command followed by precise, coarse, prior-free, and
   all-scenario commands, including useful CLI overrides and exit statuses.
5. The output directory naming convention and a file-by-file artifact table.
6. A benchmark workflow explaining which runs should share synchronized frame
   selection and which metrics should be compared.
7. Interpretation guidance for calibration error, convergence,
   observability, synchronization drops, runtime, feature counts, and
   cross-LiDAR structural alignment. Vehicle-pose ATE/RPE will be described
   only as an optional diagnostic, not a pass criterion.
8. Troubleshooting for invalid PCDs, timestamp/scan-period mismatches,
   incomplete synchronized sets, and non-converged status.

## Scope

The existing example manifest remains the concrete starting point. No runner
behavior or result schema will change as part of this documentation update.
Commands will use paths relative to the repository and will distinguish build
artifacts from per-scenario benchmark output.

## Validation

Before publishing, verify every documented CLI option against
`mloam_offline_runner --help`, confirm artifact names against the writer, run
Markdown/diff hygiene checks, and retain the already-passing standalone and
ROS-linked build evidence for the implementation commit.
