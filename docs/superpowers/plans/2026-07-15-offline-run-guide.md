# Offline Multi-LiDAR Run Guide Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn the offline runner README into an exact guide for preparing sequence datasets, running calibration scenarios, locating results, and comparing benchmarks.

**Architecture:** Keep one authoritative guide beside the runner. Validate commands against the compiled CLI and artifact names against the writer without changing runner behavior.

**Tech Stack:** Markdown, ROS Noetic, colcon/catkin, YAML, PCD/CSV/YAML artifacts, Git.

---

### Task 1: Expand the Operator Guide

**Files:**
- Modify: `estimator/offline/README.md`
- Reference: `estimator/config/offline/example_six_lidar.yaml`
- Reference: `estimator/offline/src/offline_runner.cpp`
- Reference: `estimator/offline/src/artifacts.cpp`

- [ ] **Step 1: Record the current contract**

Run the compiled runner with `--help` and search `artifacts.cpp` for every YAML, CSV, and PCD filename. Expected: the guide only names implemented options and artifacts.

- [ ] **Step 2: Add dataset and manifest instructions**

Document a `dataset_root/<lidar>/<numeric_timestamp>.pcd` layout, required scalar fields, organized dimensions, timestamp scaling/scan-period bounds, reference selection, synchronization, offsets, segmentation, colors, and extrinsic sources.

- [ ] **Step 3: Add build and run commands**

Include ROS Noetic and workspace setup, then exact commands for a short precise smoke test, full precise, coarse levels, prior-free, and all scenarios. Document include/exclude, reference, range, stride, seed, output root, and ROS publishing overrides.

- [ ] **Step 4: Add results and benchmark guidance**

Describe `<output_root>/<scenario-name>/`, every artifact, exit codes, and comparison workflow. Treat calibration error, convergence, observability, dropped sets, runtime, feature counts, and cross-LiDAR alignment as primary; vehicle-pose ATE/RPE remains optional.

- [ ] **Step 5: Add troubleshooting**

Cover malformed PCDs, ring/timestamp failures, incomplete synchronized sets, non-convergence, and diagnosing runs from synchronization/runtime CSVs.

### Task 2: Verify, Commit, and Push

**Files:**
- Verify: `estimator/offline/README.md`
- Commit: the complete offline-adapter implementation and documentation scope

- [ ] **Step 1: Verify**

Run:

```bash
cmake --build /tmp/mloam-offline-final-build -j2
ctest --test-dir /tmp/mloam-offline-final-build --output-on-failure
source /opt/ros/noetic/setup.bash
cmake --build build/mloam --target mloam_offline_runner lidar_mapper_keyframe -j2
git diff --check
```

Expected: six tests pass, both ROS-linked targets build, and diff hygiene passes.

- [ ] **Step 2: Commit intentionally**

Review `git status -sb` and `git diff --stat`, stage only this task's files, then commit with `feat: add configurable offline multi-lidar runner`.

- [ ] **Step 3: Push**

Run `git push -u origin mloam_gf`. Expected: the remote branch updates and tracking is configured.
