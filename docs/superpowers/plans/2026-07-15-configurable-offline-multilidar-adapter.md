# Configurable Offline Multi-LiDAR Adapter Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a deterministic, streaming, ROS-free multi-LiDAR PCD runner that preserves native ring/time data, drives reusable estimator/mapping adapters, and emits reproducible calibration benchmark artifacts.

**Architecture:** A standalone `mloam_offline` library owns manifest parsing, independent scan indexes, synchronization, native PCD validation/preparation, scenario transforms, status tracking, map reconstruction, and reports. A small CLI streams synchronized frames through a callback-based pipeline; the legacy estimator receives already prepared `PointICloud` frames through a new overload, while its ROS callbacks remain unchanged. Mapping state is represented by an in-process `LidarMapper` core with the legacy executable retaining ROS transport at its boundary.

**Tech Stack:** C++14, Eigen3, PCL, yaml-cpp, CMake/CTest, catkin/ROS Noetic compatibility.

---

### Task 1: Repair legacy package installation baseline

**Files:**
- Modify: `mloam_common/libs/CMakeLists.txt`
- Modify: `mloam_pcl/CMakeLists.txt`

- [ ] Add install rules for each package's public headers and library.
- [ ] Re-run the selected colcon build and confirm dependent package configuration passes the missing-include failure point.

### Task 2: Define and validate offline configuration

**Files:**
- Create: `estimator/offline/include/mloam/offline/types.hpp`
- Create: `estimator/offline/include/mloam/offline/manifest.hpp`
- Create: `estimator/offline/src/manifest.cpp`
- Create: `estimator/offline/test/test_manifest.cpp`
- Create: `estimator/config/offline/example_six_lidar.yaml`

- [ ] Write tests for defaults, duplicate names, reference membership, include/exclude overrides, inline/prototxt Z-Y-X transforms, and reference-relative transforms.
- [ ] Run the focused tests and observe the missing API failures.
- [ ] Implement yaml-cpp parsing and validation, then rerun tests to green.

### Task 3: Stream and synchronize independent scan indexes

**Files:**
- Create: `estimator/offline/include/mloam/offline/sequence_reader.hpp`
- Create: `estimator/offline/src/sequence_reader.cpp`
- Create: `estimator/offline/test/test_sequence_reader.cpp`

- [ ] Test numeric timestamp indexing, nearest unused matching, threshold rejection, fixed time offsets, range/stride, and incomplete frame reporting.
- [ ] Implement per-sensor indexes and a cursor-based streaming matcher without cloud preloading.
- [ ] Verify all synchronization tests pass.

### Task 4: Preserve native PCD geometry and time

**Files:**
- Create: `estimator/offline/include/mloam/offline/pcd_preprocessor.hpp`
- Create: `estimator/offline/src/pcd_preprocessor.cpp`
- Create: `estimator/offline/test/test_pcd_preprocessor.cpp`

- [ ] Generate organized native PCD fixtures and test required fields/types, dimensions, finite filtering, finite ratio, ring bounds, reflectivity preservation, 2 microsecond conversion, column order, and mixed 128/32-ring geometry.
- [ ] Implement header inspection and point preparation using native ring and timestamp values only.
- [ ] Verify focused PCD tests pass.

### Task 5: Add deterministic scenarios, reusable pipeline, mapper, and artifacts

**Files:**
- Create: `estimator/offline/include/mloam/offline/scenario.hpp`
- Create: `estimator/offline/src/scenario.cpp`
- Create: `estimator/offline/include/mloam/offline/lidar_mapper.hpp`
- Create: `estimator/offline/src/lidar_mapper.cpp`
- Create: `estimator/offline/include/mloam/offline/artifacts.hpp`
- Create: `estimator/offline/src/artifacts.cpp`
- Create: `estimator/offline/test/test_scenario_mapper.cpp`

- [ ] Test deterministic name/seed perturbations, precise/coarse/prior-free modes, sensor provenance/color stability, calibration state transitions, final-extrinsic reconstruction, and result schemas.
- [ ] Implement scenario generation, callback pipeline state, per-sensor keyframe storage, voxelized RGB rebuilding, diagnostics, and CSV/YAML writers.
- [ ] Verify focused tests pass.

### Task 6: Add CLI and M-LOAM adapters

**Files:**
- Create: `estimator/offline/src/offline_runner.cpp`
- Modify: `estimator/src/estimator/estimator.h`
- Modify: `estimator/src/estimator/estimator.cpp`
- Modify: `estimator/src/lidarMapper/lidar_mapper.h`
- Modify: `estimator/src/lidarMapper/lidar_mapper_keyframe.cpp`
- Modify: `estimator/CMakeLists.txt`

- [ ] Add a prepared-frame estimator overload that queues features without recalculating ring/time.
- [ ] Expose `LidarMapper::processFrame()` and keep ROS callbacks as a thin wrapper.
- [ ] Implement CLI overrides for scenario, include/exclude, reference, frame range, stride, seed, output, and ROS publishing.
- [ ] Build both the standalone runner and legacy ROS executables.

### Task 7: Generated integration fixture and acceptance verification

**Files:**
- Create: `estimator/offline/test/test_offline_integration.cpp`
- Create: `estimator/offline/README.md`

- [ ] Generate two-LiDAR sequences containing matched scans, a missing scan, one corrupt scan, and native ring/time fields.
- [ ] Run precise, all coarse levels, and prior-free; verify drop-and-continue and distinct non-convergence status.
- [ ] Assert resolved config, summaries, synchronization/runtime/feature/trajectory CSVs, histories, per-sensor maps, RGB map, keyframes, final rebuild, calibration errors, convergence, and overlap diagnostics exist.
- [ ] Run CTest, a clean standalone build, and selected colcon legacy build; inspect the complete diff against this checklist.
