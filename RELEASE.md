# Release Notes

## v0.2.8

- Fix obstacle avoidance to use swerve-and-return behavior
- Reduce avoidance window from 15m to 5m so robot returns to center after passing obstacle
- Increase lane-centering cost (deviation penalty 0.5 → 2.0) for minimum-offset swerves
- AEB now triggers when ALL d-candidates are blocked (not just center lane)
- Fix visual test scenarios: staggered wall for AEB, wider obstacle spacing for lane gap

## v0.2.7

- Add `python/test_obstacle_avoidance_visual.py`: standalone 5-scenario obstacle avoidance visual test
- Runs entirely without DORA daemon or MuJoCo; opens Rerun viewer directly
- Inlines all core math from node files: `compute_cumulative_s`, `get_frenet`, `get_xy_from_frenet`, `plan_path`, `plan_path_with_d`, Phase 1/2 avoidance logic, `Costmap2D`
- Generates synthetic pointclouds (16B header + Nx16B float32 points) per obstacle bounding box
- Inline obstacle detection pipeline: ground filter, range filter, KDTree Euclidean clustering, Frenet coords
- 5 test scenarios: baseline driving, AEB stop, lateral avoidance, two-obstacle lane selection, full blockage with recovery state machine (NORMAL → WAITING → BACKING_UP → STOPPED)
- Rerun timeline uses `rr.set_time_sequence("step", ...)` for full scrubber support
- Robust waypoints loading: tries multiple candidate paths; falls back to synthetic 80m oval if not found
- Compatible with Python 3.8+ via `from __future__ import annotations`


## v0.2.6

- Wire all obstacle avoidance nodes into `dataflow_python.yaml`
- Add `obstacle-detector`, `costmap`, and `recovery-monitor` node definitions
- Connect `obstacle_list`, `costmap_grid`, `recovery_cmd` inputs to `planning` node
- Connect `obstacle_list`, `costmap_grid` inputs to `rerun` visualization node
- Add `scipy` to prerequisites

## v0.2.5

- Extend `planning_node.py` with three-phase obstacle avoidance
- Phase 3 (recovery override): immediately STOP or BACK when `recovery_cmd` signals WAITING, BACKING_UP, or STOPPED states, bypassing all other planning
- Phase 1 (AEB): scan `obstacle_list` for obstacles within 5m ahead and inside road half-width; emit type=3 emergency brake request when closest distance < 3m
- Phase 2 (lateral avoidance): cost-minimisation over 9 d-offset candidates [-2, -1.5 … +2m]; costs combine obstacle proximity (1/lateral_dist), lane deviation (0.5×|d|), and smoothness (0.3×|Δd|); replan via `plan_path_with_d()` when best_d ≠ 0
- Add `plan_path_with_d()`: CubicSpline d-profile blending cur_d → target_d over ~3m using `scipy.interpolate.CubicSpline` with clamped boundary conditions
- Add handlers for `obstacle_list`, `costmap_grid`, and `recovery_cmd` inputs
- Update module docstring with all new input/output formats and request type codes

## v0.2.4

- Add `obstacle_list` handler to `rerun_viz_node.py`: renders each tracked obstacle as a red 3D box (`rr.Boxes3D`) with label showing ID and distance
- Add `costmap_grid` handler to `rerun_viz_node.py`: renders costmap as a flat point cloud at z=-0.05m; red (alpha 180) for fully-occupied cells (cost >= 254), yellow (alpha 100) for inflated cells
- Both handlers emit `rr.Clear(recursive=True)` when input is empty, keeping the viewer clean

## v0.2.3

- Add `costmap_node.py`: Phase 2 rolling-window 2D occupancy costmap DORA node
- Robot-centred grid with configurable size (COSTMAP_SIZE_M, default 20m), resolution (COSTMAP_RESOLUTION, default 0.1m) giving a 200x200 = 40K-cell grid
- Precomputed inflation kernel with linear cost decay 253→1 over INFLATION_RADIUS (default 0.5m)
- Vectorised numpy pipeline: pointcloud parse → ground removal (z < 0.15m) → global-frame transform → distance filter → grid update → inflation
- Serialises to 28-byte header + row-major uint8 grid data on `costmap_grid` output
- All parameters configurable via environment variables

## v0.2.2

- Add `obstacle_detector_node.py`: Phase 1 lidar-based obstacle detection DORA node
- Parses 16-byte-header pointcloud frames into float32 x,y,z,intensity points
- Ground removal (GROUND_THRESHOLD env var, default 0.15m), distance filter (MAX_DETECT_RANGE, default 15.0m)
- Euclidean clustering via scipy KDTree (CLUSTER_TOLERANCE=0.5m, MIN_CLUSTER_SIZE=3)
- Computes centroid, axis-aligned bounding box, and Frenet (s,d) coords per cluster
- Outputs packed `obstacle_list`: 4-byte count header + 44-byte obstacle structs
- All thresholds configurable via environment variables

## v0.2.1

- Add `recovery_monitor_node.py`: Phase 3 recovery behavior DORA node
- Implements 4-state machine: NORMAL → WAITING → BACKING_UP → STOPPED
- Stuck detection via 3s pose displacement window (<0.1m threshold)
- Reverse recovery at -200.0 speed for 1m, up to 3 retries before STOPPED
- Auto-reset after 30s cooldown or manual forward command
- Outputs 5-byte `recovery_cmd` (uint8 state + float32 backup_speed) on every pose update

## v0.2.0

- Add Mujoco simulation bridge for mapping and localization testing
- Implement LiDAR simulator with raycasting (configurable rays, range, noise)
- Implement IMU simulator with body dynamics and noise models
- Add sample robot model (differential drive) and warehouse world
- Add dataflows for full simulation and mapping modes

## v0.1.0

- Add CLAUDE.md to .gitignore for Claude Code configuration
