# Python dora-nav Nodes

Pure Python reimplementation of the 5 core dora-nav C++ nodes. Runs on macOS and Linux without requiring CMake, GCC, or the dora C API static library.

## Nodes

| Python Node | Replaces C++ | Algorithm |
|-------------|-------------|-----------|
| `pub_road_node.py` | `map/pub_road/pubroad` | Reads Waypoints.txt, publishes road_lane as float32 array |
| `road_lane_publisher_node.py` | `map/road_line_publisher/road_lane_publisher_node` | Frenet coordinate conversion (Cartesian → s,d) |
| `planning_node.py` | `planning/routing_planning/routing_planning_node` | Frenet path planner, 30-point local path in vehicle frame |
| `lon_controller_node.py` | `control/vehicle_control/lon_controller/lon_controller_node` | Longitudinal control (speed → torque/brake) |
| `lat_controller_node.py` | `control/vehicle_control/lat_controller/lat_controller_node` | Pure Pursuit lateral control (path → steering angle) |

Plus utility nodes:
- `pose_extractor_node.py` — extracts Pose2D_h from MuJoCo qpos (for macOS dora-mujoco compatibility)
- `task_pub_stub.py` — publishes static road attributes (replaces PostgreSQL-dependent C++ task_pub_node)
- `rerun_viz_node.py` — Rerun 3D visualization (robot body, path, pointcloud, trail)

## Wire Compatibility

All binary formats match the C++ originals:

| Message | Format | Size |
|---------|--------|------|
| `road_lane` | `[x0..xN, y0..yN]` float32 | N*8 bytes |
| `Pose2D_h` (cur_pose) | `[x, y, theta_deg]` float32 | 12 bytes |
| `CurPose_h` (cur_pose_all) | `[x, y, theta_rad, s, d]` float64 | 40 bytes |
| `RoadAttri_h` (road_attri_msg) | 10 x float32 | 40 bytes |
| `raw_path` | `[x0..x29, y0..y29]` float32 | 240 bytes |
| `Request_h` | `[type(u8), speed(f32), stop_dist(f32), aeb_dist(f32)]` | 13 bytes |
| `SteeringCmd_h` | `[angle_deg]` float32 | 4 bytes |
| `TrqBreCmd_h` | 24-byte struct (see lon_controller_node.py) | 24 bytes |

You can mix Python and C++ nodes in the same dataflow — they speak the same binary protocol.

## Quick Start

```bash
pip install dora-rs pyarrow numpy rerun-sdk

cd python
dora up && dora start dataflow_python.yaml --attach
```

## Dependencies

- `dora-rs` — dataflow runtime
- `pyarrow` — Apache Arrow arrays for dora message passing
- `numpy` — array operations
- `rerun-sdk` — 3D visualization (optional, for rerun node)
