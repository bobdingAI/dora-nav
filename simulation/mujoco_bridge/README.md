# Mujoco Simulation Bridge for DORA-NAV

This module provides a Mujoco-based simulation bridge that replaces physical sensors (LiDAR, IMU) with simulated equivalents, enabling hardware-in-the-loop testing of the navigation stack.

## Prerequisites

1. **Mujoco** (>= 2.3.0)
   ```bash
   pip install mujoco
   # Or build from source: https://github.com/deepmind/mujoco
   ```

2. **DORA-RS** (>= 0.3.8)
   - Ensure `libdora_node_api_c.a` is built

3. **CMake** (>= 3.10)

## Building

```bash
cd simulation/mujoco_bridge
mkdir build && cd build
cmake ..
cmake --build .
```

## Usage

### Full Simulation Mode

Replace all sensors with simulation:

```bash
dora up
dora start simulation/mujoco_bridge/dataflow_mujoco_sim.yml
```

### Mapping Mode

Build maps in simulation:

```bash
dora start simulation/mujoco_bridge/dataflow_mapping_sim.yml
```

## Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `MUJOCO_MODEL_PATH` | `./models/robot.xml` | Robot MJCF model path |
| `SIM_TIMESTEP` | `0.001` | Physics timestep (seconds) |
| `REAL_TIME_FACTOR` | `1.0` | Simulation speed (1.0 = real-time) |
| `LIDAR_RAYS` | `1080` | Horizontal LiDAR rays |
| `LIDAR_VERTICAL_BEAMS` | `16` | Vertical beams (for RSHELIOS-16) |
| `LIDAR_RANGE` | `100.0` | Max LiDAR range (meters) |
| `LIDAR_MIN_RANGE` | `0.5` | Min LiDAR range (meters) |
| `POINTCLOUD_RATE` | `10` | Point cloud publish rate (Hz) |
| `IMU_RATE` | `20` | IMU publish rate (Hz) |
| `IMU_ADD_NOISE` | `1` | Enable IMU noise (0/1) |

## Output Topics

| Topic | Format | Size | Description |
|-------|--------|------|-------------|
| `pointcloud` | Binary | 16 + N×16 bytes | Point cloud (matches rslidar format) |
| `imu_msg` | Binary | 32 bytes | IMU data (matches HWT9053 format) |
| `ground_truth_pose` | Binary | 12 bytes | Perfect pose for debugging |

## Customizing the Robot Model

Edit `models/robot.xml` to match your robot:

1. **Chassis dimensions**: Modify `size` attribute of `chassis` geom
2. **Wheel configuration**: Adjust wheel positions and sizes
3. **LiDAR position**: Change `lidar` body position
4. **Actuators**: Configure motor parameters

## Creating Custom Worlds

Add obstacles in `models/robot.xml` or include separate world files:

```xml
<include file="worlds/warehouse.xml"/>
```

## Architecture

```
mujoco_sim_bridge
├── main.cpp              # Entry point, config loading
├── mujoco_sim_bridge.cpp # Main simulation loop
├── lidar_simulator.cpp   # Raycasting-based LiDAR
└── imu_simulator.cpp     # Body dynamics to IMU
```

## Debugging

1. **Verify point cloud format**:
   ```bash
   # Check point count matches expected
   dora logs mujoco_sim | grep "points"
   ```

2. **Compare with ground truth**:
   - Use `ground_truth_pose` output to verify localization accuracy

3. **Visualize in Rerun**:
   - The dataflow includes a `rerun` node for visualization
