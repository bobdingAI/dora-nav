# Release Notes

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
