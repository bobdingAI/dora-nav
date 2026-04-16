# dora-nav Test Commands

## Python Obstacle Avoidance

```bash
# Standalone visual demo — 5 avoidance scenarios in Rerun
python3 python/test_obstacle_avoidance_visual.py

# Real PCD obstacle avoidance test in Rerun
python3 python/test_real_pcd_avoidance.py
```

## Vehicle / Hardware Drivers

```bash
# Head servo control test
python3 vehicle/adora_head_control_dora_node/test_head_control.py

# Lifting motor serial test
python3 vehicle/adora_lifting_motor_control_dora_node/dora_test_SerialLiftingMotor.py

# Suction pump test
python3 vehicle/adora_suction_pump_control_dora_node/dora_test.py
```

## GNSS / Localization

```bash
# GNSS test runner
bash driver/gnss_poser/test/test.sh

# GNSS test dataflow
dora up && dora start driver/gnss_poser/test/testdata.yml

# Control loop test
bash driver/gnss_poser/controltest.sh

# Control loop test dataflow
dora up && dora start driver/gnss_poser/controltest.yml

# Map test
bash driver/gnss_poser/maptest.sh

# Map test dataflow
dora up && dora start driver/gnss_poser/maptest.yml

# Planning test dataflow
dora up && dora start driver/gnss_poser/planningtest.yml

# GPS conversion test
python3 driver/gnss_poser/src/gps_to_ros2_test.py
```

## Perception

```bash
# Crosswalk / traffic light detection test
python3 peception/crosswalk-traffic-light-detection-yolov5/test.py
```

## DORA Dataflows

```bash
# Task publisher test
dora up && dora start planning/mission_planning/task_pub/test.yml

# Localization-only test
dora up && dora start load_path.yml

# Full navigation system
dora up && dora start run.yml

# Stop all dataflows
dora stop
```

## Rerun Visualization

```bash
# Launch Rerun viewer (receives data from rerun node)
rerun
```
