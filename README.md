# Sentry Robot - ROS2 Navigation Stack

## Project Structure

### sentry_bringup
**Main launch orchestration**
- `simulated_robot.launch.py` - Simulation with Gazebo
- `real_robot.launch.py` - Real robot hardware

**Launch Arguments:**
| Argument | Default | Description |
|----------|---------|-------------|
| `slam` | `false` | Use SLAM for mapping. If false, use AMCL with pre-built map |
| `use_keyboard` | `true` | Use keyboard teleop instead of joystick |
| `use_nav` | `true` | Launch navigation stack |

### sentry_description
**Robot model and simulation**
- URDF: `sentry_description.urdf.xacro` (omnidirectional 4-wheel robot)
- Gazebo world: `comp_map.sdf` (competition arena)
- RViz config: `sentry_config.rviz`
- Plugins: Planar move (omni), dual LiDAR, IMU

### sentry_localization
**Localization stack**

**Global Localization** (controlled by `slam` argument)
- SLAM mode (`slam:=true`): Uses `slam_toolbox` for mapping
- AMCL mode (`slam:=false`): Uses `nav2_amcl` + `nav2_map_server` for localization

**Local Localization (EKF)**
- Sensor fusion: wheel odometry
- Package: `robot_localization`
- Subscribes: `/odom`
- Publishes: `/odometry/filtered`

### sentry_communication
**MCB serial communication**
- `comm_hub.py` - Bidirectional UART bridge between MCB and ROS2
  - Receives: 17-float message from MCB (wheel encoders, turret encoders, IMU)
  - Publishes:
    - `/odom` (nav_msgs/Odometry) - integrated wheel odometry
    - `/imu` (sensor_msgs/Imu) - IMU data
    - `/mcb_odom` (Float32MultiArray) - raw debug data
    - `odom → base_link` TF transform
  - Subscribes: `/cmd_vel` (Twist) and sends velocity commands to MCB
  - Protocol: DJI Serial over `/dev/ttyTHS1` at 115200 baud

### sentry_navigation
**Nav2 navigation stack**

**Planner Server**
- Algorithm: SMAC Planner 2D (A* variant)
- Motion model: MOORE (8-connected for omni)
- Global costmap with static + obstacle layers

**Controller Server**
- Algorithm: DWB (Dynamic Window Approach)
- Omnidirectional: samples vx, vy, vth
- Local costmap with obstacle + inflation layers

**Smoother Server**
- Algorithm: SimpleSmoother
- Iterations: 400 with refinement

---

## Quick Start

### Simulation
```bash
ros2 launch sentry_bringup simulated_robot.launch.py
```

### Real Robot
```bash
ros2 launch sentry_bringup real_robot.launch.py
```

---

## Mapping and Localization

The robot supports two modes controlled by the `slam` argument:

### Mode 1: SLAM Mapping (`slam:=true`)

Use this to create a new map of an environment.

```bash
# Real robot
ros2 launch sentry_bringup real_robot.launch.py slam:=true

# Simulation
ros2 launch sentry_bringup simulated_robot.launch.py slam:=true
```

Drive the robot around to map the environment. Monitor in RViz to see the map being built.

**Saving the map:**
```bash
ros2 run nav2_map_server map_saver_cli -f ~/qkrt-nav/src/sentry_localization/maps/my_map
```

### Mode 2: AMCL Localization (`slam:=false`, default)

Use a pre-built map for localization during operation.

```bash
# Use default map (sentry_map.yaml)
ros2 launch sentry_bringup real_robot.launch.py

# Specify a custom map
ros2 launch sentry_bringup real_robot.launch.py map_yaml:=/path/to/my_map.yaml
```

Set the robot's initial pose in RViz using "2D Pose Estimate".

### Typical Workflow

1. **Map the arena** (once per venue):
   ```bash
   ros2 launch sentry_bringup real_robot.launch.py slam:=true
   # Drive around, then save:
   ros2 run nav2_map_server map_saver_cli -f ~/qkrt-nav/src/sentry_localization/maps/arena_map
   ```

2. **Update config** - edit `sentry_localization/config/sentry_map.yaml`:
   ```yaml
   image: ../maps/arena_map.pgm
   resolution: 0.05
   origin: [-9.0, -6.0, 0.0]
   occupied_thresh: 0.65
   free_thresh: 0.25
   negate: 0
   ```

3. **Run with localization**:
   ```bash
   ros2 launch sentry_bringup real_robot.launch.py
   ```

### Future: No-Go Zones

The navigation stack will support a separate prebuilt occupancy grid for no-go zones (obstacles too short for LiDAR detection). The control algorithm will use this for path planning while AMCL uses the SLAM-generated map for localization.

---

## System Architecture

### Data Flow (Real Robot)

```
                                    ┌─────────────────┐
                                    │  robot_state_   │
                                    │   publisher     │
                                    │ (URDF transforms)│
                                    └────────┬────────┘
                                             │ TF: base_link → laser_frames
                                             ▼
┌──────────┐     /cmd_vel      ┌─────────────────┐      /odom        ┌─────────┐
│  Nav2    │ ─────────────────▶│    comm_hub     │ ─────────────────▶│   EKF   │
│Controller│                   │  (MCB serial)   │                   │ filter  │
└──────────┘                   └────────┬────────┘                   └─────────┘
     ▲                                  │
     │                                  │ TF: odom → base_link
     │                                  ▼
     │                         ┌─────────────────┐
     │          /scan          │  laser_merger   │◀── /scan_left, /scan_right
     └─────────────────────────│                 │
                               └─────────────────┘
                                        │
                                        ▼
                               ┌─────────────────┐
                               │ AMCL or SLAM    │
                               │ (map → odom TF) │
                               └─────────────────┘
```

### TF Tree
```
map → odom          (AMCL or SLAM toolbox)
odom → base_link    (comm_hub from wheel odometry)
base_link → laser_frame_left   (robot_state_publisher)
base_link → laser_frame_right  (robot_state_publisher)
```

### Key Topics
| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/cmd_vel` | Twist | Nav2 Controller | comm_hub |
| `/odom` | Odometry | comm_hub | EKF |
| `/imu` | Imu | comm_hub | (available) |
| `/scan` | LaserScan | laser_merger | AMCL, Nav2 |
| `/scan_left` | LaserScan | rplidar_node_1 | laser_merger |
| `/scan_right` | LaserScan | rplidar_node_2 | laser_merger |
| `/map` | OccupancyGrid | map_server | Nav2 |

---

## To Be Implemented

### Priority 1: Robot Modelling
- [ ] Create accurate URDF matching real robot dimensions
- [ ] Improve Gazebo simulation (mecanum controller)
- [ ] Update competition world with accurate obstacles

### Priority 2: Competition Logic
- [ ] Custom competition behavior tree
- [ ] Battery/health monitor node
- [ ] Competition manager node (goal selection)

### Priority 3: Testing & Calibration
- [ ] Test omnidirectional motion
- [ ] Calibrate EKF sensor fusion
- [ ] Test AMCL localization accuracy
- [ ] Fine-tune Nav2 parameters

---

## Dependencies

### Installed
- ROS2 Humble
- Nav2 stack (`nav2_amcl`, `nav2_map_server`, `nav2_lifecycle_manager`)
- `robot_localization` (EKF)
- `slam_toolbox`
- Gazebo Classic
- `rplidar_ros`

---

## Robot Specifications

| Parameter | Value |
|-----------|-------|
| **Drive Type** | Omnidirectional (4 mecanum wheels) |
| **Wheel Radius** | 0.1 m |
| **Wheel Base** | 0.495 m (X and Y) |
| **Max Velocity** | 0.5 m/s linear, 1.8 rad/s angular |
| **Sensors** | 2x RPLiDAR A1, IMU, Wheel Encoders |

---

## Build Instructions
```bash
cd ~/qkrt-nav
colcon build
source install/setup.bash
```

---

## Team

**QKRT Navigation Team**
- Competition: ARC Robotics
- Date: 2025

---

## License

Apache 2.0
