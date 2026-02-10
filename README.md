# Sentry Robot - ROS2 Navigation Stack

## Project Structure

### sentry_bringup
**Main launch orchestration**
- `sim.launch.py` - Simulation with Gazebo
- `real_robot.launch.py` - Real robot hardware

### sentry_description
**Robot model and simulation**
- URDF: `sentry_description.urdf` (omnidirectional 4-wheel robot)
- Gazebo world: `my_world.sdf` (18x12m competition arena)
- RViz config: `sentry_config.rviz`
- Plugins: Planar move (omni), LiDAR, IMU

### sentry_localization
**Localization stack**

**Global Localization (AMCL)**
- Laser-based localization using known map
- Motion model: `OmniMotionModel`
- Map: 906x606px (18x12m at 2cm resolution)

**Local Localization (EKF)**
- Sensor fusion: wheel odometry + IMU
- Package: `robot_localization`
- Publishes: `/odometry/filtered` and `odom→base_link` TF

### sentry_communication
**MCB serial communication**
- `comm_hub.py` - Bidirectional UART bridge between MCB and ROS2
  - Receives: 17-float odometry message from MCB (wheel encoders, turret encoders, IMU)
  - Publishes: `Float32MultiArray` on `/mcb_odom` topic
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
- Update rate: 30 Hz

**Smoother Server**
- Algorithm: SimpleSmoother
- Purpose: Smooth jagged paths from planner
- Iterations: 400 with refinement

**Behavior Tree Navigator**
- TBD: Custom competition behavior tree

---

## To Be Implemented

### Priority 1: Robot modelling

#### sentry_description (UPDATE)
- [ ] Create accurate URDF matching real robot
  - Actual dimensions and mass properties
  - Correct wheel positions and sizes
  - Accurate sensor placements (LiDAR, IMU)
  - CAD mesh imports (if available)
  - Proper collision geometry
  - Inertia tensors for realistic dynamics
- [ ] Improve Gazebo simulation
  - Switch from `planar_move` to mecanum controller
  - Add realistic friction/damping
  - Tune physics parameters
- [ ] Update competition world
  - Add obstacles/markers from real arena
  - Accurate wall positions
  - Competition-specific features

### Priority 2: Hardware Integration

#### sentry_communication (UPDATE)
- [x] Serial comm hub (`comm_hub.py`)
  - Receives 17-float odom data from MCB (wheel encoders, turret encoders, full IMU)
  - Publishes `Float32MultiArray` on `/mcb_odom`
  - Subscribes `/cmd_vel` and sends velocity commands to MCB
- [ ] Convert raw `/mcb_odom` into standard ROS messages (`/odom`, `/imu`) for EKF integration
- [ ] LiDAR driver integration (RPLiDAR A1)
  - Topic: `/scan`

### Priority 3: Competition Logic

#### sentry_core (NEW PACKAGE)
- [ ] Battery/health monitor node
  - Publishes: `/battery_low` (Bool)
- [ ] Competition manager node
  - Goal selection based on battery
  - Interfaces with BT Navigator

#### sentry_navigation (UPDATE)
- [ ] Custom competition behavior tree
  - Navigate to center (capture point)
  - Retreat when battery low
  - Hold position at center
- [ ] Fine-tune robot parameters
  - Set accurate `robot_radius` in costmaps
  - Calibrate velocity limits
  - Optimize DWB critics for competition

### Priority 3: Testing & Calibration
- [ ] Test omnidirectional motion (strafe, diagonal)
- [ ] Calibrate EKF sensor fusion
- [ ] Test AMCL localization accuracy
- [ ] Validate Nav2 navigation in competition arena
- [ ] Multi-robot testing (6 robots)

---

## Quick Start

### Simulation
```bash
ros2 launch sentry_bringup simulated_robot.launch.py
```

### Real Robot (when hardware ready)
```bash
ros2 launch sentry_bringup real_robot.launch.py
```

---

## Mapping and Localization

The robot supports two modes controlled by the `slam` argument:

### Mode 1: SLAM Mapping (`slam:=true`)

Use this to create a new map of an environment.

```bash
# Start the robot with SLAM enabled
ros2 launch sentry_bringup real_robot.launch.py slam:=true

# Or in simulation
ros2 launch sentry_bringup simulated_robot.launch.py slam:=true
```

Drive the robot around to map the environment. Monitor in RViz to see the map being built.

**Saving the map:**
```bash
# Save the map (creates my_map.yaml and my_map.pgm)
ros2 run nav2_map_server map_saver_cli -f ~/qkrt-nav/src/sentry_localization/maps/my_map
```

### Mode 2: AMCL Localization (`slam:=false`, default)

Use a pre-built map for localization during operation.

```bash
# Use default map (sentry_map.yaml)
ros2 launch sentry_bringup real_robot.launch.py

# Or specify a custom map
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

## Dependencies

### Installed
- ROS2 Humble
- Nav2 stack
- `robot_localization` (EKF)
- Gazebo Classic
- DWB controller + critics
- SMAC planner

### Required (TBD)
- `rplidar_ros` (for LiDAR)
- `pyserial` (for microcontroller communication)

---


## Robot Specifications (To be updated)

| Parameter | Value |
|-----------|-------|
| **Drive Type** | Omnidirectional (4 mecanum wheels) |
| **Wheel Radius** | 0.05 m |
| **Wheelbase** | 0.3 m (X and Y) |
| **Robot Radius** | 0.15 m (with safety margin) |
| **Max Velocity** | 0.5 m/s linear, 1.8 rad/s angular |
| **Sensors** | LiDAR (360°), IMU, Wheel Encoders |

---

## Competition Arena

| Parameter | Value |
|-----------|-------|
| **Dimensions** | 18m × 12m |
| **Map Resolution** | 0.02 m/pixel (2cm) |
| **Map Size** | 906 × 606 pixels (with 3px border) |
| **Capture Point** | Center (0, 0) |
| **Starting Position** | TBD |

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
- Competition: [ARC Robotics]
- Date: 2025

---

## License

Apache 2.0
