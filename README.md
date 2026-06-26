# Sentry Robot — ROS 2 Navigation Stack

Autonomous navigation stack for a DJI RoboMaster-style omnidirectional sentry robot. The robot is a mecanum-wheeled platform with a decoupled turret/gimbal. It navigates a competition arena using two LiDAR sensors, an onboard IMU (mounted on the turret), and a custom serial link to the microcontroller board (MCB) running DJI firmware.

---

## Table of Contents

0. [CURRENT STATUS - READ FIRST](#current-status---read-first)
1. [VERIFY THESE SIGNS ON REAL HARDWARE](#verify-these-signs-on-real-hardware)
2. [Stack Overview](#stack-overview)
3. [Package Structure](#package-structure)
4. [System Pipeline](#system-pipeline)
5. [Topic Map](#topic-map)
6. [Key Files In Depth](#key-files-in-depth)
   - [comm_hub.py](#comm_hubpy)
   - [real_robot.launch.py](#real_robotlaunchpy)
   - [sentry_description.urdf.xacro](#sentry_descriptionurdfxacro)
   - [ekf.yaml](#ekfyaml)
   - [laser_merger.py](#laser_mergerpy)
   - [battery_mission_controller.py](#battery_mission_controllerpy)
   - [cmd_vel_gimbal_rotator.py](#cmd_vel_gimbal_rotatorpy)
7. [Setup Instructions](#setup-instructions)
8. [Known Bugs](#known-bugs)
9. [To Be Completed](#to-be-completed)

---

## CURRENT STATUS - READ FIRST

Autonav works in simulation. The full stack (SLAM, EKF, Nav2, battery mission controller) launches and runs without crashing in `simulated_robot.launch.py`.

The last known working hardware version is on **origin/develop** — look for the commit **"AUTONAV WORKS, NO BEYBLADING"**. **All current code to be tested is on origin/real_robot_dev**. Everything since then should be the same or better, but the newer code hasn't been tested on the physical robot yet.

**Since the last hardware test:**
- `turret_joint_state_publisher.py` has been merged into `comm_hub.py`. This means the turret joint states are now computed and published directly inside the MCB bridge — one fewer node, and the turret/odom data never has to pass between two separate processes. See the [VERIFY THESE SIGNS](#verify-these-signs-on-real-hardware) section before running on hardware, as several sign conventions in the new code need to be confirmed against real robot behaviour.
- Nav2 velocity commands are now rotated from chassis frame into turret frame before reaching the MCB. This is done by `cmd_vel_gimbal_rotator.py` (real robot only). Same deal — signs need to be verified.
- A battery mission controller has been added. When enabled with `use_battery_mission:=true`, the robot automatically navigates to the arena centre when battery is above 50%, and returns home when below 50%.
- Both LiDAR drivers are now re-enabled in `real_robot.launch.py`. `laser_driver_2` also had a bug where its `topic_name` and `frame_id` were both set to `right` instead of `left` — this has been fixed. **If the left cable is still physically broken, comment out `laser_driver_2` in `real_robot.launch.py` again.**

---

## VERIFY THESE SIGNS ON REAL HARDWARE

These are places in the code where we're pretty confident about the logic but couldn't confirm the +/- direction without physically running the robot. Check each one when you bring it up on hardware for the first time. They're marked with `# VERIFY SIGNS IRL` comments in the code.

### 1. Base angular velocity (`comm_hub.py` ~ line 268)

```python
omega = self.gimbal_vel - self.gyro_z
```

`gimbal_vel` is the turret's absolute yaw velocity from the MCB encoder. `gyro_z` is the turret's absolute yaw velocity from the IMU. Their difference should give you the chassis angular velocity. **Check:** spin only the chassis (no turret movement). `omega` should be positive for CCW, negative for CW. If it's backwards, flip the sign.

### 2. Gimbal joint position and velocity (`comm_hub.py` ~ line 322)

```python
self.turret_pos += dt * (self.gyro_z - omega)
```

This integrates the turret's angle relative to the chassis. **Check in RViz:** hold the chassis still and rotate the turret CCW. The `gimbal_link` in RViz should rotate CCW relative to the chassis. If it goes the wrong way, flip the sign of the whole expression.

### 3. Velocity rotation into turret frame (`cmd_vel_gimbal_rotator.py` ~ line 44)

```python
theta = -self.gimbal_angle
```

This rotates nav2's chassis-frame velocity into turret-frame before it goes to the MCB. **Check:** point the turret 90° left of the chassis, then send nav2 a goal directly in front of the turret. The chassis should strafe left (moving in the direction the turret is pointing). If the chassis goes right instead, change `-self.gimbal_angle` to `+self.gimbal_angle`.

### 4. Chassis odometry orientation (`comm_hub.py` ~ line 290)

```python
odom_msg.pose.pose.orientation.z = math.sin(self.odom_theta / 2.0)
odom_msg.pose.pose.orientation.w = math.cos(self.odom_theta / 2.0)
#THESE SHOULD PROBably BE NEGATIVE ON THE REAL ROBOT
```

**Check:** push the robot forward with the turret aligned to the chassis. RViz should show the robot moving in the +X direction. If it moves in -X or at a weird angle, the yaw convention is off — see Bug #5 in Known Bugs.

---

## Stack Overview

| Layer | Technology |
|---|---|
| OS / middleware | Ubuntu 22.04, ROS 2 Humble |
| Robot firmware | DJI RoboMaster MCB (C++, custom) |
| Jetson ↔ MCB link | UART (`/dev/ttyTHS1`, 115 200 baud) using DJI serial frame protocol |
| Sensing | 2× LDLiDAR LD19 (`/dev/ttyUSB0`, `/dev/ttyUSB1`) |
| State estimation | `robot_localization` EKF fusing wheel odometry + turret-compensated IMU |
| Global localization | SLAM Toolbox (mapping mode) or Nav2 AMCL (pre-built map mode) |
| Navigation | Nav2 — SmacPlanner2D, custom `OmniPursuitController`, SmootherServer |
| Robot model | URDF/Xacro with a revolute `gimbal_joint` representing the decoupled turret |
| Simulation | Gazebo Classic with planar-move and joint-state plugins |

---

## Package Structure

```
src/
├── ldlidar_ros2/               # Third-party LD19 LiDAR driver
├── omni_pursuit_controller/    # Custom Nav2 controller plugin (C++)
├── sentry_bringup/             # Launch files and per-sensor configs
│   ├── launch/
│   │   ├── real_robot.launch.py        # Main entry point for hardware
│   │   ├── simulated_robot.launch.py   # Main entry point for simulation
│   │   └── lidar.launch.py
│   ├── config/
│   │   ├── laser_merger.yaml           # Merged scan output parameters
│   │   ├── rplidar_a1_1.yaml           # (legacy — RPLiDAR configs, now unused)
│   │   └── rplidar_a1_2.yaml
│   └── scripts/
│       ├── laser_merger.py             # Merges two LiDAR scans into /scan
│       └── cmd_vel_gimbal_rotator.py   # Rotates nav2 cmd_vel into turret frame (real robot only)
├── sentry_communication/       # MCB serial bridge (Python ROS 2 node)
│   └── sentry_communication/
│       ├── comm_hub.py             # MCB↔ROS bridge — also computes and publishes /joint_states
│       ├── mcb_message.py          # (legacy helper, unused)
│       └── communication/
│           ├── Message.py          # DJI frame builder (NavMessage, RobotPositionMessage)
│           ├── Receive.py          # DJI frame parser / CRC checker
│           ├── Serial.py           # pyserial wrapper
│           └── CRC.py              # CRC-8 / CRC-16 implementations
├── sentry_description/         # Robot URDF, meshes, RViz configs, Gazebo worlds
│   ├── urdf/
│   │   └── sentry_description.urdf.xacro   # Primary robot model
│   ├── meshes/                 # 3v3 ARC STL files
│   └── worlds/                 # Gazebo SDF world files
├── sentry_localization/        # EKF, SLAM/AMCL, and turret compensation
│   ├── config/
│   │   ├── ekf.yaml                # robot_localization EKF parameters
│   │   ├── amcl.yaml               # Nav2 AMCL parameters
│   │   └── slam_toolbox.yaml       # SLAM Toolbox parameters
│   ├── launch/
│   │   ├── local_localization.launch.py    # EKF + IMU compensator
│   │   └── global_localization.launch.py   # SLAM or AMCL
│   ├── maps/                   # Pre-built map + keepout/ramp masks
│   └── scripts/
│       ├── turret_joint_state_publisher.py # DEPRECATED — logic moved into comm_hub.py
│       ├── imu_turret_compensator.py       # Removes turret motion from IMU before EKF
│       └── sim_comm_hub_bridge.py          # Sim equivalent of comm_hub for Gazebo
└── sentry_navigation/          # Nav2 configuration, behavior trees, and mission logic
    ├── config/
    │   ├── controller_server.yaml  # OmniPursuitController params + local costmap
    │   ├── planner_server.yaml     # SmacPlanner2D global planner + global costmap
    │   ├── bt_navigator.yaml
    │   └── smoother_server.yaml
    ├── behavior_tree/
    │   ├── simple_navigation.xml       # Nav2 BT used by the mission controller
    │   └── replanning_navigation.xml
    ├── scripts/
    │   ├── battery_mission_controller.py   # Sends nav goals based on battery level
    │   └── battery_health_publisher.py     # Dummy battery data for testing (replace with real sensor)
    └── launch/
        └── navigation.launch.py
```

---

## System Pipeline

```
MCB firmware (DJI C++)
        │  UART /dev/ttyTHS1 @ 115200
        ▼
  comm_hub.py
  ├─ Parses DJI serial frames (0xA5 header, CRC-8/CRC-16)
  ├─ Publishes /odom         (mecanum forward kinematics from 4 wheel shaft rad/s)
  ├─ Publishes /imu          (linear accel + gyro + euler→quaternion orientation)
  ├─ Publishes /turret       (yawPos, yawVel, pitchPos, pitchVel — debug/reference)
  ├─ Publishes /wheel_speeds (raw wheel rad/s — debug)
  ├─ Publishes /joint_states (gimbal_joint angle + pitch + wheel joints)
  │    ↳ gimbal angle is integrated from (gyro_z - omega) each frame
  │    ↳ this used to be done by turret_joint_state_publisher.py (now merged here)
  └─ Subscribes /cmd_vel_rotated → NavMessage → serial write

        │  [real robot only]
        ▼
  cmd_vel_gimbal_rotator.py
  ├─ Subscribes /cmd_vel (nav2 output, in chassis/base_link frame)
  ├─ Reads current gimbal_joint angle from /joint_states
  ├─ Rotates [vx, vy] so "forward" means turret-forward, not chassis-forward
  └─ Publishes /cmd_vel_rotated → comm_hub

        │
        ▼
  robot_state_publisher
  └─ Reads URDF + /joint_states → full TF tree
     (base_link → gimbal_link → turret → laser frames → imu_link)

        │
        ▼
  imu_turret_compensator.py   [real robot only]
  ├─ Subscribes /imu (mounted on turret) + /joint_states
  ├─ Removes centripetal/tangential acceleration and turret angular velocity
  └─ Publishes /imu_base_compensated  (IMU re-expressed in base_link frame)

        │ /odom  /imu_base_compensated
        ▼
  robot_localization EKF  (ekf_node @ 40 Hz)
  ├─ Fuses odom (vx, vy, omega) + compensated IMU (gyro_z)
  └─ Publishes /odometry/filtered + odom→base_link TF

        │
        ▼
  LDLiDAR LD19 ×2  (/scan_left, /scan_right)
        │
        ▼
  laser_merger.py
  ├─ Per-ray deskewing using TF (bridges through odom for full motion correction)
  └─ Publishes /scan  (merged 360° scan in base_link frame)

        │ /scan  /odometry/filtered
        ▼
  Global Localization
  ├─ SLAM mode:  slam_toolbox  → publishes map→odom TF
  └─ AMCL mode:  map_server + amcl → publishes map→odom TF

        │ /scan  /odometry/filtered  map→odom TF
        ▼
  Nav2 Stack
  ├─ SmacPlanner2D    → global path
  ├─ OmniPursuitController → /cmd_vel  (holonomic pure pursuit, zero angular output)
  ├─ SmootherServer   → smoothed path
  └─ BT Navigator     → action server for goal commands

        │  [when use_battery_mission:=true]
        ▼
  battery_mission_controller.py
  ├─ Subscribes /battery_health  (Float32, 0–100)
  ├─ Captures home position from map→base_link TF at startup
  ├─ Battery >= 50% → sends NavigateToPose goal to arena centre
  └─ Battery <  50% → sends NavigateToPose goal back to home
```

---

## Topic Map

| Topic | Type | Producer | Consumer |
|---|---|---|---|
| `/cmd_vel` | `Twist` | Nav2 / teleop | `cmd_vel_gimbal_rotator` (real), Gazebo (sim) |
| `/cmd_vel_rotated` | `Twist` | `cmd_vel_gimbal_rotator` | `comm_hub` (real robot only) |
| `/odom` | `Odometry` | `comm_hub` / Gazebo | EKF |
| `/imu` | `Imu` | `comm_hub` | `imu_turret_compensator` |
| `/turret` | `Float32MultiArray` | `comm_hub` | (debug reference) |
| `/wheel_speeds` | `Float32MultiArray` | `comm_hub` | (debug) |
| `/mcb_odom` | `Float32MultiArray` | `comm_hub` | (debug — all 17 floats raw) |
| `/joint_states` | `JointState` | `comm_hub` (real) / Gazebo (sim) | `robot_state_publisher`, `imu_turret_compensator`, `cmd_vel_gimbal_rotator` |
| `/imu_base_compensated` | `Imu` | `imu_turret_compensator` | EKF |
| `/odometry/filtered` | `Odometry` | EKF | Nav2, SLAM/AMCL |
| `/scan_left` `/scan_right` | `LaserScan` | LDLiDAR nodes | `laser_merger` |
| `/scan` | `LaserScan` | `laser_merger` | SLAM/AMCL, Nav2 costmaps |
| `/battery_health` | `Float32` | `battery_health_publisher` (dummy) or your real sensor | `battery_mission_controller` |

---

## Key Files In Depth

### `comm_hub.py`
`src/sentry_communication/sentry_communication/comm_hub.py`

The hardware bridge between the Jetson and the MCB. It opens a single UART connection at `/dev/ttyTHS1` at startup.

**Two ROS 2 nodes run inside a `MultiThreadedExecutor`:**

- **`NavSubscriber`** — listens on `/cmd_vel_rotated` and immediately serializes a `NavMessage` and writes it to the MCB.

- **`OdomPublisher`** — runs a background thread that reads incoming DJI serial frames. On a valid 68-byte odom frame it unpacks 17 floats (`ODOM_LABELS`):
  - Indices 0–3: wheel shaft speeds → mecanum kinematics → publishes `/odom`
  - Indices 4–7: turret yaw/pitch position and velocity
  - Indices 8–16: IMU linear acceleration, gyro, and euler angles → publishes `/imu`

**Joint states (merged from turret_joint_state_publisher):**

`comm_hub` now also computes and publishes `/joint_states` directly, instead of shipping that data to a separate node. This was done so that all the MCB data (odom, IMU, turret encoder) is available in the same place when computing joint angles — no more passing data between nodes via topics just to combine it.

The gimbal joint angle is integrated each frame:
```
turret_pos += dt * (gyro_z - omega)
```
`gyro_z` is the turret's absolute angular velocity from the IMU. `omega` is the chassis angular velocity computed from wheel odometry. Their difference is how fast the turret is rotating relative to the chassis. See [VERIFY THESE SIGNS](#verify-these-signs-on-real-hardware) before running on hardware.

**Key constants:**
- `WHEEL_RADIUS = 0.038 m`
- `GEAR_RATIO = 9.75` (empirically calibrated — nominal is 19:1 but measured as ~9.75)
- `WHEEL_BASE_X = WHEEL_BASE_Y = 0.2475 m`
- `IMU_BIAS = 0` — if the turret drifts, average `/imu` angular_velocity.z while stationary and set this

---

### `real_robot.launch.py`
`src/sentry_bringup/launch/real_robot.launch.py`

The single entry point for bringing up the full robot on hardware. All subsystems are controlled by launch arguments:

| Argument | Default | Description |
|---|---|---|
| `slam` | `true` | Use SLAM Toolbox for mapping. Set `false` to use AMCL with a pre-built map. |
| `map` | `sentry_map.yaml` | Map file when `slam:=false`. Must be in `sentry_localization/maps/`. |
| `robot_model` | `sentry_description.urdf.xacro` | URDF file name in `sentry_description/urdf/`. |
| `use_localization` | `true` | Launch SLAM or AMCL. Set `false` to skip map entirely. |
| `use_keepout` | `true` | Load the arena keepout filter into Nav2 costmaps. Set `false` outside the arena. |
| `use_nav` | `true` | Launch the Nav2 navigation stack. |
| `use_battery_mission` | `false` | Launch the battery mission controller (see below). |
| `center_x` | `6.0` | X coordinate of arena centre in the map frame (used by battery mission). |
| `center_y` | `4.0` | Y coordinate of arena centre in the map frame (used by battery mission). |
| `display` | `false` | Launch RViz. |

**Node launch order:**
1. `robot_state_publisher` — URDF → TF
2. `comm_hub` — MCB serial bridge, subscribed to `/cmd_vel_rotated`
3. `cmd_vel_gimbal_rotator` — rotates `/cmd_vel` into turret frame, publishes `/cmd_vel_rotated`
4. `ldlidar_ros2_node` ×1 (left cable broken — re-enable when fixed)
5. `laser_merger` — merges scans into `/scan`
6. `teleop_twist_keyboard` — opens in a new `xterm` window
7. `global_localization` (sub-launch) — SLAM or AMCL
8. `local_localization` (sub-launch) — EKF + IMU compensator
9. `navigation` (sub-launch) — Nav2 stack + battery mission (if enabled)
10. `rviz2` (if `display:=true`)

---

### `battery_mission_controller.py`
`src/sentry_navigation/scripts/battery_mission_controller.py`

Watches battery level and autonomously decides where the robot should go. Launched as part of the navigation sub-launch when `use_battery_mission:=true`.

**How to turn it on:**
```bash
ros2 launch sentry_bringup real_robot.launch.py use_battery_mission:=true center_x:=6.0 center_y:=4.0
```

**What it does:**
- Waits for the map→base_link TF to be available (i.e. SLAM/AMCL has a fix), then locks in the robot's starting position as "home"
- Listens to `/battery_health` (a `std_msgs/Float32` between 0 and 100)
- Battery ≥ 50% → sends a nav2 goal to `(center_x, center_y)`
- Battery < 50% → sends a nav2 goal back to the captured home position
- If the battery crosses the threshold while the robot is already moving, it cancels the current goal and sends a new one

**Plugging in real battery data:**
Replace `battery_health_publisher.py` with a node that reads your actual battery sensor and publishes a `Float32` to `/battery_health`. The mission controller doesn't care where the number comes from — just that it arrives on that topic in the 0–100 range.

**The dummy publisher (`battery_health_publisher.py`):**
For testing only. Publishes a triangle wave that goes 100 → 0 → 100 over 40 seconds, crossing the 50% threshold every 10 seconds. This lets you watch the robot switch targets back and forth in simulation without needing real hardware.

**Arena centre coordinates:**
These depend on your map. With SLAM, the map origin is wherever the robot started, so `center_x`/`center_y` should be the offset from the robot's starting corner to the physical arena centre. With AMCL and a pre-built map, they're absolute map coordinates measured once and hardcoded.

---

### `cmd_vel_gimbal_rotator.py`
`src/sentry_bringup/scripts/cmd_vel_gimbal_rotator.py`

Sits between Nav2 and the MCB on the real robot only (not used in sim). Nav2 plans paths relative to the chassis (`base_link`), but the MCB expects velocity commands relative to the turret direction. This node does the coordinate conversion.

If the turret is pointing 90° left of the chassis and nav2 says "go forward at 0.5 m/s", the MCB needs to hear "strafe left at 0.5 m/s" so the robot actually moves in the direction the turret is pointing. That's what this does — it reads the current `gimbal_joint` angle from `/joint_states` and rotates the velocity vector accordingly.

**Topic flow:** `/cmd_vel` (nav2) → this node → `/cmd_vel_rotated` → `comm_hub` → MCB

**Signs need to be verified on hardware** — see the [VERIFY THESE SIGNS](#verify-these-signs-on-real-hardware) section.

---

### `sentry_description.urdf.xacro`
`src/sentry_description/urdf/sentry_description.urdf.xacro`

Defines the full kinematic and visual model of the robot. Key design decisions:

**Frame hierarchy:**
```
base_link
├── front_left_wheel_joint   (continuous)
├── front_right_wheel_joint  (continuous)
├── back_left_wheel_joint    (continuous)
├── back_right_wheel_joint   (continuous)
└── gimbal_joint             (continuous, axis = [0,0,-1])
    └── gimbal_link
        └── bottom_support_joint (fixed)
            └── bottom_support_link
                ├── right_support_joint (fixed) → right_support_link
                │   └── laser_joint_right (fixed) → laser_frame_right
                ├── left_support_joint (fixed) → left_support_link
                │   └── laser_joint_left (fixed) → laser_frame_left
                └── turret_shaft_joint (revolute, near-locked)
                    └── turret_shaft_link
                        └── turret_joint (fixed) → turret_link
                            ├── imu_joint (fixed) → imu_link
                            ├── gun_shaft_joint (fixed) → gun_shaft_link
                            └── depth_camera_joint (fixed) → depth_camera_link
```

**Critical gimbal design:** `gimbal_joint` has `axis = [0,0,-1]` (negative Z). A CCW chassis spin should leave the turret stationary in the world — the negative axis combined with the sign convention in `comm_hub` is what makes that happen. If the turret spins in the wrong direction in RViz during a chassis rotation, check this axis and the signs in `comm_hub` together.

**LiDAR placement:** Both LiDARs are children of the turret, so they rotate with the turret, not the chassis. The laser_merger deskews for this.

**IMU:** Attached to `turret_link` via a fixed joint. The `imu_turret_compensator` corrects for turret motion before passing IMU data to the EKF.

**Gazebo:** Uses `libgazebo_ros_planar_move.so` for omnidirectional motion and `libgazebo_ros_joint_state_publisher.so` for joint states. In sim, Gazebo publishes `/joint_states` directly, so `comm_hub` (and the old `turret_joint_state_publisher`) don't run.

---

### `ekf.yaml`
`src/sentry_localization/config/ekf.yaml`

Configures `robot_localization`'s `ekf_node` running at 40 Hz in 2D mode.

**Inputs fused:**

- **`odom0: /odom`** — from `comm_hub`. Fuses `vx`, `vy`, and `omega` (chassis angular velocity).

- **`imu0: /imu_base_compensated`** — from `imu_turret_compensator`. Fuses only gyro Z (yaw rate). The compensated IMU reinforces the wheel odometry yaw estimate without introducing the turret's own rotation into the chassis pose.

The IMU orientation and linear acceleration are disabled. Only the compensated gyroscope yaw rate is used.

**Sim vs real:** In sim, the EKF uses `/imu_comm_hub` (from `sim_comm_hub_bridge`) and disables `publish_tf` since Gazebo's planar_move plugin already publishes `odom → base_link`. On real hardware, `publish_tf: true` so the EKF owns that transform.

---

### `laser_merger.py`
`src/sentry_bringup/scripts/laser_merger.py`

Subscribes to `/scan_left` and `/scan_right` from the two LD19s and publishes a single merged `/scan` at 20 Hz. The non-trivial part is deskewing — a LiDAR spinning at 10 Hz takes ~100 ms per revolution, during which the robot and turret have moved. Without correction, each ray was captured from a different physical position, smearing obstacles and corrupting SLAM.

**Deskewing method:** For each ray, the node looks up the TF from the laser frame at the exact time that ray was captured, transforming it into the `base_link` frame at the latest available time. It uses `odom` as a bridge frame so it accounts for both turret rotation (via the `gimbal_joint` TF) and robot body translation.

**Three fallback modes** (tried in order):

| Mode | How | Corrects |
|---|---|---|
| `full` | TF lookup bridged through odom at exact ray time | Turret rotation + body motion |
| `turret` | TF lookup at exact ray time (no bridge) | Turret rotation only |
| `latest` | TF lookup at time=0 (newest available) | Nothing — one static transform per scan |

**LD19 scan direction note:** With `laser_scan_dir: True` in the driver, index 0 is the last-captured ray and index `N-1` is the first. Interpolation alpha accounts for this.

---

## Setup Instructions

### Starting Up the Physical Robot
1. Unplug the chassis power cable from the referee serial module
2. Insert a battery. Double-click the power button, hold on the second click to activate
3. Flick the switch from 'O' to 'I' to activate power distribution
4. While the robot powers on, hold the turret level so the turret-mounted IMU calibrates properly
5. Once you hear startup sounds and see 3 green lights on the serial module (chassis, gimbal, ammo booster), replug the chassis power cable
6. Turn the remote on. If not paired, push the button under the back-left of the turret with a pen until it flashes, then pair
7. Flick the left switch down on the remote to accept autonav commands. Neutral = controller only (safety)

Troubleshooting:
- If the chassis light turns off after reinserting the cable: remove the Jetson power cable, turn power distribution off and on, try again. Replug the Jetson once it works. If that doesn't work: keep setup the same, turn on power distribution with chassis cable already in. Pray.
- If the turret spins erratically or too fast: IMU lost calibration. Restart from step 1
- If SSH drops: unplug and replug the room router

The Jetson connects to **QKRT-5G** (or **QKRT** if 5G fails). Password: `goodlife`

SSH in with:
```bash
ssh -X qkrt@192.168.8.102   # password: qkrt123
cd qkrt-nav
```

### Hardware Requirements

- Mini PC: Nvidia Jetson, flashed with Ubuntu 22.04.5 LTS
- MCB firmware branch: `feat/auto_nav_comms_working`
- LiDAR serial assignments: `/dev/ttyUSB0` (left?) and `/dev/ttyUSB1` (right?) — verify with `ls /dev/ttyUSB*`

### Dependency Installation

```bash
# Clone the LDLiDAR driver into src/ if not already present
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git src/ldlidar_ros2

# Install all ROS and system dependencies
sudo rosdep init   # skip if already done
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# pyserial is not in rosdep — install manually
pip3 install pyserial
```

### Build

```bash
cd qkrt-nav
colcon build --symlink-install
source install/setup.bash
```

### Udev / Permissions

All three serial devices need to be accessible without sudo:
```bash
sudo usermod -aG dialout $USER
# log out and back in, or run: newgrp dialout
```

To lock in which LiDAR is left vs right (so USB assignment doesn't flip on reboot):
```bash
# Plug in one LiDAR at a time, find its port path
udevadm info -a /dev/ttyUSB0 | grep 'KERNELS==' | head -3
```

Create `/etc/udev/rules.d/99-sentry-lidar.rules`:
```
SUBSYSTEM=="tty", KERNELS=="<usb-path-of-left-lidar>",  SYMLINK+="lidar_left"
SUBSYSTEM=="tty", KERNELS=="<usb-path-of-right-lidar>", SYMLINK+="lidar_right"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Then update `real_robot.launch.py` to use `/dev/lidar_left` and `/dev/lidar_right`.

### Running on Hardware

On your local machine, set your ROS domain ID to match the Jetson:
```bash
export ROS_DOMAIN_ID=67
# or add it to ~/.bashrc to make it permanent
```

Open RViz locally (better performance than running it on the Jetson):
TODO: Command is wrong - fix this
```bash
rviz2 -d /local-path-to/src/sentry_description/rviz/sentry_config.rviz
```

TODO: Run rviz with debug rviz config
rviz2 -d ~/qkrt/qkrt-nav/src/sentry_description/rviz/slam_debug.rviz

TODO: Save a slam map
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name: {data: 'tron_room'}"

TODO: Run this command for going back and forth rn
ros2 launch sentry_bringup real_robot.launch.py     slam:=false     map:=tron_room.yaml     use_keepout:=false     use_battery_mission:=true     center_x:=3.0     center_y:=0.0


On the Jetson, launch the full stack:
```bash
# Full bringup with SLAM (default)
# TODO: Consider preventing keepout in other rooms for testing with: ros2 launch sentry_bringup real_robot.launch.py use_keepout:=false
ros2 launch sentry_bringup real_robot.launch.py

# With pre-built map (AMCL)
ros2 launch sentry_bringup real_robot.launch.py slam:=false map:=sentry_map.yaml

# Without Nav2 (odometry + sensor testing only)
ros2 launch sentry_bringup real_robot.launch.py use_nav:=false use_localization:=false

# With battery mission controller enabled
ros2 launch sentry_bringup real_robot.launch.py use_battery_mission:=true center_x:=6.0 center_y:=4.0

# Simulation
ros2 launch sentry_bringup simulated_robot.launch.py

# Simulation with battery mission
ros2 launch sentry_bringup simulated_robot.launch.py use_battery_mission:=true
```

---

## Known Bugs

**IN THE CURRENT VERSION, NO BASE ROTATION IS ACCOUNTED FOR. BEYBLADING DOES NOT WORK, AND NEITHER DO SMALL ROTATIONS**

1. **Odometry drift on pure rotation ("beyblading")** — The mecanum kinematics give reasonable translation but accumulate yaw error during sustained spinning. The `GEAR_RATIO` of 9.75 was tuned for straight-line travel; rotational calibration was never verified separately. If the chassis orientation in RViz doesn't match reality during rotation, odometry will be wrong, which breaks localization. If the robot moves forward in real life but moves at an angle in RViz, this is the likely cause.

2. **Odometry drift on pure translation** — Works fine currently. If it regresses: the `GEAR_RATIO` was empirically tuned and may need re-calibration. Lidar scans that physically drift and smear during translation are also a symptom.

3. **LiDAR deskewing fails at high rotation speeds** — At fast turn speeds, scans can be rotationally distorted and SLAM may generate phantom walls. Believed cause is the linear interpolation approximation breaking down at high speeds, but not confirmed.

4. **LiDAR deskewing falls back to `latest` mode** — When the EKF TF isn't available at the exact scan timestamp, `laser_merger.py` falls back and logs a warning. In `latest` mode there's no per-ray deskewing, which smears the scan during fast motion.

5. **Fixed yaw misalignment between chassis and turret** — At launch the turret can have a fixed angular offset from the chassis. To fix: align the turret parallel to the mecanum wheels, then in the URDF find `gimbal_joint` and tweak the yaw in `<origin xyz="..." rpy="0 0 ${1.5708}"/>`. Common working values: `0`, `1.5708`, `${1.5708 + 1.05/2}`, `${1.05/2}`. To verify, push the robot forward, run `ros2 topic echo /odom --once`, and confirm it moved in +X. If it moved backwards, negate the offset. Root cause: the MCB encoder yaw doesn't reset between ROS restarts and can drift or reset randomly.

6. **No transform between `odom → base_link`:**
   - a. MCB isn't sending data — check `ros2 topic echo /odom --once`
   - b. `comm_hub.py` isn't receiving frames — check `ros2 topic echo /mcb_raw`
   - c. `comm_hub.py` is crashing on startup (serial port not found)
   - d. `ekf.yaml` has a misconfiguration

7. **No `map → odom` transform** — Either: a. LiDAR drivers silently failing, b. SLAM can't generate a map (bad scan quality), or c. `laser_merger.py` is failing.

8. **Time delay between turret rotation and deskewed scan updating** — Sometimes there's a noticeable lag between physically rotating the turret and seeing the deskewed scan update. Cause unknown.

9. **Turret position drifts over time** — The IMU has gyro drift. Go to `comm_hub.py`, reset `IMU_BIAS = 0` and check if that fixes it. If not, run `ros2 topic echo /imu --field angular_velocity.z`, average the values while the robot is stationary, and set `IMU_BIAS` to that average.

10. **Signs in the new joint state + cmd_vel rotation code are unverified** — `comm_hub.py` and `cmd_vel_gimbal_rotator.py` were written based on the correct math but have not been run on real hardware yet. See [VERIFY THESE SIGNS](#verify-these-signs-on-real-hardware) for exactly what to test.

---

## To Be Completed

### Critical

- **ONLY IF ROTATION IS NECESSARY: Fix base rotation → turret drift bug** — If beyblading is needed, the gimbal joint angle calculation in `comm_hub.py` needs to be re-verified on hardware. The math is:
  - `turret_pos += dt * (gyro_z - omega)` where `omega = gimbal_vel - gyro_z`
  - Test by spinning only the chassis — the turret should stay still in RViz
  - If it doesn't, the signs are wrong somewhere in `comm_hub.py`

- **Verify all signs on hardware** — Before relying on autonomous navigation on the real robot, go through the [VERIFY THESE SIGNS](#verify-these-signs-on-real-hardware) section item by item.

- **End-to-end autonomous navigation working** — Get the robot reliably navigating around obstacles without breaking down.

- **Replace dummy battery publisher with real data** — `battery_health_publisher.py` is a fake that ramps 100→0→100 for testing. Replace it with a node that reads the actual robot battery and publishes to `/battery_health`. Everything else is already wired up.

- **Tune arena centre coordinates for AMCL map** — `center_x: 6.0` and `center_y: 4.0` are estimates based on the 12×8m arena. Once you have a real AMCL map, measure the actual centre in map coordinates and update the defaults in `navigation.launch.py` (or pass them as launch args).

- **Write autonav/autoaim startup script** — Script that launches autonav and autoaim on Jetson startup.

- **Sentry_vision integration if time permits**

### Should Have

- **Separate odometry and IMU covariance tuning** — The covariance values in `comm_hub.py` and `ekf.yaml` are placeholder 0.01s. Should be tuned with actual sensor noise measurements.

- **Confirm LiDAR USB assignments** — Document and enforce which LiDAR is `ttyUSB0` (left) and which is `ttyUSB1` (right). A swap causes both scans to use the wrong TF frame.

- **Map saving workflow** — After running SLAM, save with `map_saver_cli`. Document the exact command and where to put the output files for AMCL mode.

- **`comm_hub` reconnect on serial error** — If the MCB reboots or UART disconnects, `comm_hub` crashes. It should catch `SerialException` and attempt reconnection.

- **Remove or fix legacy `mcb_message.py`** — Exists in the package but is not imported anywhere. Either delete it or consolidate with `communication/Message.py`.

- **Generate AMCL map from SDF mesh** — Since the 2026 ARC CAD model already exists as an STL (`2026_ARC_3v3_simplified.stl`), it's possible to raycast it into a nav2-compatible PGM/YAML map offline without needing to SLAM the physical arena. This would give a perfect pre-built map before the competition.

### Nice to Have

- **Add `imu_link` to EKF orientation fusion** — The EKF currently only uses gyro Z from the compensated IMU. Enabling the full orientation quaternion (once turret compensation is verified correct) would improve yaw accuracy during fast maneuvers.

- **Depth camera integration** — The URDF and Gazebo plugin for the depth camera are defined but the topic is not used anywhere. Point cloud to laserscan conversion would improve close-range obstacle detection.

- **CI / automated smoke test** — A `colcon test` target that launches the simulated robot, sends a goal, and checks that `/odometry/filtered` is being published would catch regressions from future changes.

- **Fix turret pitch control** — Pitch position is read from the MCB and published in `/joint_states` as `turret_shaft_joint`. The signs on the pitch values may be inverted.

- **Test/attempt to use the Sentry_Vision package** — Test CV localization in tandem with the other forms of localization.
