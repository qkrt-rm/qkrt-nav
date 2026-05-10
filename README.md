# Sentry Robot — ROS 2 Navigation Stack

Autonomous navigation stack for a DJI RoboMaster-style omnidirectional sentry robot. The robot is a mecanum-wheeled platform with a decoupled turret/gimbal. It navigates a competition arena using two LiDAR sensors, an onboard IMU (mounted on the turret), and a custom serial link to the microcontroller board (MCB) running DJI firmware.

---

## Table of Contents

0. [CURRENT_STATUS - READ FIRST](#current-status---read-first)
1. [Stack Overview](#stack-overview)
2. [Package Structure](#package-structure)
3. [System Pipeline](#system-pipeline)
4. [Topic Map](#topic-map)
5. [Key Files In Depth](#key-files-in-depth)
   - [comm_hub.py](#comm_hubpy)
   - [turret_joint_state_publisher.py](#turret_joint_state_publisherpy)
   - [real_robot.launch.py](#real_robotlaunchpy)
   - [sentry_description.urdf.xacro](#sentry_descriptionurdfxacro)
   - [ekf.yaml](#ekfyaml)
6. [Setup Instructions](#setup-instructions)
7. [Known Bugs](#known-bugs)
8. [To Be Completed](#to-be-completed)

---

## CURRENT STATUS - READ FIRST
We have autonav working with no beyblading for sure on **origin/real_robot_dev**. Look at the git log for **"AUTONAV WORKS, NO BEYBLADING"** commit. That was the latest working version. Everything after should be the same, but I removed clutter files and have not tested this branch to make sure it still works. I will also merge the decluttered version with **origin/develop**

As of testing during the late night of May 10th, the robot can barely sometimes navigate through walls, and appears to "dance" when it gets too close to walls or confused in its navigation. this is the first error to debug.

Also, one of the lidar cables broke. **Be sure to reenable the left laser driver node in real_robot.launch.py when this is fixed, and ensure the right usb assignments are present (left vs. right, /dev/ttyUSB0 vs /dev/ttyUSB1)**

The main script, **turret_joint_state_publisher.py** is really messy, and some code can be technically taken out. The working version uses purely the gimbal encoder values to determine orientation and angular velocity. Tony is working on a version that uses imu angular velocity for the turret and imu angular velocity - encoder angular velocity for the chassis angular velocity.

Priority order in [To Be Completed](#to-be-completed) is still correct. **Look at the first critical item to fix. If we need beyblading, we need to implement that. Tony should try while luke works on nav stack**.


## Stack Overview

| Layer | Technology |
|---|---|
| OS / middleware | Ubuntu 22.04, ROS 2 Humble |
| Robot firmware | DJI RoboMaster MCB (C++, custom) |
| Jetson ↔ MCB link | UART (`/dev/ttyTHS1`, 115 200 baud) using DJI serial frame protocol |
| Sensing | 2× LDLiDAR LD19 (`/dev/ttyUSB0`, `/dev/ttyUSB1`) |
| State estimation | `robot_localization` EKF fusing wheel odometry + turret-compensated IMU |
| Global localization | SLAM Toolbox (mapping mode) or Nav2 AMCL (pre-built map mode) |
| Navigation | Nav2 — NavFn planner, custom `OmniPursuitController`, SmootherServer |
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
│   │   ├── real_robot.launch.py    # Main entry point for hardware
│   │   ├── simulated_robot.launch.py
│   │   └── lidar.launch.py
│   ├── config/
│   │   ├── laser_merger.yaml       # Merged scan output parameters
│   │   ├── rplidar_a1_1.yaml       # (legacy — RPLiDAR configs, now unused)
│   │   └── rplidar_a1_2.yaml
│   └── scripts/
│       └── laser_merger.py         # Merges two LiDAR scans into /scan
├── sentry_communication/       # MCB serial bridge (Python ROS 2 node)
│   └── sentry_communication/
│       ├── comm_hub.py             # Main MCB↔ROS bridge node
│       ├── mcb_message.py          # (legacy helper)
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
│   │   ├── local_localization.launch.py    # EKF + turret JSP + IMU compensator
│   │   └── global_localization.launch.py   # SLAM or AMCL
│   ├── maps/                   # Pre-built map + keepout/ramp masks
│   └── scripts/
│       ├── turret_joint_state_publisher.py # Publishes /joint_states from turret encoder + IMU
│       ├── imu_turret_compensator.py       # Removes turret motion from IMU before EKF
│       └── sim_comm_hub_bridge.py          # Sim equivalent of comm_hub for Gazebo
└── sentry_navigation/          # Nav2 configuration and behavior trees
    ├── config/
    │   ├── controller_server.yaml  # OmniPursuitController params + local costmap
    │   ├── planner_server.yaml     # NavFn global planner
    │   ├── bt_navigator.yaml
    │   └── smoother_server.yaml
    ├── behavior_tree/
    │   ├── simple_navigation.xml
    │   └── replanning_navigation.xml
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
  ├─ Publishes /odom   (mecanum forward kinematics from 4 wheel shaft rad/s)
  ├─ Publishes /imu    (linear accel + gyro + euler→quaternion orientation)
  ├─ Publishes /turret (yawPos, yawVel, pitchPos, pitchVel)
  ├─ Publishes /wheel_speeds
  └─ Subscribes /cmd_vel → NavMessage → serial write

        │
        ▼
  turret_joint_state_publisher.py
  ├─ Subscribes /turret, /imu, /odom
  ├─ Integrates (imu.gyro_z - odom.omega) → gimbal_joint angle
  └─ Publishes /joint_states  (gimbal_joint, turret_shaft_joint, and technically 4x wheel joints but they are given a dummy value of 0.0)

        │
        ▼
  robot_state_publisher
  └─ Reads URDF + /joint_states → full TF tree
     (base_link → gimbal_link → turret → laser frames → imu_link)

        │
        ▼
  imu_turret_compensator.py
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
  ├─ NavFn planner   → global path
  ├─ OmniPursuitController → /cmd_vel  (holonomic pure pursuit, zero angular output)
  ├─ SmootherServer  → smoothed path
  └─ BT Navigator    → action server for goal commands
```

---

## Topic Map

| Topic | Type | Producer | Consumer |
|---|---|---|---|
| `/cmd_vel` | `Twist` | Nav2 / teleop | `comm_hub` |
| `/odom` | `Odometry` | `comm_hub` | EKF, `turret_jsp` |
| `/imu` | `Imu` | `comm_hub` | `imu_turret_compensator`, `turret_jsp` |
| `/turret` | `Float32MultiArray` | `comm_hub` | `turret_jsp` |
| `/wheel_speeds` | `Float32MultiArray` | `comm_hub` | (debug) |
| `/mcb_odom` | `Float32MultiArray` | `comm_hub` | (debug — all 17 floats) |
| `/joint_states` | `JointState` | `turret_jsp` | `robot_state_publisher`, `imu_turret_compensator` |
| `/imu_base_compensated` | `Imu` | `imu_turret_compensator` | EKF |
| `/odometry/filtered` | `Odometry` | EKF | Nav2, SLAM/AMCL |
| `/scan_left` `/scan_right` | `LaserScan` | LDLiDAR nodes | `laser_merger` |
| `/scan` | `LaserScan` | `laser_merger` | SLAM/AMCL, Nav2 costmaps |

---

## Key Files In Depth

### `comm_hub.py`
`src/sentry_communication/sentry_communication/comm_hub.py`

The hardware bridge between the Jetson and the MCB. It opens a single UART connection at `/dev/ttyTHS1` at startup (module-level singleton `serial`).

**Two ROS 2 nodes run inside a `MultiThreadedExecutor`:**

- **`NavSubscriber`** — listens on `/cmd_vel` and immediately serializes a `NavMessage` (message type `0x02`, payload = `[Vx, Vy, omega]` as three little-endian floats) and writes it to the MCB.

- **`OdomPublisher`** — runs a dedicated background thread (`_read_loop`) that blocks on `read_frame()`, synchronizing on the DJI `0xA5` start byte, then reading the 4-byte header, 2-byte message type, N-byte payload, and 2-byte CRC16. On a valid frame of type `MCB_MESSAGE_TYPE_ODOM = 3`:
  - If the body is 4 bytes it treats it as a test counter.
  - If the body is 68 bytes (17 × `float32`) it unpacks `ODOM_LABELS`:
    - Indices 0–3: wheel shaft speeds (rad/s, already gear-reduced). The gear ratio is empirically calibrated at **9.75** (not the nominal 19:1).
    - Indices 4–7: turret yaw position/velocity, pitch position/velocity → published on `/turret`.
    - Indices 8–16: IMU linear acceleration, gyro, and euler angles (yaw/pitch/roll). Euler angles are converted to a quaternion in ZYX convention and published on `/imu`.
  - Mecanum forward kinematics compute `vx`, `vy`, `omega` in the chassis frame. Wheel signs are negated relative to MCB convention. Odometry pose is integrated and published on `/odom` with `child_frame_id = base_link`. **The quaternion orientation sign is negated** (both `.z` and `.w`) to match the ROS convention that CCW rotation is positive.

**Key constants:**
- `WHEEL_RADIUS = 0.038 m`
- `GEAR_RATIO = 9.75` (empirically calibrated)
- `WHEEL_BASE_X = WHEEL_BASE_Y = 0.2475 m`

---

### `turret_joint_state_publisher.py`
`src/sentry_localization/scripts/turret_joint_state_publisher.py`

Translates raw hardware data into `/joint_states` so `robot_state_publisher` can maintain the TF tree with the correct turret angle.

The central problem: the IMU is physically mounted on the turret, not on the chassis. When the robot's chassis rotates, the turret (and IMU) can remain stationary in the world, or vice versa. The gimbal joint angle in the URDF represents turret angle *relative to the chassis*.

**How gimbal angle is computed:**

```
d(gimbal_joint)/dt = odom.omega - imu.gyro_z
```

`odom.omega` is the chassis angular velocity (from wheel odometry). `imu.gyro_z` is the world-frame angular velocity of the turret. Their difference is the rate at which the turret rotates relative to the chassis. This is integrated each IMU tick to produce `integrated_turret_position`, which is published as the `gimbal_joint` position.

**Published joint names:** `gimbal_joint`, `turret_shaft_joint`, `front_left_wheel_joint`, `front_right_wheel_joint`, `back_left_wheel_joint`, `back_right_wheel_joint`.

The pitch joint (`turret_shaft_joint`) position is set to `-turret_data[2]` (negated pitch position from MCB). Wheel joint positions are always zero (not tracked).

---

### `real_robot.launch.py`
`src/sentry_bringup/launch/real_robot.launch.py`

The single entry point for bringing up the full robot on hardware. All subsystems are controlled by launch arguments:

| Argument | Default | Description |
|---|---|---|
| `slam` | `true` | Use SLAM Toolbox for mapping. Set `false` to use AMCL with a pre-built map. |
| `map` | `sentry_map.yaml` | Map file to load when `slam:=false`. Must be in `sentry_localization/maps/`. |
| `robot_model` | `sentry_description.urdf.xacro` | URDF file name in `sentry_description/urdf/`. |
| `use_localization` | `true` | Launch SLAM or AMCL. Set `false` to skip map entirely (e.g. odometry-only testing). |
| `use_keepout` | `true` | Load the arena keepout filter mask into Nav2 costmaps. Set `false` outside the arena. |
| `use_nav` | `true` | Launch the Nav2 navigation stack. |
| `display` | `false` | Launch RViz. |

**Node launch order:**
1. `robot_state_publisher` — URDF → TF
2. `comm_hub` — MCB serial bridge (`/odom`, `/imu`, `/turret`, `/cmd_vel`)
3. `ldlidar_ros2_node` ×2 — LD19 drivers on `/dev/ttyUSB0` and `/dev/ttyUSB1`
4. `laser_merger` — merges scans into `/scan`
5. `teleop_twist_keyboard` — opens in a new `xterm` window
6. `global_localization` (sub-launch) — SLAM or AMCL
7. `local_localization` (sub-launch) — EKF + turret JSP + IMU compensator
8. `navigation` (sub-launch) — Nav2 stack
9. `rviz2` (if `display:=true`)

Note: `joint_state_publisher` is declared but commented out of the final `LaunchDescription`. The turret JSP fulfills this role on hardware.

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

**Critical gimbal design:** `gimbal_joint` has `axis = [0,0,-1]` (negative Z). Combined with the turret JSP publishing `-turretYawPos`, a CCW chassis spin (which decreases `turretYawPos` in the MCB's reference) produces an increasing joint value → the gimbal rotates CW relative to the chassis → the turret stays stationary in the world. This sign convention is the load-bearing mechanism for counter-rotation.

**LiDAR placement:** Both LiDARs are children of the turret structure (attached to left/right support standoffs), so they rotate with the turret, not the chassis. The laser_merger deskews for this.

**IMU:** Attached to `turret_link` via a fixed joint. The `imu_turret_compensator` node corrects for turret motion before passing IMU data to the EKF.

**Gazebo:** Uses `libgazebo_ros_planar_move.so` for omnidirectional motion simulation and `libgazebo_ros_joint_state_publisher.so` for joint states.

---

### `ekf.yaml`
`src/sentry_localization/config/ekf.yaml`

Configures `robot_localization`'s `ekf_node` running at 40 Hz in 2D mode.

**Inputs fused:**

- **`odom0: /odom`** — from `comm_hub`. Fuses:
  - `vx`, `vy` (linear body velocity, indices [6,7] in the 15-element config vector)
  - `vyaw` (angular velocity about Z, index [11])
  - Pose Z-yaw (`[5]`) is also fused.

- **`imu0: /imu_base_compensated`** — from `imu_turret_compensator`. Fuses only:
  - `vyaw` (gyro Z, index [11])

The IMU orientation, linear acceleration, and all other fields are disabled. Only the compensated gyroscope yaw rate is used to reinforce the wheel odometry yaw estimate.

**On real hardware:** `ekf_node_real` uses these parameters directly. The `ekf_node` (sim) variant overrides `imu0` to use `/imu_comm_hub` (produced by `sim_comm_hub_bridge`) and disables several fields.

`publish_tf: true` in the YAML but the real-robot launch overrides it to `false` in the node parameters — the `odom → base_link` TF is published by `comm_hub`'s odometry implicitly through `robot_localization`. Check `local_localization.launch.py` if the TF is missing at runtime.

---

### `laser_merger.py`
`src/sentry_bringup/scripts/laser_merger.py`

Subscribes to `/scan_left` and `/scan_right` from the two LD19s and publishes a single merged `/scan` in `base_link` frame at 20 Hz. The non-trivial part is deskewing — a LiDAR spinning at 10 Hz takes ~100 ms to complete one revolution, during which the robot (and the turret the LiDARs are mounted on) has moved. Without correction, each ray in the scan was captured from a slightly different physical position, which smears obstacles in the merged cloud and corrupts SLAM.

**Deskewing method**

For each incoming scan the node computes a transform for the first ray (at `t_start`) and a transform for the last ray (at `t_end = scan.header.stamp`), then linearly interpolates across all rays in between:

```
t_start = t_end - (N - 1) * scan.time_increment
```

Each ray's transform is looked up via `lookup_transform_full` — the 4-argument "bridge-frame" variant of the TF API:

```python
tf_buffer.lookup_transform_full(
    target_frame,  rclpy.time.Time(),   # where we want the point (latest EKF time)
    source_frame,  ray_capture_time,    # where the sensor was at capture
    fixed_frame,                        # odom — the bridge
    timeout
)
```

Using `odom` as the bridge frame means the transform accounts for **both** turret rotation relative to the chassis (via the URDF/TF chain through `gimbal_joint`) and robot body translation/rotation during the scan sweep (via the EKF-published `odom → base_link` TF). This is the most complete form of scan deskewing available without a dedicated motion-distortion solver.

*Note: other solutions might be possible but have not yet been explored.

**Three fallback modes** (tried in order, logged as warnings when degraded):

| Mode | How | Corrects |
|---|---|---|
| `full` | `lookup_transform_full` via odom bridge | Turret rotation + body motion |
| `turret` | `lookup_transform` at exact ray timestamp | Turret rotation only |
| `latest` | `lookup_transform` at time=0 (newest available) | Nothing — single static transform for whole scan |

**LD19 scan direction note:** The LD19 scans counter-clockwise. With `laser_scan_dir: True` set in the driver, the range array is mirrored so that index 0 corresponds to the physically last-captured ray (at `t_end`) and index `N-1` to the first (at `t_start`). The interpolation alpha is therefore `1.0 - i / (N-1)` — the first index in the array gets the `t_end` transform, not `t_start`.

**Merged scan stamping:** Rather than stamping the output with `now()`, the node probes the TF buffer for the latest available EKF transform and uses that timestamp. This ensures SLAM's message filter can immediately look up the corresponding `odom → base_link` TF without waiting for extrapolation into the future, which was the cause of earlier queue-full warnings and the map appearing to drift backwards.

All transformed points are binned into a fixed angular grid (`angle_min=-π`, `angle_max=π`, `angle_increment=0.01 rad`) with nearest-range-wins per bin, then published as a standard `LaserScan`.

---

## Setup Instructions

### Startup Up the Physical Robot
1. Unplug the chassis power cable from the referee serial module
2. Insert a battery into the battery holder. Double click the power button and hold for 3 seconds on the second click to activate the battery
3. Flick the switch from 'O' to 'I' to activate power distribution
4. While the robot powers on, hold the turret level so the turret-mounted IMU calibrates properly
5. Once you hear some startup noise, see lights flashing, and the serial module shows 3 green lights going to the chassis, gimbal, and ammo booster, replug the chassis power cable into the serial module
6. Turn the remote on. If the remote is paired, it will auto connect. If not, look under the back-left side of the turret for a white/silver square. With a pen, push and hold the central-most button until lights start flashing. Your controller should then pair with the white receiver that is now in pairing mode.
7. Drive the robot. Flick the left switch down to accept autonav commands. If the left switch is neutral, the robot will only accept controller commands. This is a safety feature

Troubleshooting: 
- If the chassis light turns off after reinserting cable, a: Remove the jetson power cable, turn the power distribution off and on again. Try the steps above again. Replug the jetson once everything works. b: keep the hardware setup the same, but turn on the power distribution with the chassis power cable already plugged in. Then, pray. 
- If at any point the turret starts spinning erratically or you can spin it way faster than it should allow, it means the imu lost calibration. Restart from step 1 and power cycle the robot
- If ssh drops or disconnects at any point, unplug and replug the router in the room

The jetson connects to to the **QKRT-5G** or the **QKRT** wifi if 5G doesn't work. Log in to the wifi. Password: goodlife

ssh into the jetson with: 

```bash 
ssh -X qkrt@192.168.8.102 #current password: qkrt123  
cd qkrt-nav
```
qkrt-nav has already been cloned on our jetson, so skip to **Build** section. If not, read below.


### Hardware Requirements

- Mini pc: Nvidia Jetson, flashed with ubuntu 22.04.5 LTS  
- MCB firmware version / branch: feat/auto_nav_comms_working 
- LiDAR serial device assignment: /dev/ttyUSB0 (left?) and /dev/ttyUSB1 (right?)
- How to determine which USB port is which LiDAR: ```ls /dev/ttyUSB* ```

### Dependency Installation

```bash
# Clone the LDLiDAR driver into src/ if not already present
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git src/ldlidar_ros2

# Install all ROS and system dependencies declared in every package.xml
sudo rosdep init   # skip if already initialized
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# pyserial is not in rosdep — install manually
pip3 install pyserial
```

### Build

```bash
cd qkrt-nav
colcon build --symlink-install
source install/setup.bash #in another terminal for good practice
```

### Udev / Permissions
**Do this if not already done**

All three serial devices (`/dev/ttyTHS1`, `/dev/ttyUSB0`, `/dev/ttyUSB1`) are owned by the `dialout` group. Add your user to it once and re-login:

```bash
sudo usermod -aG dialout $USER
# then log out and back in — or run: newgrp dialout
```

Verify you can open the devices without sudo:

```bash
ls -la /dev/ttyTHS1 /dev/ttyUSB0 /dev/ttyUSB1
# should show group = dialout and your user in that group
```

**Locking in which LiDAR is left vs right**

Both LD19s are identical hardware so the OS assigns `ttyUSB0`/`ttyUSB1` by whichever enumerates first at boot — this can flip. Fix it by creating udev symlinks tied to physical USB port:

```bash
# Plug in one LiDAR at a time and run this to find its port path
udevadm info -a /dev/ttyUSB0 | grep 'KERNELS==' | head -3
# Look for a value like: KERNELS=="1-2.3.1"
```

Create the rules file:

```bash
sudo nano /etc/udev/rules.d/99-sentry-lidar.rules
```

Paste (substituting your actual port paths):

```
SUBSYSTEM=="tty", KERNELS=="<usb-path-of-left-lidar>",  SYMLINK+="lidar_left"
SUBSYSTEM=="tty", KERNELS=="<usb-path-of-right-lidar>", SYMLINK+="lidar_right"
```

Reload and replug:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Then update `real_robot.launch.py` to use `/dev/lidar_left` and `/dev/lidar_right` instead of `ttyUSB0`/`ttyUSB1`.

### Running on Hardware
On a local machine (not the jetson), set your **ROS_DOMAIN_ID** to match the jetson's domain id.

```export ROS_DOMAIN_ID=67``` --- current domain ID is 67

Alternatively, run ```nano ~/.bashrc```, paste the above line at the bottom of the script, Ctrl+O, Ctrl+X, and run ```source ~/.bashrc``` to set the domain id every time a new terminal is opened.

Navigate to ```/src/sentry_description/rviz/sentry_config.rviz``` on your local machine, and run 

```bash
rviz2 -d /local-path-to-sentry_config.rviz
```

This opens rviz with the correct configuration on your local machine, which subscribes to the jetson's topics that are published over the same ROS_DOMAIN_ID. The reason for doing this is to run rviz with high performance. Running rviz directly off of the jetson is too slow to see continuous updates.

On the jetson, launch the full stack with one of the following commands:


``` 
# Full bringup with SLAM
ros2 launch sentry_bringup real_robot.launch.py

# With pre-built map (AMCL)
ros2 launch sentry_bringup real_robot.launch.py slam:=false map:=sentry_map.yaml

# Without Nav2 (odometry + sensor testing only)
ros2 launch sentry_bringup real_robot.launch.py use_nav:=false use_localization:=false

# Simulation
ros2 launch sentry_bringup simulated_robot.launch.py
```

---

## Known Bugs

**IN THE CURRENT VERSION, NO BASE ROTATION IS ACCOUNTED FOR. BEYBLADING DOES NOT WORK, AND NEITHER DOES SMALL ROTATIONS**

1. **Odometry drift on pure rotation ("beyblading")** — The mecanum forward kinematics formula gives reasonable translation estimates but accumulates yaw error during sustained spinning. The `GEAR_RATIO` of 9.75 was empirically tuned for straight-line travel; rotational calibration was not separately verified. The misalignment in actual chassis orientation vs. rviz chassis orientation causes improper odometry. If the robot moves forward in real life, the odometry calculation move the robot forward in whatever direction the chassis faces in rviz, causing the robot to move at an angle in rviz, relative to where it moved in real life. This severly degrades localization performance.

2. **Odometry drift on pure translation** — This works fine now, but in case it doesn't: The `GEAR_RATIO` of 9.75 was empirically tuned for straight-line travel. However, sometimes during translation, lidar scans physically drift and smear. The current perceived solution is to tune the odometry better

3. **Lidar Deskewing failing at high enough rotational velocities** — At a high enough turn speed, lidar scans can be rotationally distorted causing slam to generate new walls where there shouldn't be. The believed cause is linear interpolation approximations failing at high speeds, although this has yet to be verified.

4. **LiDAR deskewing falls back to `latest` mode** — When the EKF TF is not available at the exact scan timestamp, `laser_merger.py` falls back to `turret` or `latest` mode and logs a warning. In `latest` mode there is no per-ray deskewing, which can smear the scan during fast motion.

5. **Fixed yaw misalignment between chassis and turret** - on launch, the turret can have a fixed misalignment from the chassis. To fix, align the turret parallel to the mechanum wheels. In the urdf, navigate to the **gimbal_joint** section. Look for the line ```<origin xyz="0 0 ${base_height + ground_offset + gimbal_height/2}" rpy="0 0 ${1.5708}"/>```. Note that the current offsest is pi/2 radians (1.5708). tweak the yaw (the third number in the rpy section) offset, until the turret lines up parallel to the wheels in rviz. Common values that have previously worked are **0, 1.5708, ${1.5708 + 1.05/2}, and ${1.05/2}**, but tweak this until it feels right. To ensure correct orientation, start the robot up, align the turret parallel to the mechanum wheels, push it forward, run ```ros2 topic echo /odom --once```, and verify the robot moved in the +x direction. If it moved backwards in rviz, invert the offset with a negative sign. The actual odometry code is correct, the axes just need to be aligned properly.

Believed cause: the encoder yaw value doesn't reset (doesn't re-initialize to 0) when ros2 starts and stops and when the robot is turned on and off. The encoder yaw value is always fixed, except in some circumstances that have been noted below:

a. the encoder value is recalibrated using mcb code
b. the encoder yaw angle can sometimems reset seemingly randomly


6. **No Transform Between `odom-->base_link`**: 
      a. The mcb is not sending commands - verify with ```ros2 topic echo /odom --once ```
      b. `comm_hub.py` is not receiving mcb commands - verify with ```ros2 topic echo /turret```
      c. `turret_joint_state_publisher` failed
      d. `ekf.yaml` has an issue

7. **`publish_tf` conflict** — `ekf.yaml` sets `publish_tf: true` but `local_localization.launch.py` overrides it with `False` for the real robot. If the launch parameter is accidentally removed, the EKF will publish a duplicate `odom → base_link` TF that conflicts with `robot_state_publisher`.

8. **No `map-->odom` transform**: either a. the lidar drivers are silently failing, b. slam cannot generate a map, or c. laser_merger.py is failing

9. **Time delay between scans being published and being deskewed** - sometimes, there is a noticeable delay between rotating the turret in real life and the deskewing process. A lot of distortion and smearing will be likely visible here. Cause is unknown.  

10. **Turret position drifts** - imu is drifting. Go to `comm_hub.py`. Reset the `IMU_BIAS` variable to 0, and see if that fixes the issue. If not, keep a bias of 0, and do ```ros2 topic echo /imu --field angular_velocity.z```. Average the values printed from there, and `IMU_BIAS` equal to that.

---

## To Be Completed

### Critical

- **ONLY IF ROTATION IS NECESSARY: Fix base rotation → turret drift bug** — The core localization correctness issue. A rewrite of `turret_joint_state_publisher` is necessary. If and only if we have an accurate IMU, we need: 
      a. turret velocity in js_msg should only be gyro_z from IMU. position should be `current_pos += (chassis_speed * (tnow - t_last_odom) - gyro_z * (tnow - tlast_message))` The second term is the turret's own position. First term is to counteract the chassis new orientation after beyblading. Since the chassis is `base_link`, changing the chassis orientation also changes the turret orientation, so we must counteract this and then apply the turret's own movement on top. This is where the most issues happen.
      b. Chassis angular velocity can be calculated by something like `turret[1] - gyro_z`. Remember, `turret[1] = chassis speed + turret speed from imu. chassis speed therefore = turret[1] - gyroz`. Then, `pose.pose.orientation.w`, the bases actual orientation that gets publishes should be `chassis_speed * (tnow-tlast_odom)`. These edits should be made in comm_hub.py using the values directly communicated by mcb, in other words, using the values[] array. Look at the comments in `comm_hub.py` to determine the indicies of gimbal encoder angular velocity and imu angular velocity. 
      c. Test in rviz that the turret stays still during beyblading 


- **End-to-end autonomous navigation working** - get the robot through walls, around obstacles, without breaking down, reliably

- **Behaviour Tree Creation and Tuning** - create a working behaviour tree and tune it to work between two arbitrary points

- **if time permits: Test AutoNav and AutoAim Minimum Viable Product** - Autonav commands send the robot to the middle, robot camps in mid. No Nav commands sent. Auto Aim activates, and shoots other Robots. Autonav starts resending nav commands when health/battery/balls are low.

- **Write Autonav/Aim startup script** - write a script that launches autonav and autoaim on jetson startup

- Sentry_vision integration if time permits

### Should Have

- **Separate odometry and IMU covariance tuning** — The current covariance values in `comm_hub.py` (`0.01` across the board) and `ekf.yaml` are placeholder estimates. These should be tuned with actual sensor noise characterization (Allan variance for IMU, wheel-slip analysis for odometry).

- **Confirm LiDAR USB assignments** — Document and enforce which physical LiDAR is `ttyUSB0` (left) and which is `ttyUSB1` (right). A swap causes both scans to report from the wrong TF frame, breaking the merged scan and all localization that depends on it.

- **Better turret scan deskewing when turret rotates** — The current laser_merger assumes the LiDAR frames move rigidly with the turret as tracked by TF. Verify that SLAM/AMCL receives correct scan data when the turret is actively rotating relative to the chassis during navigation. 

- **Map saving workflow** — After running SLAM, the resulting map must be saved with `map_saver_cli`. Document the exact command and where to put the output files for AMCL mode. The current `sentry_map.yaml` was generated manually, using the competition map in simulation.

- **`comm_hub` reconnect on serial error** — If the MCB reboots or the UART disconnects, `comm_hub` crashes. It should catch `SerialException` and attempt reconnection rather than killing the whole process.

- **Remove or fix legacy `mcb_message.py`** — The file exists in the package but is no longer imported. Either delete it or consolidate with `communication/Message.py`.

### Nice to Have

- **Add `imu_link` to EKF orientation fusion** — The EKF currently only uses gyro Z from the compensated IMU. Enabling the full orientation quaternion (once the turret compensation is verified correct) would significantly improve yaw accuracy during fast maneuvers.

- **Further Depth camera integration** — The URDF and Gazebo plugin for the depth camera are defined but the camera topic is not used anywhere in the navigation or localization stack. Point cloud to laserscan conversion and obstacle layer integration would improve close-range obstacle detection.

- **Keepout zone from SDF generation script** — `generate_keepout_from_sdf.py` automates keepout mask creation. Integrate this into the build or map-generation workflow so the keepout mask stays in sync with the world file.

- **CI / automated smoke test** — A `colcon test` target that launches the simulated robot, sends a goal, and checks that `/odometry/filtered` is being published would catch regressions from future changes.

- **Fix Turret pitch control** — Pitch position is read from the MCB and published in `/joint_states` as `turret_shaft_joint`. The signs on `turret_data[2]` and `turret_data[3]` may be inverted.

- **Test/Attempt to use the new Sentry_Vision package** - Test cv localization in tandem with the other forms of localization
