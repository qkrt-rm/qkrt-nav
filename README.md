## Current structure

### sentry_bringup
- launches everything (real or sim)

### sentry_description
- URDF, worlds, rviz config

### sentry_localization
- global localization: AMCL + maps (uses lidar and our map)
- local localization: EKF (combines odom and imu readings)

### sentry_navigation
- planner server  
  - Path planning with smac planner
- controller server  
  - DWB control
- smoother server  
  - smooths path so it's not jagged
- behaviour server and behavior_tree  
  - TBD


## To be added

- implement bt  
- implement lidar  

### sentry_firmware
- get imu and odom serial from robot and publish it  
- ino files for the microcontroller too  

### sentry_core
- competition logic (needed for bt, uses battery etc.)

- add robot radius in planner server
