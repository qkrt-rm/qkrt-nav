## Current structure:

sentry_bringup - launches everything (real or sim)
sentry_description - URDF/meshes/worlds
sentry_localization - EKF + AMCL + maps
sentry_navigation - planners/controllers/smoother/BT

## to be added:
implement bt
implement lidar
sentry_drivers or sentry_firmware - get serial from robot and publish it
sentry_core - competition logic (needed for bt)
robot radius in planner server




