# Sentry Vision

Handles computer vision tasks for the Sentry Robot. Currently implements the AprilTag Detection and Localization pipeline.

## Nodes

### 1. `apriltag_detector`
Detects AprilTags (family `tag36h11`), estimates their pose relative to the camera, and broadcasts TF transforms.

**Interface**
| Type | Topic | Message Type | Description |
| :--- | :--- | :--- | :--- |
| **Sub** | `/sentry/depth_camera/image_raw` | `sensor_msgs/Image` | Raw RGB stream. Expects standard ROS camera encoding. |
| **Sub** | `/sentry/depth_camera/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics. |
| **Pub** | `/sentry_vision/debug_image` | `sensor_msgs/Image` | Visualization with bounding boxes and distances. |
| **Pub** | `/sentry_vision/tag_detections` | `geometry_msgs/PoseArray` | Array of detected tag poses in the camera frame. |
| **TF**  | `/tf` | `tf2_msgs/TFMessage` | Broadcasts transform from camera frame to tag frame (`tag_<id>`). |

### 2. `apriltag_localizer`
Converts AprilTag detections into robot global pose estimates using known map coordinates.

**Interface**
| Type | Topic | Message Type | Description |
| :--- | :--- | :--- | :--- |
| **Sub** | `/sentry_vision/tag_detections` | `geometry_msgs/PoseArray` | Detections from the detector node. |
| **Pub** | `/apriltag_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Estimated robot pose in the map frame. |

## Dependencies (Critical)
* **`pupil-apriltags`**: Required for detection logic.
* **`numpy < 2.0`**: **Strict Requirement.** ROS 2 Humble is incompatible with NumPy 2.x.

## Quick Start
```bash
# 1. Install specific dependencies (Fixes cv_bridge conflict)
pip install pupil-apriltags "numpy<2"

# 2. Launch the vision pipeline (Detector + Localizer)
ros2 launch sentry_vision vision.launch.py use_sim_time:=true
```
