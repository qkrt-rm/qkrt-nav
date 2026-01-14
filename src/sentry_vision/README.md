# Sentry Vision

Handles computer vision tasks for the Sentry Robot. Currently implements the `apriltag_detector` node.

## Node: `apriltag_detector`

Detects AprilTags (family `tag36h11`) and publishes debug visualization.

### Interface (The Contract)
| Type | Topic | Message Type | Description |
| :--- | :--- | :--- | :--- |
| **Sub** | `/camera/image_raw` | `sensor_msgs/Image` | Raw RGB stream. Expects standard ROS camera encoding. |
| **Pub** | `/sentry_vision/debug_image` | `sensor_msgs/Image` | Visualization with green bounding boxes around tags. |

### Dependencies (Critical)
* **`pupil-apriltags`**: Required for detection logic.
* **`numpy < 2.0`**: **Strict Requirement.** ROS 2 Humble is incompatible with NumPy 2.x.

### Quick Start
```bash
# 1. Install specific dependencies (Fixes cv_bridge conflict)
pip install pupil-apriltags "numpy<2"

# 2. Run Node
ros2 run sentry_vision detector
