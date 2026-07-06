import os
import sys
import types

# apt opencv 4.5.4 defaults to a GStreamer pipeline that fails to open the
# OV9782 (reports 0x0); force the V4L2 backend instead.
os.environ.setdefault('OPENCV_VIDEOIO_PRIORITY_GSTREAMER', '0')

# 1. POINT DIRECTLY TO THE INNER SRC DIRECTORY
# This allows Python to see 'camera', 'detector', 'pose_estimator', etc.
sys.path.append('/home/qkrt/qkrt-aim/src')

# Compatibility shims: qkrt-aim targets Python 3.12 with opencv>=4.8 and
# line_profiler; ROS Humble runs Python 3.10 with apt opencv 4.5.4 and
# neither is available for 3.10.
import cv2
import numpy as np
if not hasattr(cv2, 'typing'):
    _cv2_typing = types.ModuleType('cv2.typing')
    _cv2_typing.MatLike = np.ndarray
    sys.modules['cv2.typing'] = _cv2_typing
    cv2.typing = _cv2_typing
try:
    import line_profiler  # noqa: F401
except ImportError:
    _lp = types.ModuleType('line_profiler')
    _lp.profile = lambda func: func
    sys.modules['line_profiler'] = _lp

# Importing util before detector avoids a circular-import failure under
# Python 3.10 (detector -> Target -> util -> ImageLabeller -> detector).
import util  # noqa: F401

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# 2. IMPORT DIRECTLY FROM THE ADDED PATH
from detector import HUSTDetector
from camera.Camera import Camera, OV9782_CONFIG
from pose_estimator.TargetPositionEstimator import TargetPositionEstimator
from rules import CenterTargetRule, TargetSelector

class VisionBridgeNode(Node):
    def __init__(self):
        super().__init__('vision_bridge_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/sentry/aim_target', 10)
       
        # Initialize camera tracking pipelines
        self.camera = Camera(OV9782_CONFIG)
       
        # 3. USE THE CORRECT PATH TO YOUR MODELS & CALIBRATION INSIDE THE REPO
        self.detector = HUSTDetector("/home/qkrt/qkrt-aim/src/detector/models/HUST_model.onnx")
        self.pose_estimator = TargetPositionEstimator("/home/qkrt/qkrt-aim/src/example_camera_calibration.json")
        self.target_selector = TargetSelector([CenterTargetRule(self.camera.width, self.camera.height)])
       
        self.timer = self.create_timer(1.0 / 30.0, self.processing_loop)
        self.get_logger().info("Vision Bridge Node successfully initialized!")

    def processing_loop(self):
        frame = self.camera.getFrame()
        if frame is None:
            self.get_logger().warn("No frame from camera", throttle_duration_sec=5.0)
            return
        targets = self.detector.processInput(frame)
       
        if len(targets) > 0:
            best_target = self.target_selector.getBestTarget(targets)
            _, _, target_position = self.pose_estimator.estimatePosition(best_target)
           
            # Map string to enum integer
            color_map = {"Blue": 0.0, "Red": 1.0, "Neutral": 2.0, "Purple": 3.0}
            color_id = color_map.get(best_target.color, 2.0)

            # Publish variables directly onto the topic line
            msg = Float32MultiArray()
            msg.data = [float(target_position.x), float(target_position.y), float(target_position.z), color_id]
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisionBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()