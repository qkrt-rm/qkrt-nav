import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pupil_apriltags
import numpy as np


class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')

        self.bridge = CvBridge()

        self.get_logger().info('Initializing AprilTag Detector (tag36h11)...')
        self.detector = pupil_apriltags.Detector(families='tag36h11')

        # Allow image topic to be overridden via parameter
        self.declare_parameter('image_topic', '/sentry/depth_camera/image_raw')
        topic_name = self.get_parameter('image_topic').get_parameter_value().string_value

        self.get_logger().info(f'Subscribing to: {topic_name}')

        # Use sensor data QoS to match Gazebo's "best effort" publisher
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.listener_callback,
            qos_profile_sensor_data)

        self.debug_publisher = self.create_publisher(Image, '/sentry_vision/debug_image', 10)

        self.get_logger().info('AprilTag Detector Node Started. Waiting for images...')

    def listener_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            detections = self.detector.detect(gray)

            if detections:
                self.get_logger().info(f'Detected {len(detections)} tag(s)')

                for tag in detections:
                    # Draw bounding box
                    pts = tag.corners.reshape((-1, 1, 2)).astype(int)
                    cv2.polylines(frame, [pts], True, (0, 255, 0), 3)

                    # Draw center point
                    cX, cY = int(tag.center[0]), int(tag.center[1])
                    cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)

            # Publish annotated frame for visualization in RViz
            debug_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.debug_publisher.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to process image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
