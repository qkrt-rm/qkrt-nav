import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
import cv2
import pupil_apriltags
import numpy as np


class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')

        self.bridge = CvBridge()
        self.camera_params = None

        self.get_logger().info('Initializing AprilTag Detector (tag36h11)...')
        self.detector = pupil_apriltags.Detector(families='tag36h11')

        # Parameters
        self.declare_parameter('image_topic', '/sentry/depth_camera/image_raw')
        self.declare_parameter('camera_info_topic', '/sentry/depth_camera/camera_info')
        self.declare_parameter('tag_size', 0.2)
        self.declare_parameter('tag_poses_file', '')

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.tag_size = self.get_parameter('tag_size').get_parameter_value().double_value

        self.get_logger().info(f'Subscribing to: {image_topic}')
        self.get_logger().info(f'Tag size: {self.tag_size} m')

        # Subscribe to camera info to get intrinsics
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10)

        # Subscribe to image stream (sensor QoS for Gazebo compatibility)
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data)

        # Publishers
        self.debug_publisher = self.create_publisher(Image, '/sentry_vision/debug_image', 10)
        self.detections_publisher = self.create_publisher(PoseArray, '/sentry_vision/tag_detections', 10)

        # TF broadcaster for tag poses
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('AprilTag Detector Node started. Waiting for camera info...')

    def camera_info_callback(self, msg):
        """Extract camera intrinsics from CameraInfo message."""
        if self.camera_params is not None:
            return

        # K matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        fx = msg.k[0]
        fy = msg.k[4]
        cx = msg.k[2]
        cy = msg.k[5]

        if fx == 0.0 or fy == 0.0:
            return

        self.camera_params = [fx, fy, cx, cy]
        self.get_logger().info(
            f'Camera intrinsics received: fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}')

    def image_callback(self, msg):
        """Process incoming image: detect tags, estimate pose, publish results."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Run detection with or without pose estimation
            if self.camera_params is not None:
                detections = self.detector.detect(
                    gray,
                    estimate_tag_pose=True,
                    camera_params=self.camera_params,
                    tag_size=self.tag_size)
            else:
                detections = self.detector.detect(gray)

            pose_array = PoseArray()
            pose_array.header.stamp = msg.header.stamp
            pose_array.header.frame_id = msg.header.frame_id

            if detections:
                self.get_logger().info(f'Detected {len(detections)} tag(s)')

                for tag in detections:
                    self._draw_detection(frame, tag)

                    if self.camera_params is not None and tag.pose_R is not None:
                        pose = self._build_pose(tag)
                        pose_array.poses.append(pose)
                        self._broadcast_tf(tag, pose, msg.header)

            self.detections_publisher.publish(pose_array)

            # Publish annotated frame for visualization in RViz
            debug_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.debug_publisher.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to process image: {str(e)}')

    def _draw_detection(self, frame, tag):
        """Draw bounding box, center, tag ID, and distance on the frame."""
        # Bounding box
        pts = tag.corners.reshape((-1, 1, 2)).astype(int)
        cv2.polylines(frame, [pts], True, (0, 255, 0), 3)

        # Center point
        cX, cY = int(tag.center[0]), int(tag.center[1])
        cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)

        # Tag ID label
        label = f'ID: {tag.tag_id}'

        # Add distance if pose estimation is available
        if tag.pose_t is not None:
            distance = float(tag.pose_t[2])
            label += f' d={distance:.2f}m'

        cv2.putText(frame, label, (cX - 30, cY - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    def _build_pose(self, tag):
        """Convert tag detection to a geometry_msgs/Pose."""
        pose = Pose()

        pose.position.x = float(tag.pose_t[0])
        pose.position.y = float(tag.pose_t[1])
        pose.position.z = float(tag.pose_t[2])

        # Convert rotation matrix to quaternion
        quat = self._rotation_matrix_to_quaternion(tag.pose_R)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        return pose

    @staticmethod
    def _rotation_matrix_to_quaternion(R):
        """Convert a 3x3 rotation matrix to quaternion [x, y, z, w]."""
        trace = np.trace(R)

        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s

        return [x, y, z, w]

    def _broadcast_tf(self, tag, pose, header):
        """Broadcast tag pose as a TF transform from camera frame to tag frame."""
        t = TransformStamped()
        t.header.stamp = header.stamp
        t.header.frame_id = header.frame_id
        t.child_frame_id = f'tag_{tag.tag_id}'

        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
