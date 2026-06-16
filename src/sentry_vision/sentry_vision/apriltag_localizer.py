"""
AprilTag Localizer Node.

Converts AprilTag detections into robot global pose estimates.

Pipeline:
  1. Loads known tag world poses from tag_poses.yaml
  2. Listens for TF transforms broadcast by the detector (camera → tag)
  3. Computes: robot_in_map = tag_in_map × inverse(tag_in_camera) × inverse(camera_in_base)
  4. Publishes robot pose as PoseWithCovarianceStamped on /apriltag_pose

This publishes to a SEPARATE topic so it doesn't interfere with existing
AMCL/EKF localization. It can be integrated into the EKF later by adding
it as a pose sensor source.
"""

import yaml
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
import tf2_ros
from tf2_ros import TransformException


class AprilTagLocalizer(Node):
    def __init__(self):
        super().__init__('apriltag_localizer')

        # Parameters
        self.declare_parameter('tag_poses_file', '')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'depth_camera_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('publish_rate_limit', 10.0)  # Max Hz

        tag_poses_file = self.get_parameter('tag_poses_file').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.rate_limit = (
            self.get_parameter('publish_rate_limit').get_parameter_value().double_value
        )

        # Load known tag poses
        self.known_tags = {}
        if tag_poses_file:
            self._load_tag_poses(tag_poses_file)
        else:
            self.get_logger().warn(
                'No tag_poses_file specified. Localizer will not compute global poses.')

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribe to tag detections from detector
        self.detections_sub = self.create_subscription(
            PoseArray,
            '/sentry_vision/tag_detections',
            self.detections_callback,
            10
        )

        # Publish estimated robot pose (separate topic, non-invasive)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/apriltag_pose',
            10
        )

        # Rate limiting
        self.min_interval = 1.0 / self.rate_limit if self.rate_limit > 0 else 0.0
        self.last_publish_time = self.get_clock().now()

        self.get_logger().info(
            f'AprilTag Localizer started.\n'
            f'  Known tags: {list(self.known_tags.keys())}\n'
            f'  Camera frame: {self.camera_frame}\n'
            f'  Base frame: {self.base_frame}\n'
            f'  Publishing on: /apriltag_pose'
        )

    def _load_tag_poses(self, filepath):
        """Load known tag world poses from YAML file."""
        try:
            with open(filepath, 'r') as f:
                config = yaml.safe_load(f)

            for tag_id, tag_data in config.get('tags', {}).items():
                pos = tag_data['position']
                ori = tag_data['orientation']
                self.known_tags[int(tag_id)] = {
                    'position': np.array(pos, dtype=float),
                    'orientation': np.array(ori, dtype=float),  # roll, pitch, yaw
                    'size': tag_data.get('size', 0.2),
                    'transform': self._pose_to_matrix(pos, ori),
                }
                self.get_logger().info(
                    f'Loaded tag {tag_id}: pos={pos}, ori={ori}'
                )

        except Exception as e:
            self.get_logger().error(f'Failed to load tag poses from {filepath}: {e}')

    def detections_callback(self, msg: PoseArray):
        """When tags are detected, look up TF and compute robot global pose."""
        if not self.known_tags:
            return

        # Rate limiting
        now = self.get_clock().now()
        elapsed = (now - self.last_publish_time).nanoseconds / 1e9
        if elapsed < self.min_interval:
            return

        # Try to get robot pose from each detected tag
        for tag_id in self.known_tags:
            tag_frame = f'tag_{tag_id}'

            try:
                # Look up camera → tag transform (broadcast by detector)
                cam_to_tag = self.tf_buffer.lookup_transform(
                    self.camera_frame,
                    tag_frame,
                    rclpy.time.Time(),  # latest available
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )

                # Look up base_link → camera transform (from URDF/robot_state_publisher)
                base_to_cam = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.camera_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )

            except TransformException as e:
                self.get_logger().debug(f'TF lookup failed for tag {tag_id}: {e}')
                continue

            # Compute robot pose in map frame
            robot_pose = self._compute_robot_pose(tag_id, cam_to_tag, base_to_cam)
            if robot_pose is not None:
                self.pose_pub.publish(robot_pose)
                self.last_publish_time = now
                self.get_logger().info(
                    f'Published robot pose from tag {tag_id}: '
                    f'x={robot_pose.pose.pose.position.x:.2f}, '
                    f'y={robot_pose.pose.pose.position.y:.2f}'
                )
                return  # One good estimate per callback is enough

    def _compute_robot_pose(self, tag_id, cam_to_tag, base_to_cam):
        """
        Compute robot pose in map frame.

        Using:
          robot_in_map = tag_in_map × inverse(tag_in_camera) × inverse(camera_in_base)

        Which simplifies to:
          robot_in_map = tag_in_map × inverse(cam_to_tag) × inverse(base_to_cam)

        More intuitively:
          - We know where the tag is in the world (tag_in_map)
          - We know where the tag is relative to camera (cam_to_tag)
          - We know where the camera is relative to base_link (base_to_cam)
          - So we can solve for where base_link is in the world
        """
        try:
            tag_in_map = self.known_tags[tag_id]['transform']
            cam_to_tag_mat = self._transform_to_matrix(cam_to_tag.transform)
            base_to_cam_mat = self._transform_to_matrix(base_to_cam.transform)

            # robot_in_map = tag_in_map × inv(cam_to_tag) × inv(base_to_cam)
            robot_in_map = (
                tag_in_map @ np.linalg.inv(cam_to_tag_mat) @ np.linalg.inv(base_to_cam_mat)
            )

            # Extract position and orientation
            position = robot_in_map[:3, 3]
            quaternion = self._rotation_matrix_to_quaternion(robot_in_map[:3, :3])

            # Build PoseWithCovarianceStamped message
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.map_frame

            pose_msg.pose.pose.position.x = float(position[0])
            pose_msg.pose.pose.position.y = float(position[1])
            pose_msg.pose.pose.position.z = float(position[2])

            pose_msg.pose.pose.orientation.x = float(quaternion[0])
            pose_msg.pose.pose.orientation.y = float(quaternion[1])
            pose_msg.pose.pose.orientation.z = float(quaternion[2])
            pose_msg.pose.pose.orientation.w = float(quaternion[3])

            # Covariance: higher uncertainty for tags detected at longer range
            # Using a simple diagonal covariance
            # [x, y, z, rot_x, rot_y, rot_z] - 6x6 matrix flattened to 36 elements
            covariance = [0.0] * 36
            covariance[0] = 0.05    # x variance
            covariance[7] = 0.05    # y variance
            covariance[14] = 0.1    # z variance (less certain)
            covariance[21] = 0.1    # rot_x variance
            covariance[28] = 0.1    # rot_y variance
            covariance[35] = 0.05   # rot_z (yaw) variance
            pose_msg.pose.covariance = covariance

            return pose_msg

        except Exception as e:
            self.get_logger().error(f'Failed to compute robot pose from tag {tag_id}: {e}')
            return None

    @staticmethod
    def _pose_to_matrix(position, orientation_rpy):
        """Convert position [x,y,z] and orientation [roll,pitch,yaw] to 4x4 transform matrix."""
        x, y, z = position
        roll, pitch, yaw = orientation_rpy

        # Rotation matrices
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr],
        ])

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    @staticmethod
    def _transform_to_matrix(transform):
        """Convert a geometry_msgs/Transform to a 4x4 matrix."""
        t = transform.translation
        q = transform.rotation

        # Quaternion to rotation matrix
        x, y, z, w = q.x, q.y, q.z, q.w

        R = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
        ])

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    @staticmethod
    def _rotation_matrix_to_quaternion(R):
        """Convert 3x3 rotation matrix to quaternion [x, y, z, w]."""
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


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
