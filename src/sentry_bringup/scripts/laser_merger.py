#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
import numpy as np
import math


class LaserMerger(Node):
    def __init__(self):
        super().__init__('laser_merger')

        # Parameters
        self.declare_parameter('destination_frame', 'base_link')
        self.declare_parameter('scan_destination_topic', '/scan')
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', 0.01)
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 10.0)

        self.destination_frame = self.get_parameter('destination_frame').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Storage for latest scans
        self.scan_left = None
        self.scan_right = None

        # Subscribers
        self.sub_left = self.create_subscription(
            LaserScan, '/scan_left', self.scan_left_callback, qos_profile_sensor_data)
        self.sub_right = self.create_subscription(
            LaserScan, '/scan_right', self.scan_right_callback, qos_profile_sensor_data)

        # Publisher
        scan_topic = self.get_parameter('scan_destination_topic').value
        self.pub_scan = self.create_publisher(LaserScan, scan_topic, qos_profile_sensor_data)

        # Timer to publish merged scan
        self.create_timer(0.05, self.publish_merged_scan)  # 20 Hz

        self.get_logger().info(f'Laser merger started, publishing to {scan_topic}')

    def scan_left_callback(self, msg):
        self.scan_left = msg

    def scan_right_callback(self, msg):
        self.scan_right = msg

    def transform_scan_to_frame(self, scan, target_frame):
        """Transform a laser scan to the target frame and return points as (angle, range) in target frame."""
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                scan.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn(f'Could not transform from {scan.header.frame_id} to {target_frame}: {e}')
            return []

        # Extract translation and rotation
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        q = transform.transform.rotation
        # Convert quaternion to yaw (assuming 2D)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        points = []
        angle = scan.angle_min
        for r in scan.ranges:
            if self.range_min <= r <= self.range_max:
                # Point in scan frame
                x_scan = r * math.cos(angle)
                y_scan = r * math.sin(angle)

                # Transform to target frame
                x_target = math.cos(yaw) * x_scan - math.sin(yaw) * y_scan + tx
                y_target = math.sin(yaw) * x_scan + math.cos(yaw) * y_scan + ty

                # Convert back to polar
                r_target = math.sqrt(x_target**2 + y_target**2)
                angle_target = math.atan2(y_target, x_target)

                if self.range_min <= r_target <= self.range_max:
                    points.append((angle_target, r_target))

            angle += scan.angle_increment

        return points

    def publish_merged_scan(self):
        if self.scan_left is None and self.scan_right is None:
            return

        # Collect all points from both scans
        all_points = []

        if self.scan_left is not None:
            points = self.transform_scan_to_frame(self.scan_left, self.destination_frame)
            all_points.extend(points)

        if self.scan_right is not None:
            points = self.transform_scan_to_frame(self.scan_right, self.destination_frame)
            all_points.extend(points)

        if not all_points:
            return

        # Create merged scan
        num_readings = int((self.angle_max - self.angle_min) / self.angle_increment)
        ranges = [float('inf')] * num_readings

        # Fill in ranges (keep minimum range for each angle bin)
        for angle, r in all_points:
            if self.angle_min <= angle <= self.angle_max:
                idx = int((angle - self.angle_min) / self.angle_increment)
                if 0 <= idx < num_readings:
                    ranges[idx] = min(ranges[idx], r)

        # Build message
        merged = LaserScan()
        merged.header.stamp = self.get_clock().now().to_msg()
        merged.header.frame_id = self.destination_frame
        merged.angle_min = self.angle_min
        merged.angle_max = self.angle_max
        merged.angle_increment = self.angle_increment
        merged.time_increment = 0.0
        merged.scan_time = 0.05
        merged.range_min = self.range_min
        merged.range_max = self.range_max
        merged.ranges = ranges
        merged.intensities = []

        self.pub_scan.publish(merged)


def main(args=None):
    rclpy.init(args=args)
    node = LaserMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
