#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
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

        # Publisher - use RELIABLE so SLAM toolbox can subscribe
        scan_topic = self.get_parameter('scan_destination_topic').value
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.pub_scan = self.create_publisher(LaserScan, scan_topic, reliable_qos)

        # Timer to publish merged scan
        self.create_timer(0.05, self.publish_merged_scan)  # 20 Hz

        self.get_logger().info(f'Laser merger started, publishing to {scan_topic}')

    def scan_left_callback(self, msg):
        self.scan_left = msg

    def scan_right_callback(self, msg):
        self.scan_right = msg

    def _extract_2d_transform(self, tf_stamped):
        """Extract (tx, ty, yaw) from a TransformStamped."""
        tx = tf_stamped.transform.translation.x
        ty = tf_stamped.transform.translation.y
        q = tf_stamped.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return tx, ty, math.atan2(siny_cosp, cosy_cosp)

    def transform_scan_to_frame(self, scan, target_frame):
        """Transform a laser scan to the target frame with per-ray motion compensation."""
        # The ldlidar_ros2 driver stamps each scan at publication time (after all rays
        # are collected), so header.stamp is the END of the scan. The first ray was
        # captured at stamp - (n-1)*time_increment.
        n = len(scan.ranges)
        scan_duration = (n - 1) * scan.time_increment if scan.time_increment > 0.0 and n > 1 else 0.0
        t_end = rclpy.time.Time.from_msg(scan.header.stamp)
        t_start = t_end - rclpy.duration.Duration(seconds=scan_duration)

        using_exact_time = True
        try:
            tf_start = self.tf_buffer.lookup_transform(
                target_frame,
                scan.header.frame_id,
                t_start,
                timeout=rclpy.duration.Duration(seconds=0)
            )
        except Exception:
            # TF at scan time unavailable — fall back to latest available transform.
            using_exact_time = False
            try:
                tf_start = self.tf_buffer.lookup_transform(
                    target_frame,
                    scan.header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0)
                )
            except Exception as e:
                self.get_logger().warning(
                    f'Could not transform from {scan.header.frame_id} to {target_frame}: {e}',
                    throttle_duration_sec=2.0)
                return []

        tx_s, ty_s, yaw_s = self._extract_2d_transform(tf_start)

        # Motion compensation: interpolate TF across scan duration using time_increment.
        # The LD19 takes ~167ms per scan; with a rotating turret this causes significant
        # smearing if all rays are transformed with the same turret angle.
        tx_e, ty_e, yaw_e = tx_s, ty_s, yaw_s
        do_interp = False
        if using_exact_time and scan_duration > 0.0:
            try:
                tf_end = self.tf_buffer.lookup_transform(
                    target_frame,
                    scan.header.frame_id,
                    t_end,
                    timeout=rclpy.duration.Duration(seconds=0)
                )
                tx_e, ty_e, yaw_e = self._extract_2d_transform(tf_end)
                # Normalize yaw delta to [-pi, pi] to handle wrap-around
                dyaw = yaw_e - yaw_s
                if dyaw > math.pi:
                    dyaw -= 2.0 * math.pi
                elif dyaw < -math.pi:
                    dyaw += 2.0 * math.pi
                yaw_e = yaw_s + dyaw
                do_interp = True
            except Exception:
                pass  # fall back to single transform at scan start

        points = []
        inv_n1 = 1.0 / (n - 1) if n > 1 else 0.0
        for i, r in enumerate(scan.ranges):
            if not (self.range_min <= r <= self.range_max):
                continue

            angle = scan.angle_min + i * scan.angle_increment

            if do_interp:
                # LD19 scans CCW; with laser_scan_dir=True the driver mirrors the
                # array, so index 0 = physical 359° (captured last, at t_end) and
                # index n-1 = physical 0° (captured first, at t_start).
                alpha = 1.0 - i * inv_n1
                tx = tx_s + alpha * (tx_e - tx_s)
                ty = ty_s + alpha * (ty_e - ty_s)
                yaw = yaw_s + alpha * (yaw_e - yaw_s)
            else:
                tx, ty, yaw = tx_s, ty_s, yaw_s

            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            x_scan = r * math.cos(angle)
            y_scan = r * math.sin(angle)
            x_target = cos_yaw * x_scan - sin_yaw * y_scan + tx
            y_target = sin_yaw * x_scan + cos_yaw * y_scan + ty

            r_target = math.sqrt(x_target ** 2 + y_target ** 2)
            if self.range_min <= r_target <= self.range_max:
                points.append((math.atan2(y_target, x_target), r_target))

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
                idx = int((angle - self.angle_min) / self.angle_increment + 0.5)
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
