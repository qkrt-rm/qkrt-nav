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

        self.declare_parameter('destination_frame', 'base_link')
        self.declare_parameter('fixed_frame', 'odom')
        self.declare_parameter('scan_destination_topic', '/scan')
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', 0.01)
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 10.0)

        self.destination_frame = self.get_parameter('destination_frame').value
        self.fixed_frame = self.get_parameter('fixed_frame').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scan_left = None
        self.scan_right = None

        self.sub_left = self.create_subscription(
            LaserScan, '/scan_left', self.scan_left_callback, qos_profile_sensor_data)
        self.sub_right = self.create_subscription(
            LaserScan, '/scan_right', self.scan_right_callback, qos_profile_sensor_data)

        scan_topic = self.get_parameter('scan_destination_topic').value
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.pub_scan = self.create_publisher(LaserScan, scan_topic, reliable_qos)
        self.create_timer(0.05, self.publish_merged_scan)

        self.get_logger().info(
            f'Laser merger started, publishing to {scan_topic}, '
            f'deskew bridge frame: {self.fixed_frame}')

    def scan_left_callback(self, msg):
        self.scan_left = msg

    def scan_right_callback(self, msg):
        self.scan_right = msg

    def _extract_2d_transform(self, tf_stamped):
        tx = tf_stamped.transform.translation.x
        ty = tf_stamped.transform.translation.y
        q = tf_stamped.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return tx, ty, math.atan2(siny_cosp, cosy_cosp)

    def _lookup_deskew(self, source_frame, source_time, target_frame):
        """Look up the deskew transform for one ray endpoint in time.

        Tries three methods in order, returning (transform, mode_str):
          'full'   — odom-bridged: corrects turret rotation + robot body motion
          'turret' — exact time, no bridge: corrects turret rotation only
          'latest' — latest available TF: no per-ray correction at all
        Returns (None, None) if all methods fail.

        Target time is rclpy.time.Time() (latest EKF) to avoid blocking the
        executor.  The merged scan is stamped with self.get_clock().now() to
        match, so SLAM applies the correct pose.  Residual error is at most
        one EKF period (~20 ms at 50 Hz) — roughly 2 cm at 1 m/s.
        """
        zero = rclpy.duration.Duration(seconds=0)
        try:
            tf = self.tf_buffer.lookup_transform_full(
                target_frame, rclpy.time.Time(),
                source_frame, source_time,
                self.fixed_frame, zero)
            return tf, 'full'
        except Exception:
            pass
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame, source_frame, source_time, zero)
            return tf, 'turret'
        except Exception:
            pass
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time(), zero)
            return tf, 'latest'
        except Exception:
            return None, None

    def transform_scan_to_frame(self, scan, target_frame):
        """Transform scan rays to target_frame with full per-ray deskewing.

        Each ray is corrected using the TF at its exact capture time, bridged
        through odom so both turret rotation and robot body motion are removed.
        All resulting points are expressed in target_frame at rclpy.time.Time()
        (the latest available EKF time).
        """
        n = len(scan.ranges)
        scan_duration = (n - 1) * scan.time_increment if scan.time_increment > 0.0 and n > 1 else 0.0
        t_end = rclpy.time.Time.from_msg(scan.header.stamp)
        t_start = t_end - rclpy.duration.Duration(seconds=scan_duration)

        tf_start, mode = self._lookup_deskew(
            scan.header.frame_id, t_start, target_frame)
        if tf_start is None:
            self.get_logger().warning(
                f'TF completely unavailable: {scan.header.frame_id} → {target_frame}',
                throttle_duration_sec=2.0)
            return []

        tx_s, ty_s, yaw_s = self._extract_2d_transform(tf_start)
        tx_e, ty_e, yaw_e = tx_s, ty_s, yaw_s
        do_interp = False

        # Only interpolate when we have a consistent time-aware lookup for both endpoints.
        # 'latest' mode gives a TF at an unknown time, so interpolating to t_end is wrong.
        if mode != 'latest' and scan_duration > 0.0:
            tf_end, mode_end = self._lookup_deskew(
                scan.header.frame_id, t_end, target_frame)
            if tf_end is not None and mode_end == mode:
                tx_e, ty_e, yaw_e = self._extract_2d_transform(tf_end)
                dyaw = yaw_e - yaw_s
                if dyaw > math.pi:
                    dyaw -= 2.0 * math.pi
                elif dyaw < -math.pi:
                    dyaw += 2.0 * math.pi
                yaw_e = yaw_s + dyaw
                do_interp = True

        if not do_interp:
            self.get_logger().warning(
                f'Deskewing inactive for {scan.header.frame_id} (mode={mode}): '
                f'using single transform for all rays',
                throttle_duration_sec=5.0)

        points = []
        inv_n1 = 1.0 / (n - 1) if n > 1 else 0.0
        for i, r in enumerate(scan.ranges):
            if not (self.range_min <= r <= self.range_max):
                continue

            angle = scan.angle_min + i * scan.angle_increment

            if do_interp:
                # LD19 scans CCW; with laser_scan_dir=True the driver mirrors the
                # array so index 0 = physical 359° (captured last, at t_end) and
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

        # Probe the actual latest EKF stamp so the merged scan is stamped with
        # a time that is already in the TF buffer.  The deskew lookups use
        # rclpy.time.Time() (= this same EKF time), so stamp and deskew target
        # are consistent — SLAM can look up the TF immediately without waiting
        # for extrapolation into the future (which caused the message-filter
        # queue-full warnings and the room appearing to drift backwards).
        zero = rclpy.duration.Duration(seconds=0)
        try:
            ekf_tf = self.tf_buffer.lookup_transform(
                self.destination_frame, self.fixed_frame, rclpy.time.Time(), zero)
            merged_stamp = ekf_tf.header.stamp
        except Exception:
            merged_stamp = self.get_clock().now().to_msg()

        all_points = []
        if self.scan_left is not None:
            all_points.extend(
                self.transform_scan_to_frame(self.scan_left, self.destination_frame))
        if self.scan_right is not None:
            all_points.extend(
                self.transform_scan_to_frame(self.scan_right, self.destination_frame))

        if not all_points:
            return

        num_readings = int((self.angle_max - self.angle_min) / self.angle_increment)
        ranges = [float('inf')] * num_readings

        for angle, r in all_points:
            if self.angle_min <= angle <= self.angle_max:
                idx = int((angle - self.angle_min) / self.angle_increment + 0.5)
                if 0 <= idx < num_readings:
                    ranges[idx] = min(ranges[idx], r)

        merged = LaserScan()
        merged.header.stamp = merged_stamp
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
