#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import serial
import struct
import math


# DJISerial frame format
FRAME_HEADER = 0xA5
SENSOR_DATA_MSG_TYPE = 0x0002
SENSOR_DATA_SIZE = 56
SENSOR_DATA_FORMAT = '<4f 3f 3f 3f I'  # little-endian


def crc8(data):
    """CRC8 calculation (same as Taproot)"""
    crc = 0xFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x31
            else:
                crc <<= 1
            crc &= 0xFF
    return crc


def parse_sensor_data(data):
    values = struct.unpack(SENSOR_DATA_FORMAT, data)
    return {
        'wheel_velocities': values[0:4],  # LF, LB, RB, RF in rad/s
        'gyro': {'x': values[4], 'y': values[5], 'z': values[6]},  # rad/s
        'accel': {'x': values[7], 'y': values[8], 'z': values[9]},  # m/s^2
        'orientation': {'yaw': values[10], 'pitch': values[11], 'roll': values[12]},
        'timestamp_ms': values[13]
    }


def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles to quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyTHS1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('base_width', 0.495)  # Distance between left and right wheels
        self.declare_parameter('base_length', 0.495)  # Distance between front and back wheels
        self.declare_parameter('publish_tf', True)

        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.base_width = self.get_parameter('base_width').value
        self.base_length = self.get_parameter('base_length').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # Mecanum kinematics parameters
        self.lx = self.base_length / 2.0  # Distance from center to front/back wheels
        self.ly = self.base_width / 2.0   # Distance from center to left/right wheels

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)

        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Serial connection
        try:
            self.serial = serial.Serial(serial_port, baud_rate, timeout=0.1)
            self.get_logger().info(f'Connected to {serial_port} at {baud_rate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.serial = None

        self.buffer = bytearray()

        # Timer for reading serial data
        self.create_timer(0.01, self.read_serial)  # 100 Hz

    def compute_mecanum_odometry(self, wheel_velocities):
        """
        Compute robot velocities from mecanum wheel velocities.
        wheel_velocities: [LF, LB, RB, RF] in rad/s
        Returns: (vx, vy, omega) in robot frame
        """
        w_lf, w_lb, w_rb, w_rf = wheel_velocities
        r = self.wheel_radius

        # Mecanum wheel kinematics (inverse)
        # vx = (w_lf + w_lb + w_rb + w_rf) * r / 4
        # vy = (-w_lf + w_lb - w_rb + w_rf) * r / 4
        # omega = (-w_lf - w_lb + w_rb + w_rf) * r / (4 * (lx + ly))

        vx = (w_lf + w_lb + w_rb + w_rf) * r / 4.0
        vy = (-w_lf + w_lb - w_rb + w_rf) * r / 4.0
        omega = (-w_lf - w_lb + w_rb + w_rf) * r / (4.0 * (self.lx + self.ly))

        return vx, vy, omega

    def update_odometry(self, vx, vy, omega):
        """Update odometry position estimate."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Transform velocities from robot frame to world frame
        vx_world = vx * math.cos(self.theta) - vy * math.sin(self.theta)
        vy_world = vx * math.sin(self.theta) + vy * math.cos(self.theta)

        # Integrate
        self.x += vx_world * dt
        self.y += vy_world * dt
        self.theta += omega * dt

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        return dt

    def publish_odom(self, vx, vy, omega, orientation):
        """Publish odometry message and TF."""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation from IMU
        q = euler_to_quaternion(
            orientation['roll'],
            orientation['pitch'],
            orientation['yaw']
        )
        odom.pose.pose.orientation = q

        # Velocity in robot frame
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = omega

        # Covariance (adjust based on your robot's accuracy)
        odom.pose.covariance[0] = 0.01   # x
        odom.pose.covariance[7] = 0.01   # y
        odom.pose.covariance[35] = 0.01  # yaw
        odom.twist.covariance[0] = 0.01  # vx
        odom.twist.covariance[7] = 0.01  # vy
        odom.twist.covariance[35] = 0.01 # omega

        self.odom_pub.publish(odom)

        # Publish TF
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

    def publish_imu(self, gyro, accel, orientation):
        """Publish IMU message."""
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'imu_link'

        # Orientation
        q = euler_to_quaternion(
            orientation['roll'],
            orientation['pitch'],
            orientation['yaw']
        )
        imu.orientation = q
        imu.orientation_covariance[0] = 0.01
        imu.orientation_covariance[4] = 0.01
        imu.orientation_covariance[8] = 0.01

        # Angular velocity
        imu.angular_velocity.x = gyro['x']
        imu.angular_velocity.y = gyro['y']
        imu.angular_velocity.z = gyro['z']
        imu.angular_velocity_covariance[0] = 0.01
        imu.angular_velocity_covariance[4] = 0.01
        imu.angular_velocity_covariance[8] = 0.01

        # Linear acceleration
        imu.linear_acceleration.x = accel['x']
        imu.linear_acceleration.y = accel['y']
        imu.linear_acceleration.z = accel['z']
        imu.linear_acceleration_covariance[0] = 0.01
        imu.linear_acceleration_covariance[4] = 0.01
        imu.linear_acceleration_covariance[8] = 0.01

        self.imu_pub.publish(imu)

    def read_serial(self):
        """Read and parse serial data."""
        if self.serial is None:
            return

        try:
            data = self.serial.read(256)
            if data:
                self.buffer.extend(data)
        except serial.SerialException as e:
            self.get_logger().error(f'Serial read error: {e}')
            return

        # Process buffer
        while True:
            # Find frame header
            while len(self.buffer) > 0 and self.buffer[0] != FRAME_HEADER:
                self.buffer.pop(0)

            # Need at least header (5 bytes) + msg_type (2) + data + crc16 (2)
            if len(self.buffer) < 9:
                break

            data_length = self.buffer[1] | (self.buffer[2] << 8)
            frame_size = 5 + 2 + data_length + 2

            if len(self.buffer) < frame_size:
                break

            msg_type = self.buffer[5] | (self.buffer[6] << 8)

            if msg_type == SENSOR_DATA_MSG_TYPE and data_length == SENSOR_DATA_SIZE:
                sensor_data = parse_sensor_data(bytes(self.buffer[7:7+SENSOR_DATA_SIZE]))

                # Compute odometry from wheel velocities
                vx, vy, omega = self.compute_mecanum_odometry(sensor_data['wheel_velocities'])
                self.update_odometry(vx, vy, omega)

                # Publish
                self.publish_odom(vx, vy, omega, sensor_data['orientation'])
                self.publish_imu(sensor_data['gyro'], sensor_data['accel'], sensor_data['orientation'])

            self.buffer = self.buffer[frame_size:]


def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
