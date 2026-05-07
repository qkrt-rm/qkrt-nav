import struct
import threading
import math
import time

from sentry_communication.communication import RobotPositionMessage, NavMessage, Serial
from sentry_communication.communication.Receive import parse_frame
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout, UInt8MultiArray, UInt32
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

# MCB uses UART1 at 115200 baud
serial = Serial("/dev/ttyTHS1", 115200)

# Message type IDs matching jetson_message.hpp on the MCB
MCB_MESSAGE_TYPE_ODOM = 3


def read_frame(ser):
    """Synchronize on 0xA5 and read one complete DJI serial frame.

    Returns the full frame as bytes, or None on timeout.
    """
    # State 1: scan for start byte
    while True:
        byte = ser.read(1)
        if len(byte) == 0:
            return None
        if byte[0] == 0xA5:
            break

    # State 2: read rest of header — payload_len(2) + seq(1) + crc8(1)
    hdr_rest = ser.read(4)
    if len(hdr_rest) < 4:
        return None

    payload_len = struct.unpack_from('<H', hdr_rest, 0)[0]
    if payload_len > 1024:
        return None

    # State 3: read msg_type(2) + payload(payload_len) + crc16(2)
    remaining_size = 2 + payload_len + 2
    remaining = ser.read(remaining_size)
    if len(remaining) < remaining_size:
        return None

    # Assemble full frame for parse_frame()
    frame = bytes([0xA5]) + hdr_rest + remaining
    return frame


def sendVelocityCommand(command: list[float]):
    message = NavMessage(command)
    serial.write(message.createMessage())


class NavSubscriber(Node):
    def __init__(self):
        super().__init__('nav_subscriber')
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.listener_callback, 10)

    def listener_callback(self, msg):
        cmd = [msg.linear.x, msg.linear.y, msg.angular.z]
        self.get_logger().info(f'Sending Command: {cmd}')
        sendVelocityCommand(cmd)


ODOM_LABELS = [
    'wheelLF', 'wheelLB', 'wheelRB', 'wheelRF',
    'turretYawPos', 'turretYawVel', 'turretPitchPos', 'turretPitchVel',
    'imuAx', 'imuAy', 'imuAz',
    'imuGx', 'imuGy', 'imuGz',
    'imuYaw', 'imuPitch', 'imuRoll',
]
ODOM_NUM_FLOATS = len(ODOM_LABELS)
ODOM_BYTES = ODOM_NUM_FLOATS * 4  # 68

# Robot geometry for mecanum wheel odometry
# Wheel radius from MCB: WHEEL_DIAMETER_M = 0.076 in holonomic_chassis_subsystem.hpp
WHEEL_RADIUS = 0.038  # meters
# MCB getVelocity() appears to return wheel shaft rad/s (already gear-reduced in firmware).
# Empirically calibrated: robot travels 1m, odom reported 0.513m with GEAR_RATIO=19 → ratio ~9.75.
GEAR_RATIO = 9.75
WHEEL_BASE_X = 0.2475  # half of base_width (distance from center to wheel along x)
WHEEL_BASE_Y = 0.2475  # half of base_length (distance from center to wheel along y)

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        # Raw/debug publishers
        self.publisher_ = self.create_publisher(Float32MultiArray, 'mcb_odom', 10)
        self.raw_publisher_ = self.create_publisher(UInt8MultiArray, 'mcb_raw', 10)
        self.test_publisher_ = self.create_publisher(UInt32, 'mcb_test_counter', 10)

        # Separate topic publishers
        self.imu_publisher_ = self.create_publisher(Imu, 'imu', 10)
        self.wheel_publisher_ = self.create_publisher(Float32MultiArray, 'wheel_speeds', 10)
        self.turret_publisher_ = self.create_publisher(Float32MultiArray, 'turret', 10)

        self.odom_publisher_ = self.create_publisher(Odometry, 'odom', 10)

        # Odometry state (chassis = base_link frame)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.last_odom_time = None

        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        self.get_logger().info('Read loop started, waiting for data...')
        while rclpy.ok():
            frame = read_frame(serial.port)
            if frame is None:
                continue

            # Publish raw frame bytes (for debugging)
            raw_msg = UInt8MultiArray()
            raw_msg.data = list(frame)
            self.raw_publisher_.publish(raw_msg)

            received = parse_frame(frame)
            if received is None:
                self.get_logger().warn('Bad frame received (CRC mismatch)')
                continue

            if received["msg_type"] == MCB_MESSAGE_TYPE_ODOM:
                body = received["body"]

                # Test message: 4-byte counter
                if len(body) == 4:
                    counter = struct.unpack('<I', body)[0]
                    msg = UInt32()
                    msg.data = counter
                    self.test_publisher_.publish(msg)
                    self.get_logger().info(f'Test counter received: {counter}')

                # Full odom message: 68 bytes (17 floats)
                elif len(body) == ODOM_BYTES:
                    values = struct.unpack(f'<{ODOM_NUM_FLOATS}f', body)

                    # Publish combined mcb_odom (all values)
                    msg = Float32MultiArray()
                    msg.layout = MultiArrayLayout(
                        dim=[MultiArrayDimension(label='odom', size=ODOM_NUM_FLOATS, stride=ODOM_NUM_FLOATS)],
                        data_offset=0,
                    )
                    msg.data = list(values)
                    self.publisher_.publish(msg)

                    # Publish wheel speeds (indices 0-3)
                    wheel_msg = Float32MultiArray()
                    wheel_msg.layout = MultiArrayLayout(
                        dim=[MultiArrayDimension(label='wheels', size=4, stride=4)],
                        data_offset=0,
                    )
                    wheel_msg.data = list(values[0:4])  # wheelLF, wheelLB, wheelRB, wheelRF
                    self.wheel_publisher_.publish(wheel_msg)

                    # Publish turret data (indices 4-7)
                    turret_msg = Float32MultiArray()
                    turret_msg.layout = MultiArrayLayout(
                        dim=[MultiArrayDimension(label='turret', size=4, stride=4)],
                        data_offset=0,
                    )
                    turret_msg.data = list(values[4:8])  # yawPos, yawVel, pitchPos, pitchVel
                    self.turret_publisher_.publish(turret_msg)

                    # Publish IMU data (indices 8-16)
                    imu_msg = Imu()
                    imu_msg.header.stamp = self.get_clock().now().to_msg()
                    imu_msg.header.frame_id = 'imu_link'
                    # Linear acceleration (indices 8-10)
                    imu_msg.linear_acceleration.x = values[8]   # imuAx
                    imu_msg.linear_acceleration.y = values[9]   # imuAy
                    imu_msg.linear_acceleration.z = values[10]  # imuAz
                    # Angular velocity (indices 11-13)
                    imu_msg.angular_velocity.x = values[11]  # imuGx
                    imu_msg.angular_velocity.y = values[12]  # imuGy
                    imu_msg.angular_velocity.z = values[13]  # imuGz
                    imu_msg.angular_velocity_covariance[8] = 0.01  # gyro z variance (rad/s)^2
                    # Orientation from euler angles (indices 14-16: yaw, pitch, roll)
                    yaw, pitch, roll = values[14], values[15], values[16]
                    # Convert euler to quaternion (ZYX convention)
                    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
                    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
                    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
                    imu_msg.orientation.w = cr * cp * cy + sr * sp * sy
                    imu_msg.orientation.x = sr * cp * cy - cr * sp * sy
                    imu_msg.orientation.y = cr * sp * cy + sr * cp * sy
                    imu_msg.orientation.z = cr * cp * sy - sr * sp * cy
                    self.imu_publisher_.publish(imu_msg)

                    # Compute and publish wheel odometry
                    self._publish_odometry(values[0:4], yaw)

                    self.get_logger().debug(
                        f'Odom: ' + ', '.join(f'{l}={v:.3f}' for l, v in zip(ODOM_LABELS, values)))

                else:
                    self.get_logger().warn(f'Unknown ODOM body size: {len(body)} bytes')

    def _publish_odometry(self, wheel_speeds, imu_yaw):
        """Compute odometry from mecanum wheel speeds and publish.

        base_link is the chassis frame. odom_theta tracks chassis heading.
        The turret's world orientation is handled by the TF chain through gimbal_joint.
        """
        current_time = time.time()

        if self.last_odom_time is None:
            self.last_odom_time = current_time
            return

        dt = current_time - self.last_odom_time
        self.last_odom_time = current_time

        if dt <= 0.0 or dt > 1.0:
            return

        # Negate and scale: MCB sends motor shaft rad/s with sign inverted relative to our convention.
        w_lf, w_lb, w_rb, w_rf = [-v / GEAR_RATIO for v in wheel_speeds]

        r = WHEEL_RADIUS
        l_sum = WHEEL_BASE_X + WHEEL_BASE_Y

        vx = -(w_lf + w_rf + w_lb + w_rb) * r / 4.0   # forward (all wheels same direction)
        vy = -(-w_lf + w_rf + w_lb - w_rb) * r / 4.0  # lateral (strafe pattern)
        omega = (-w_lf + w_rf - w_lb + w_rb) * r / (4.0 * l_sum)

        self.odom_theta += omega * dt

        cos_theta = math.cos(self.odom_theta)
        sin_theta = math.sin(self.odom_theta)
        self.odom_x += (vx * cos_theta - vy * sin_theta) * dt
        self.odom_y += (vx * sin_theta + vy * cos_theta) * dt

        now = self.get_clock().now().to_msg()

        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.odom_x
        odom_msg.pose.pose.position.y = self.odom_y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.odom_theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.odom_theta / 2.0)

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = omega

        odom_msg.pose.covariance[0] = 0.01   # x
        odom_msg.pose.covariance[7] = 0.01   # y
        odom_msg.pose.covariance[35] = 0.01  # yaw
        odom_msg.twist.covariance[0] = 0.01  # vx
        odom_msg.twist.covariance[7] = 0.01  # vy
        odom_msg.twist.covariance[35] = 0.01  # omega

        self.odom_publisher_.publish(odom_msg)


def main():
    rclpy.init()
    nav_subscriber = NavSubscriber()
    odom_publisher = OdomPublisher()

    executor = MultiThreadedExecutor()
    executor.add_node(nav_subscriber)
    executor.add_node(odom_publisher)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        nav_subscriber.destroy_node()
        odom_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
