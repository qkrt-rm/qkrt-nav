import struct
import threading

from sentry_communication.communication import RobotPositionMessage, Serial
from sentry_communication.communication.Receive import parse_frame
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from rclpy.timer import Timer

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
    message = RobotPositionMessage(command)
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

class BSPublisher(Node):
    def __init__(self):
        super().__init__('bs_publisher')
        timer_period = 0.001
        self.timer:Timer = self.create_timer(timer_period,self.timer_callback)
    def timer_callback(self):
        byte = serial.port.read(1)
        self.get_logger().info(f'bytes are being received: {byte}')

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'mcb_odom', 10)
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        while rclpy.ok():
            frame = read_frame(serial.port)
            if frame is None:
                continue

            received = parse_frame(frame)
            if received is None:
                self.get_logger().warn('Bad frame received (CRC mismatch)')
                continue

            if received["msg_type"] == MCB_MESSAGE_TYPE_ODOM:
                body = received["body"]
                if len(body) == ODOM_BYTES:
                    values = struct.unpack(f'<{ODOM_NUM_FLOATS}f', body)
                    msg = Float32MultiArray()
                    msg.layout = MultiArrayLayout(
                        dim=[MultiArrayDimension(label='odom', size=ODOM_NUM_FLOATS, stride=ODOM_NUM_FLOATS)],
                        data_offset=0,
                    )
                    msg.data = list(values)
                    self.publisher_.publish(msg)
                    self.get_logger().info(
                        f'Odom: ' + ', '.join(f'{l}={v:.3f}' for l, v in zip(ODOM_LABELS, values)))


def main():
    rclpy.init()
    nav_subscriber = NavSubscriber()
    odom_publisher = OdomPublisher()
    bs_publisher = BSPublisher()

    executor = MultiThreadedExecutor()
    executor.add_node(nav_subscriber)
    executor.add_node(bs_publisher)
    executor.add_node(odom_publisher)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        nav_subscriber.destroy_node()
        odom_publisher.destroy_node()
        bs_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
