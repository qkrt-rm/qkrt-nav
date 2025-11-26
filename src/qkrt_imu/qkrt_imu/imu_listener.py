import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuListener(Node):
    def __init__(self):
        super().__init__('imu_listener')

        self.subscription = self.create_subscription(
            Imu,
            'imu/data_repub',
            self.imu_callback,
            10
        )

        self.get_logger().info('IMU listener started, waiting for imu/data_repub...')

    def imu_callback(self, msg: Imu):
        q = msg.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        acc_mag = math.sqrt(ax*ax + ay*ay + az*az)

        self.get_logger().info(
            f"yaw: {yaw:.3f} rad, acc: ({ax:.2f}, {ay:.2f}, {az:.2f}) m/s^2, |a|={acc_mag:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ImuListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
