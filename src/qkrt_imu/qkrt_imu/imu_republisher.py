import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuRepublisher(Node):
    def __init__(self):
        super().__init__('imu_republisher')

        self.declare_parameter('input_topic', 'imu/data')
        self.declare_parameter('output_topic', 'imu/data_repub')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('convert_deg_to_rad', False)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.convert_deg_to_rad = self.get_parameter('convert_deg_to_rad').value

        self.get_logger().info(
            f'IMU Republisher started.\n'
            f'  input_topic: {self.input_topic}\n'
            f'  output_topic: {self.output_topic}\n'
            f'  frame_id: {self.frame_id}\n'
            f'  convert_deg_to_rad: {self.convert_deg_to_rad}'
        )

        self.sub = self.create_subscription(
            Imu,
            self.input_topic,
            self.imu_callback,
            50
        )

        self.pub = self.create_publisher(Imu, self.output_topic, 50)

    def imu_callback(self, msg: Imu):
        msg.header.frame_id = self.frame_id

        if self.convert_deg_to_rad:
            factor = math.pi / 180.0
            msg.angular_velocity.x *= factor
            msg.angular_velocity.y *= factor
            msg.angular_velocity.z *= factor

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
