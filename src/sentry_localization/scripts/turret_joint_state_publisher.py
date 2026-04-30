#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray


PUBLISH_RATE = 50.0  # Hz — keeps TF fresh even if MCB data is intermittent


class TurretJointStatePublisher(Node):

    def __init__(self):
        super().__init__('turret_joint_state_publisher')

        self.js_publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.turret_subsciber_ = self.create_subscription(Float32MultiArray, '/turret', self.turret_cb, 10)
        self._last_turret_data = None
        self._last_mcb_time = None
        self.create_timer(1.0 / PUBLISH_RATE, self._timer_cb)

    def publish_turret_js(self, turret_data):
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.name = [
            'gimbal_joint',
            'turret_shaft_joint',
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'back_left_wheel_joint',
            'back_right_wheel_joint'
        ]

        js_msg.position = [
            (-turret_data[0]),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        ]

        js_msg.velocity = [
            (turret_data[1] * -1),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        ]

        self.js_publisher_.publish(js_msg)

    def _timer_cb(self):
        if self._last_turret_data is None or self._last_mcb_time is None:
            return
        dt = min((self.get_clock().now() - self._last_mcb_time).nanoseconds / 1e9, 0.5)
        extrapolated = list(self._last_turret_data)
        extrapolated[0] = self._last_turret_data[0] + self._last_turret_data[1] * dt
        self.publish_turret_js(extrapolated)

    def turret_cb(self, msg: Float32MultiArray):
        self._last_turret_data = msg.data
        self._last_mcb_time = self.get_clock().now()
        self.publish_turret_js(msg.data)


def main(args=None):
    rclpy.init(args=args)

    turret_js_publisher = TurretJointStatePublisher()

    rclpy.spin(turret_js_publisher)

    turret_js_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
