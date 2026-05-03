#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray


PUBLISH_RATE = 50.0  # Hz


class TurretJointStatePublisher(Node):

    def __init__(self):
        super().__init__('turret_joint_state_publisher')

        self.js_publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.turret_subscriber_ = self.create_subscription(
            Float32MultiArray, '/turret', self.turret_cb, 10)

        self._last_turret_data = None
        self._last_mcb_time = None

        # Publish at fixed rate, extrapolating forward from last known encoder position.
        self.create_timer(1.0 / PUBLISH_RATE, self._timer_cb)

    def publish_joint_states(self, turret_data):
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()

        # chassis_joint: angle of chassis_link relative to base_link (turret frame).
        # turret_data[0] is the absolute encoder reading (turret relative to chassis).
        # Chassis relative to turret = +turret_data[0] (opposite rotation direction).
        # turret_data[1] is turretYawVel — same sign convention as turret_data[0].
        js_msg.name = [
            'chassis_joint',
            'turret_shaft_joint',
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'back_left_wheel_joint',
            'back_right_wheel_joint'
        ]
        js_msg.position = [
            turret_data[0],
            0.0, 0.0, 0.0, 0.0, 0.0
        ]
        js_msg.velocity = [
            turret_data[1],
            0.0, 0.0, 0.0, 0.0, 0.0
        ]
        self.js_publisher_.publish(js_msg)

    def _timer_cb(self):
        if self._last_turret_data is None or self._last_mcb_time is None:
            return
        dt = (self.get_clock().now() - self._last_mcb_time).nanoseconds / 1e9
        dt = min(dt, 0.5)
        extrapolated = list(self._last_turret_data)
        extrapolated[0] = self._last_turret_data[0] + self._last_turret_data[1] * dt
        self.publish_joint_states(extrapolated)

    def turret_cb(self, msg: Float32MultiArray):
        self._last_turret_data = msg.data
        self._last_mcb_time = self.get_clock().now()
        self.publish_joint_states(msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = TurretJointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
