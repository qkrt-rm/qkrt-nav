#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float32MultiArray
import math


PUBLISH_RATE = 50.0  # Hz
class TurretJointStatePublisher(Node):

    def __init__(self):
        super().__init__('turret_joint_state_publisher')

        self.js_publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.turret_subsciber_ = self.create_subscription(Float32MultiArray, '/turret', self.turret_cb, 10)
        self.imu_subscriber_ = self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        self.gyro_z = 0.0

        self.base_heading = None
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
            -turret_data[2],
            0.0,
            0.0,
            0.0,
            0.0
        ]
        js_msg.velocity = [
            (turret_data[1] * -1),
            turret_data[3],
            0.0,
            0.0,
            0.0,
            0.0
        ]
        self.js_publisher_.publish(js_msg)

    def _timer_cb(self):
        if self._last_turret_data is None or self._last_mcb_time is None:
            return
        dt = (self.get_clock().now() - self._last_mcb_time).nanoseconds / 1e9
        dt = min(dt, 0.5)
        extrapolated = list(self._last_turret_data)
        extrapolated[0] = self._last_turret_data[0] + self.gyro_z * dt
        self.publish_turret_js(extrapolated)

    def imu_cb(self, msg: Imu):
        self.gyro_z = msg.angular_velocity.z

    def turret_cb(self, msg: Float32MultiArray):
        self._last_turret_data = msg.data
        self._last_mcb_time = self.get_clock().now()
        turret_heading = msg.data[0]

        if self.base_heading is None:
            self.base_heading = turret_heading

        joint_angle = turret_heading - self.base_heading
        joint_angle = -joint_angle
        
        if joint_angle < 0:
            joint_angle += 2 * math.pi

        # overwrite for publishing
        # positive for ccw, negative for cw
        turret_data = list(msg.data)
        self.publish_turret_js(turret_data)
        self.get_logger().info(f"Current: {turret_heading:.3f}, "f"Base: {self.base_heading:.3f}, "f"Joint: {joint_angle:.3f}")


def main(args=None):
    rclpy.init(args=args)

    node = TurretJointStatePublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()