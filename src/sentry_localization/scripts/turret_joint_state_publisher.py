#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float32MultiArray, String
from nav_msgs.msg import Odometry
import math


class TurretJointStatePublisher(Node):

    def __init__(self):
        super().__init__('turret_joint_state_publisher')

        self.js_publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.turret_subsciber_ = self.create_subscription(Float32MultiArray, '/turret', self.turret_cb, 10)
        self.imu_subscriber_ = self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        self.odom_subscriber_ = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        self.gyro_z = 0.0
        self.odom_omega = 0.0

        self.integrated_turret_position = 0.0
        self.prev_imu_time = None
        self.prev_odom_time = None

        self.base_heading = None
        self._last_turret_data = None
        self._last_mcb_time = None
        self.test_publisher_ = self.create_publisher(String, '/adjusted_speed', 10)

        self.create_timer(0.02, self._heartbeat_cb)  # 50 Hz

    def _heartbeat_cb(self):
        if self._last_turret_data is not None:
            self.publish_turret_js(self._last_turret_data)

    def publish_turret_js(self, turret_data, stamp=None):
        js_msg = JointState()
        js_msg.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()

        js_msg.name = [
            'gimbal_joint',
            'turret_shaft_joint',
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'back_left_wheel_joint',
            'back_right_wheel_joint'
        ]

        js_msg.position = [
            self.integrated_turret_position,
            -turret_data[2],
            0.0, 0.0, 0.0, 0.0
        ]
        js_msg.velocity = [
            self.odom_omega - self.gyro_z,
            turret_data[3],
            0.0, 0.0, 0.0, 0.0
        ]

        self.js_publisher_.publish(js_msg)

        newMsg = String()
        newMsg.data = f"new_speed: {self.odom_omega - self.gyro_z:.3f}"
        self.test_publisher_.publish(newMsg)

    def imu_cb(self, msg: Imu):
        self.gyro_z = msg.angular_velocity.z

        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.prev_imu_time is not None:
            dt = current_time - self.prev_imu_time
            self.integrated_turret_position += (self.odom_omega - self.gyro_z) * dt

        self.prev_imu_time = current_time

        if self._last_turret_data is not None:
            self.publish_turret_js(self._last_turret_data, stamp=msg.header.stamp)

    def odom_cb(self, msg: Odometry):
        self.odom_omega = msg.twist.twist.angular.z
        self.prev_odom_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def turret_cb(self, msg: Float32MultiArray):
        self._last_turret_data = msg.data
        self._last_mcb_time = self.get_clock().now()

        turret_heading = msg.data[0]
        if self.base_heading is None:
            self.base_heading = turret_heading

        self.publish_turret_js(list(msg.data))


def main(args=None):
    rclpy.init(args=args)
    node = TurretJointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
