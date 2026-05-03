#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float32MultiArray
import math


class TurretJointStatePublisher(Node):

    def __init__(self):
        super().__init__('turret_joint_state_publisher')

        self.js_publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.turret_subsciber_ = self.create_subscription(Float32MultiArray, '/turret', self.turret_cb, 10)
        self.imu_subscriber_ = self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        self.gyro_z = 0.0

        self.integrated_turret_position = 0.0
        self.prev_imu_time = None

        self.base_heading = None
        self._last_turret_data = None
        self._last_mcb_time = None

        # Heartbeat: republish last known joint states so TF never goes stale
        # even if the MCB serial hiccups briefly.
        self.create_timer(0.02, self._heartbeat_cb)  # 50 Hz

    def _heartbeat_cb(self):
        if self._last_turret_data is not None:
            self.publish_turret_js(self._last_turret_data)

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
            self.integrated_turret_position,
            -turret_data[2],
            0.0,
            0.0,
            0.0,
            0.0
        ]
        js_msg.velocity = [
            -turret_data[1],
            turret_data[3],
            0.0,
            0.0,
            0.0,
            0.0
        ]
        self.js_publisher_.publish(js_msg)

    def imu_cb(self, msg: Imu):
        self.gyro_z = msg.angular_velocity.z
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_imu_time is not None:
            dt = current_time - self.prev_imu_time
            self.integrated_turret_position += self.gyro_z * dt

            if self.integrated_turret_position < 0:
                self.integrated_turret_position += 2 * math.pi
            elif self.integrated_turret_position >= 2 * math.pi:
                self.integrated_turret_position -= 2 * math.pi

        self.prev_imu_time = current_time
        self.get_logger().info(f"itp: {self.integrated_turret_position}", throttle_duration_sec=1.0)

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
        #self.get_logger().info(f"Current: {turret_heading:.3f}, "f"Base: {self.base_heading:.3f}, "f"Joint: {joint_angle:.3f}")


def main(args=None):
    rclpy.init(args=args)

    node = TurretJointStatePublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()