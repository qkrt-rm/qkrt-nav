#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState   
from std_msgs.msg import Float32MultiArray   



class TurretJointStatePublisher(Node):

    def __init__(self):
        super().__init__('turret_joint_state_publisher')

        self.js_publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.turret_subsciber_ = self.create_subscription(Float32MultiArray, '/turret', self.turret_cb, 10)
        

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

        # to fill in later
        js_msg.position = [
            ((-1 * turret_data[0]) + 1.57079633), #90 degree offset to align with the physical turret orientation
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
    

    def turret_cb(self, msg: Float32MultiArray):
        turret_data = msg.data
        self.publish_turret_js(turret_data)


def main(args=None):
    rclpy.init(args=args)

    turret_js_publisher = TurretJointStatePublisher()

    rclpy.spin(turret_js_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turret_js_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()