#!/usr/bin/env python3
#
# CMD_VEL GIMBAL ROTATOR
# ======================
# WHY THIS EXISTS
# ---------------
# Nav2 tells the robot to move in directions relative to the chassis (base_link).
# For example "move 0.5 m/s forward" means forward relative to the chassis body.
#
# But our MCB firmware expects velocity commands relative to the TURRET (gimbal_link),
# because the turret is what the operator thinks of as "forward". If the turret is
# pointing 90° left and nav2 says "go forward", we actually want the chassis to
# strafe left so the robot moves in the direction the turret is pointing.
#
# This node sits in between nav2 and the MCB and rotates the velocity vector to
# account for the current turret angle, so "forward" always means turret-forward.
#
# THE MATHS
# ---------
# If the turret is rotated θ degrees relative to the chassis, then a velocity
# (vx, vy) in chassis frame becomes (vx', vy') in turret frame using a simple
# 2D rotation matrix with angle -θ:
#
#   vx' =  vx * cos(θ) + vy * sin(θ)    (this is just rotating the vector)
#   vy' = -vx * sin(θ) + vy * cos(θ)
#
# Angular velocity (spin) is the same in both frames, so it passes through unchanged.
#
# ONLY RUNS ON THE REAL ROBOT
# ---------------------------
# This node is launched by real_robot.launch.py but NOT simulated_robot.launch.py.
# In simulation, Gazebo handles movement directly in world coordinates so no
# rotation is needed.
#
# TOPIC FLOW
# ----------
#   nav2  →  /cmd_vel  →  [this node]  →  /cmd_vel_rotated  →  comm_hub  →  MCB

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


class CmdVelGimbalRotator(Node):
    def __init__(self):
        super().__init__('cmd_vel_gimbal_rotator')

        # These match what's set in real_robot.launch.py — change there, not here
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel_rotated')
        self.declare_parameter('gimbal_joint_name', 'gimbal_joint')

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.gimbal_joint_name = self.get_parameter('gimbal_joint_name').value

        # Current turret angle relative to chassis (radians, CCW positive)
        # Stays 0.0 until the first joint state message arrives
        self.gimbal_angle = 0.0

        self.pub = self.create_publisher(Twist, out_topic, 10)
        self.create_subscription(Twist, in_topic, self._cmd_vel_cb, 10)
        # We read the turret angle from /joint_states, published by comm_hub or Gazebo
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

    def _joint_state_cb(self, msg: JointState):
        # Keep our local copy of the turret angle up to date
        if self.gimbal_joint_name in msg.name:
            idx = msg.name.index(self.gimbal_joint_name)
            if idx < len(msg.position):
                self.gimbal_angle = msg.position[idx]

    def _cmd_vel_cb(self, msg: Twist):
        # Rotate the incoming velocity from chassis frame into turret frame.
        theta = self.gimbal_angle + 3*math.pi/2      # add 90° to convert from chassis-forward to turret-forward
        c = math.cos(theta)
        s = math.sin(theta)

        out = Twist()
        out.linear.x = - (c * msg.linear.x - s * msg.linear.y)
        out.linear.y = - (s * msg.linear.x + c * msg.linear.y)
        out.linear.z = msg.linear.z   # unused for ground robot, pass through anyway
        out.angular = msg.angular     # spin rate is the same in both frames
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelGimbalRotator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
