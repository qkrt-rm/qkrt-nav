#!/usr/bin/env python3
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


WHEEL_ORDER = [
    "front_left_wheel_joint",
    "back_left_wheel_joint",
    "back_right_wheel_joint",
    "front_right_wheel_joint",
]

TURRET_YAW_JOINT = "turret_shaft_joint"
TURRET_PITCH_JOINT = "gimbal_joint"

# Keep these aligned with sentry_communication/comm_hub.py
ODOM_LABELS = [
    "wheelLF", "wheelLB", "wheelRB", "wheelRF",
    "turretYawPos", "turretYawVel", "turretPitchPos", "turretPitchVel",
    "imuAx", "imuAy", "imuAz",
    "imuGx", "imuGy", "imuGz",
    "imuYaw", "imuPitch", "imuRoll",
]
ODOM_NUM_FLOATS = len(ODOM_LABELS)

# Robot geometry for mecanum wheel odometry
WHEEL_RADIUS = 0.1
WHEEL_BASE_X = 0.2475
WHEEL_BASE_Y = 0.2475


def quaternion_to_euler_zyx(x: float, y: float, z: float, w: float):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw, pitch, roll


class SimCommHubBridge(Node):
    def __init__(self):
        super().__init__("sim_comm_hub_bridge")

        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("imu_in_topic", "/imu")
        self.declare_parameter("imu_out_topic", "/imu_comm_hub")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("publish_odom_tf", False)

        joint_states_topic = self.get_parameter("joint_states_topic").value
        imu_in_topic = self.get_parameter("imu_in_topic").value
        imu_out_topic = self.get_parameter("imu_out_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        self.publish_odom_tf = bool(self.get_parameter("publish_odom_tf").value)

        self.mcb_odom_pub = self.create_publisher(Float32MultiArray, "mcb_odom", 10)
        self.wheel_pub = self.create_publisher(Float32MultiArray, "wheel_speeds", 10)
        self.turret_pub = self.create_publisher(Float32MultiArray, "turret", 10)
        self.imu_pub = self.create_publisher(Imu, imu_out_topic, 10)
        self.publish_imu = imu_out_topic != imu_in_topic
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(JointState, joint_states_topic, self.joint_state_cb, 10)
        self.create_subscription(Imu, imu_in_topic, self.imu_cb, 10)

        self.latest_imu = Imu()
        self.latest_imu.header.frame_id = "imu_link"
        self.latest_imu_euler = (0.0, 0.0, 0.0)  # yaw, pitch, roll
        self.has_imu = False

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.last_odom_stamp_ns = None

        self.get_logger().info(
            f"Bridge started. joint_states={joint_states_topic}, imu_in={imu_in_topic}, imu_out={imu_out_topic}, "
            f"odom_topic={odom_topic}, publish_odom_tf={self.publish_odom_tf}"
        )
        if not self.publish_imu:
            self.get_logger().info("IMU output disabled because imu_out_topic matches imu_in_topic.")

    def imu_cb(self, msg: Imu):
        self.latest_imu = msg
        q = msg.orientation
        self.latest_imu_euler = quaternion_to_euler_zyx(q.x, q.y, q.z, q.w)
        self.has_imu = True

    def joint_state_cb(self, msg: JointState):
        values = [0.0] * ODOM_NUM_FLOATS

        idx = {name: i for i, name in enumerate(msg.name)}

        # wheelLF, wheelLB, wheelRB, wheelRF
        for out_i, joint_name in enumerate(WHEEL_ORDER):
            if joint_name in idx:
                j = idx[joint_name]
                if j < len(msg.velocity):
                    values[out_i] = float(msg.velocity[j])

        # turretYawPos / turretYawVel
        if TURRET_YAW_JOINT in idx:
            j = idx[TURRET_YAW_JOINT]
            if j < len(msg.position):
                values[4] = float(msg.position[j])
            if j < len(msg.velocity):
                values[5] = float(msg.velocity[j])

        # turretPitchPos / turretPitchVel
        if TURRET_PITCH_JOINT in idx:
            j = idx[TURRET_PITCH_JOINT]
            if j < len(msg.position):
                values[6] = float(msg.position[j])
            if j < len(msg.velocity):
                values[7] = float(msg.velocity[j])

        if self.has_imu:
            imu = self.latest_imu
            values[8] = float(imu.linear_acceleration.x)
            values[9] = float(imu.linear_acceleration.y)
            values[10] = float(imu.linear_acceleration.z)
            values[11] = float(imu.angular_velocity.x)
            values[12] = float(imu.angular_velocity.y)
            values[13] = float(imu.angular_velocity.z)
            values[14], values[15], values[16] = self.latest_imu_euler

        self.publish_mcb_odom(values)
        self.publish_wheels(values[0:4])
        self.publish_turret(values[4:8])
        if self.has_imu and self.publish_imu:
            self.imu_pub.publish(self.latest_imu)
        self.publish_odometry(values[0:4], values[14], msg.header.stamp)

    def publish_mcb_odom(self, values):
        msg = Float32MultiArray()
        msg.layout = MultiArrayLayout(
            dim=[MultiArrayDimension(label="odom", size=ODOM_NUM_FLOATS, stride=ODOM_NUM_FLOATS)],
            data_offset=0,
        )
        msg.data = values
        self.mcb_odom_pub.publish(msg)

    def publish_wheels(self, wheel_values):
        msg = Float32MultiArray()
        msg.layout = MultiArrayLayout(
            dim=[MultiArrayDimension(label="wheels", size=4, stride=4)],
            data_offset=0,
        )
        msg.data = list(wheel_values)
        self.wheel_pub.publish(msg)

    def publish_turret(self, turret_values):
        msg = Float32MultiArray()
        msg.layout = MultiArrayLayout(
            dim=[MultiArrayDimension(label="turret", size=4, stride=4)],
            data_offset=0,
        )
        msg.data = list(turret_values)
        self.turret_pub.publish(msg)

    def publish_odometry(self, wheel_speeds, imu_yaw, stamp):
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if stamp_ns <= 0:
            stamp_ns = self.get_clock().now().nanoseconds

        if self.last_odom_stamp_ns is None:
            self.last_odom_stamp_ns = stamp_ns
            return

        dt = (stamp_ns - self.last_odom_stamp_ns) / 1e9
        self.last_odom_stamp_ns = stamp_ns
        if dt <= 0.0 or dt > 1.0:
            return

        w_lf, w_lb, w_rb, w_rf = wheel_speeds
        r = WHEEL_RADIUS
        l_sum = WHEEL_BASE_X + WHEEL_BASE_Y

        vx = (w_lf + w_rf + w_lb + w_rb) * r / 4.0
        vy = (-w_lf + w_rf + w_lb - w_rb) * r / 4.0
        omega = (-w_lf + w_rf - w_lb + w_rb) * r / (4.0 * l_sum)

        self.odom_theta = imu_yaw
        cos_theta = math.cos(self.odom_theta)
        sin_theta = math.sin(self.odom_theta)
        self.odom_x += (vx * cos_theta - vy * sin_theta) * dt
        self.odom_y += (vx * sin_theta + vy * cos_theta) * dt

        now = self.get_clock().now().to_msg()

        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.odom_x
        odom_msg.pose.pose.position.y = self.odom_y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.odom_theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.odom_theta / 2.0)

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = omega

        odom_msg.pose.covariance[0] = 0.01
        odom_msg.pose.covariance[7] = 0.01
        odom_msg.pose.covariance[35] = 0.01
        odom_msg.twist.covariance[0] = 0.01
        odom_msg.twist.covariance[7] = 0.01
        odom_msg.twist.covariance[35] = 0.01
        self.odom_pub.publish(odom_msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation.x = self.odom_x
        tf_msg.transform.translation.y = self.odom_y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = odom_msg.pose.pose.orientation
        if self.publish_odom_tf:
            self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimCommHubBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
