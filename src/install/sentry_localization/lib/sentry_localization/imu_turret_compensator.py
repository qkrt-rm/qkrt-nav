#!/usr/bin/env python3
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from tf2_ros import Buffer, TransformListener


def vec_add(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def vec_sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def vec_scale(v, s):
    return [v[0] * s, v[1] * s, v[2] * s]


def vec_cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def quat_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def quat_mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quat_norm(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / n, y / n, z / n, w / n)


def rotate_vec_by_quat(v, q):
    # v' = q * [v,0] * q_conj
    vq = (v[0], v[1], v[2], 0.0)
    qc = quat_conj(q)
    out = quat_mul(quat_mul(q, vq), qc)
    return [out[0], out[1], out[2]]


def normalize(v):
    n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if n <= 1e-12:
        return [0.0, 0.0, 1.0]
    return [v[0] / n, v[1] / n, v[2] / n]


class ImuTurretCompensator(Node):
    def __init__(self):
        super().__init__("imu_turret_compensator")

        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("output_topic", "/imu_base_compensated")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("yaw_joint_name", "turret_shaft_joint")
        self.declare_parameter("pitch_joint_name", "gimbal_joint")
        self.declare_parameter("yaw_axis_base", [0.0, 0.0, 1.0])
        self.declare_parameter("pitch_axis_yaw_frame", [0.0, 1.0, 0.0])

        self.imu_topic = self.get_parameter("imu_topic").value
        self.joint_states_topic = self.get_parameter("joint_states_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.yaw_joint_name = self.get_parameter("yaw_joint_name").value
        self.pitch_joint_name = self.get_parameter("pitch_joint_name").value
        self.yaw_axis_base = normalize(list(self.get_parameter("yaw_axis_base").value))
        self.pitch_axis_yaw_frame = normalize(list(self.get_parameter("pitch_axis_yaw_frame").value))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.imu_pub = self.create_publisher(Imu, self.output_topic, 20)
        self.create_subscription(Imu, self.imu_topic, self.imu_cb, 50)
        self.create_subscription(JointState, self.joint_states_topic, self.joint_state_cb, 50)

        self.joint_pos = {}
        self.joint_vel = {}
        self.last_omega_turret = [0.0, 0.0, 0.0]
        self.last_alpha_time = None
        self.warned_tf = False

        self.get_logger().info(
            f"IMU compensator started: imu_topic={self.imu_topic}, joint_states={self.joint_states_topic}, output={self.output_topic}"
        )

    def joint_state_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_pos[name] = float(msg.position[i])
            if i < len(msg.velocity):
                self.joint_vel[name] = float(msg.velocity[i])

    def _turret_omega_alpha_base(self):
        yaw_pos = self.joint_pos.get(self.yaw_joint_name, 0.0)
        yaw_vel = self.joint_vel.get(self.yaw_joint_name, 0.0)
        pitch_vel = self.joint_vel.get(self.pitch_joint_name, 0.0)

        # Approximation: pitch axis is defined in yaw frame and rotated by yaw about yaw_axis_base.
        half = yaw_pos * 0.5
        q_yaw = (
            self.yaw_axis_base[0] * math.sin(half),
            self.yaw_axis_base[1] * math.sin(half),
            self.yaw_axis_base[2] * math.sin(half),
            math.cos(half),
        )
        pitch_axis_base = rotate_vec_by_quat(self.pitch_axis_yaw_frame, q_yaw)
        omega = vec_add(
            vec_scale(self.yaw_axis_base, yaw_vel),
            vec_scale(pitch_axis_base, pitch_vel),
        )

        now = self.get_clock().now().nanoseconds
        alpha = [0.0, 0.0, 0.0]
        if self.last_alpha_time is not None:
            dt = (now - self.last_alpha_time) / 1e9
            if 1e-4 < dt < 1.0:
                alpha = vec_scale(vec_sub(omega, self.last_omega_turret), 1.0 / dt)
        self.last_alpha_time = now
        self.last_omega_turret = omega
        return omega, alpha

    def imu_cb(self, msg: Imu):
        if not msg.header.frame_id:
            self.get_logger().warn("Received IMU message without frame_id; skipping.")
            return

        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.base_frame,
                msg.header.frame_id,
                stamp,
                timeout=Duration(seconds=0.05),
            )
            self.warned_tf = False
        except Exception:
            # Fallback to latest transform if exact timestamp is unavailable.
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    msg.header.frame_id,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.05),
                )
                self.warned_tf = False
            except Exception as ex:
                if not self.warned_tf:
                    self.get_logger().warn(f"TF lookup failed for IMU compensation: {ex}")
                    self.warned_tf = True
                return

        q_bi = quat_norm((
            tf_msg.transform.rotation.x,
            tf_msg.transform.rotation.y,
            tf_msg.transform.rotation.z,
            tf_msg.transform.rotation.w,
        ))
        q_ib = quat_conj(q_bi)
        r_bi = [
            tf_msg.transform.translation.x,
            tf_msg.transform.translation.y,
            tf_msg.transform.translation.z,
        ]

        omega_turret, alpha_turret = self._turret_omega_alpha_base()

        omega_i = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ]
        omega_meas_base = rotate_vec_by_quat(omega_i, q_bi)
        omega_base = vec_sub(omega_meas_base, omega_turret)

        acc_i = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ]
        acc_meas_base = rotate_vec_by_quat(acc_i, q_bi)
        acc_rot = vec_add(
            vec_cross(alpha_turret, r_bi),
            vec_cross(omega_turret, vec_cross(omega_turret, r_bi)),
        )
        acc_base = vec_sub(acc_meas_base, acc_rot)

        q_wi = quat_norm((
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ))
        # world->base = (world->imu) * (imu->base)
        q_wb = quat_norm(quat_mul(q_wi, q_ib))

        out = Imu()
        out.header = msg.header
        out.header.frame_id = self.base_frame
        out.orientation.x = q_wb[0]
        out.orientation.y = q_wb[1]
        out.orientation.z = q_wb[2]
        out.orientation.w = q_wb[3]
        out.orientation_covariance = msg.orientation_covariance

        out.angular_velocity.x = omega_base[0]
        out.angular_velocity.y = omega_base[1]
        out.angular_velocity.z = omega_base[2]
        out.angular_velocity_covariance = msg.angular_velocity_covariance

        out.linear_acceleration.x = acc_base[0]
        out.linear_acceleration.y = acc_base[1]
        out.linear_acceleration.z = acc_base[2]
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self.imu_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuTurretCompensator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
