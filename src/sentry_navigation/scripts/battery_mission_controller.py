#!/usr/bin/env python3
#
# BATTERY MISSION CONTROLLER
# ==========================
# Simple rule: if the battery is above BATTERY_THRESHOLD, drive to the arena
# centre. Otherwise, drive back to the spawn (home) position captured at startup.
#
# HOME POSITION
# -------------
# Captured from the map->base_link TF at startup (assumes the robot starts at
# home). No separate home_x/home_y parameters.
#
# WHERE TO PLUG IN REAL BATTERY DATA
# -----------------------------------
# Replace the battery_health_publisher.py dummy node with whatever reads your
# real battery. It should publish a std_msgs/Float32 to /battery_health with a
# value between 0.0 (empty) and 100.0 (full). Nothing else needs to change.
#
# TUNING
# ------
# center_x / center_y   ->  set these in the launch file to match your AMCL map
# BATTERY_THRESHOLD     ->  battery percentage cutoff between centre and home
# HOME_CAPTURE_DELAY_S  ->  how long to wait before locking in the home position

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from nav2_msgs.action import NavigateToPose


# How long after startup to wait before capturing the home position.
# Needs to be long enough for SLAM/AMCL to produce a valid map->base_link TF.
HOME_CAPTURE_DELAY_S = 5.0

# Battery health from the MCB is a uint16 (0-200). Above this the robot goes to
# centre; at or below it retreats home. So at 151 it retreats.
BATTERY_THRESHOLD = 151.0

import tf2_ros


class BatteryMissionController(Node):
    def __init__(self):
        super().__init__('battery_mission_controller')

        # These are set in the launch file — no need to touch this code
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        self.center_x = self.get_parameter('center_x').value
        self.center_y = self.get_parameter('center_y').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # home_pose is None until we successfully read the TF at startup
        self.home_pose = None
        # 'center' or 'home' — whichever goal we currently have active, so we
        # don't spam nav2 with a fresh goal on every battery message.
        self._target = None

        self._goal_handle = None

        # TF listener so we can read where the robot is in the map
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Action client that sends goals to nav2's navigation server
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # /battery_health carries a Float32 between 0 and 100.
        self.create_subscription(Float32, '/battery_health', self._battery_cb, 10)

        # Keep retrying the home capture every HOME_CAPTURE_DELAY_S seconds
        # until the map TF is available. Timer cancels itself once home is locked.
        self._home_capture_timer = self.create_timer(HOME_CAPTURE_DELAY_S, self._capture_home_once)

        # Resend the current target goal every second regardless of whether the
        # target changed, so a dropped/rejected/aborted goal gets retried instead
        # of silently stalling until the battery crosses the threshold again.
        self._resend_timer = self.create_timer(1.0, self._send_current_goal)

    # ------------------------------------------------------------------
    # STARTUP: lock in the home position

    def _capture_home_once(self):
        # Try to read where the robot is right now in the map frame.
        # This will fail (and we'll retry) until SLAM/AMCL has a fix.
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame,
                rclpy.time.Time(), timeout=Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().warn(f'Home capture failed (map not ready yet), retrying: {e}')
            return

        # Got a valid position — save it and stop retrying
        self._home_capture_timer.cancel()
        self.home_pose = self._tf_to_pose(tf)
        self.get_logger().info(
            f'Home captured: ({self.home_pose.pose.position.x:.2f}, '
            f'{self.home_pose.pose.position.y:.2f})')

    # ------------------------------------------------------------------
    # BATTERY LOGIC: called every time /battery_health publishes

    def _battery_cb(self, msg: Float32):
        self.get_logger().info(f"received battery info: {msg.data}")

        if self.home_pose is None:
            return  # still capturing home; mission hasn't started yet

        target = 'center' if msg.data > BATTERY_THRESHOLD else 'home'
        if target == self._target:
            return  # already heading there; nothing to do

        self._target = target
        if target == 'center':
            self.get_logger().info(
                f'Battery {msg.data:.1f}% > {BATTERY_THRESHOLD}% -> going to centre')
        else:
            self.get_logger().info(
                f'Battery {msg.data:.1f}% <= {BATTERY_THRESHOLD}% -> going home')
        # A target change preempts whatever nav2 is currently doing.
        self._cancel_current_goal()
        self._send_current_goal()

    def _send_current_goal(self):
        # Also runs on a 1s timer so a rejected/aborted/completed goal gets
        # retried instead of stalling until the battery crosses the threshold
        # again. Skips while a goal is still actively executing, so this
        # doesn't cancel-and-replan a goal that's already under way.
        if self._goal_handle is not None:
            return
        if self._target == 'center':
            self._send_goal(self._pose_at(self.center_x, self.center_y))
        elif self._target == 'home':
            self._send_goal(self.home_pose)

    def _cancel_current_goal(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    # ------------------------------------------------------------------
    # NAV2 COMMUNICATION: send goals

    def _send_goal(self, pose: PoseStamped):
        if not self._nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('NavigateToPose action server not available')
            return

        # Update the pose timestamp to the exact current time
        # This stops the planner from looking up transforms at t=0
        pose.header.stamp = self.get_clock().now().to_msg()

        goal = NavigateToPose.Goal()
        goal.pose = pose

        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_accepted_cb)

    def _goal_accepted_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by nav2')
            return
        self._goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        # Goal finished (succeeded, aborted, or canceled) — clear the handle so
        # the next resend tick or target change can send a fresh goal instead
        # of thinking one is still in flight.
        self._goal_handle = None

    # ------------------------------------------------------------------
    # HELPERS: build PoseStamped messages

    def _pose_at(self, x: float, y: float) -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id = self.map_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.orientation.w = 1.0  # face forward (no specific heading required)
        return p

    def _tf_to_pose(self, tf) -> PoseStamped:
        # Convert a TF transform into a PoseStamped the action client can use
        p = PoseStamped()
        p.header.frame_id = self.map_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = tf.transform.translation.x
        p.pose.position.y = tf.transform.translation.y
        p.pose.position.z = 0.0
        p.pose.orientation = tf.transform.rotation
        return p


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
