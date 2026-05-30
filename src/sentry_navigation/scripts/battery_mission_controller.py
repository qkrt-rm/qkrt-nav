#!/usr/bin/env python3
#
# BATTERY MISSION CONTROLLER
# ===========================
# This node watches the robot's battery level and decides where it should go:
#
#   Battery >= 50%  →  drive to the centre of the arena
#   Battery <  50%  →  drive back to where it started
#
# HOW IT WORKS
# ------------
# 1. At startup it waits for the map → base_link transform to exist
#    (meaning SLAM/AMCL has initialised and knows where the robot is).
#    Once that's available it records the robot's current position as "home".
#
# 2. It then listens to the /battery_health topic (a Float32, range 0–100).
#    Every time a new reading arrives it decides: go to centre or go home?
#    If the destination hasn't changed it does nothing and lets the robot keep
#    going. If it HAS changed it cancels the current nav2 goal and sends a new one.
#
# 3. It talks to nav2 through the NavigateToPose action. Nav2 handles all the
#    actual path planning and obstacle avoidance — this node just says "go here".
#
# WHERE TO PLUG IN REAL BATTERY DATA
# -----------------------------------
# Replace the battery_health_publisher.py dummy node with whatever reads your
# real battery. Your real battery node should publish a std_msgs/Float32 to the
# topic /battery_health with a value between 0.0 (empty) and 100.0 (full).
# Nothing else needs to change.
#
# TUNING
# ------
# center_x / center_y  →  set these in the launch file to match your AMCL map
# BATTERY_THRESHOLD     →  change the 50.0 below if you want a different cutoff
# HOME_CAPTURE_DELAY_S  →  how long to wait before locking in the home position

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

import tf2_ros


# How long after startup to wait before capturing the home position.
# Needs to be long enough for SLAM/AMCL to produce a valid map→base_link TF.
HOME_CAPTURE_DELAY_S = 5.0

# Below this percentage the robot goes home. At or above it goes to centre.
BATTERY_THRESHOLD = 50.0


class BatteryMissionController(Node):
    def __init__(self):
        super().__init__('battery_mission_controller')

        # These are set in the launch file — no need to touch this code
        self.declare_parameter('center_x', 6.0)
        self.declare_parameter('center_y', 4.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        self.center_x = self.get_parameter('center_x').value
        self.center_y = self.get_parameter('center_y').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # home_pose is None until we successfully read the TF at startup
        self.home_pose = None
        # last battery reading we received (None until first message arrives)
        self.battery = None
        # which destination the robot is currently heading to ('center' or 'home')
        self.current_target = None
        # handle to the active nav2 goal so we can cancel it if needed
        self._goal_handle = None
        self._navigating = False

        # TF listener so we can read where the robot is in the map
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Action client that sends goals to nav2's navigation server
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # /battery_health carries a Float32 between 0 and 100.
        # REPLACE battery_health_publisher.py with your real battery node —
        # just make sure it publishes to this same topic with the same type.
        self.create_subscription(Float32, '/battery_health', self._battery_cb, 10)

        # Keep retrying the home capture every HOME_CAPTURE_DELAY_S seconds
        # until the map TF is available. Timer cancels itself once home is locked.
        self._home_capture_timer = self.create_timer(HOME_CAPTURE_DELAY_S, self._capture_home_once)

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

        # If a battery reading already came in while we were waiting, act on it now
        if self.battery is not None:
            self._evaluate()

    # ------------------------------------------------------------------
    # BATTERY LOGIC: called every time /battery_health publishes

    def _battery_cb(self, msg: Float32):
        # ---------------------------------------------------------------
        # TO INTEGRATE REAL BATTERY DATA:
        # Delete battery_health_publisher.py and replace it with a node that
        # reads your actual battery sensor and publishes here.
        # This callback doesn't care where the number came from.
        # ---------------------------------------------------------------
        self.battery = msg.data
        if self.home_pose is not None:
            self._evaluate()

    def _evaluate(self):
        # Decide where we should be going based on current battery level
        desired = 'center' if self.battery >= BATTERY_THRESHOLD else 'home'

        # Don't interrupt if we're already going to the right place
        if desired == self.current_target and self._navigating:
            return

        self.get_logger().info(f'Battery {self.battery:.1f}% → targeting {desired}')
        self.current_target = desired
        goal_pose = self._center_pose() if desired == 'center' else self.home_pose
        self._send_goal(goal_pose)

    # ------------------------------------------------------------------
    # NAV2 COMMUNICATION: send/cancel goals and handle results

    def _send_goal(self, pose: PoseStamped):
        if not self._nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('NavigateToPose action server not available')
            return

        # Cancel whatever nav2 is currently doing before sending the new goal
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self._navigating = True
        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._goal_accepted_cb)

    def _goal_accepted_cb(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().warn('Goal rejected by nav2')
            self._navigating = False
            return
        # Goal is running — register a callback for when it finishes
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        self._navigating = False
        self._goal_handle = None
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Reached {self.current_target}')
        else:
            # Nav2 failed or was cancelled — we'll try again on the next battery update
            self.get_logger().warn(
                f'Goal to {self.current_target} failed (status {status}), '
                f'will retry on next battery update')

    # ------------------------------------------------------------------
    # HELPERS: build PoseStamped messages

    def _center_pose(self) -> PoseStamped:
        # Arena centre — coordinates come from launch args (center_x, center_y)
        p = PoseStamped()
        p.header.frame_id = self.map_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = self.center_x
        p.pose.position.y = self.center_y
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
