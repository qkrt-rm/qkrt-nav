#!/usr/bin/env python3
#
# BATTERY MISSION CONTROLLER (finite state machine)
# ===================================================
# States
# -------
#   CAPTURING_HOME  Waiting for the initial map->base_link TF. Assumes the robot is
#                   physically placed at "home" before this node starts.
#   NAV_TO_CENTER   Battery is healthy -- drive to the arena centre.
#   STRAFING        At the centre, ping-ponging sideways (+/-0.25m about centre_y) for
#                   as long as the battery stays healthy.
#   NAV_TO_HOME     Battery dropped below threshold -- return home.
#   WAIT_AT_HOME    Reached home successfully -- hold position for HOME_WAIT_S seconds
#                   (time for a human to swap/refill the battery), then head back out.
#
# Battery readings only ever trigger the NAV_TO_CENTER/STRAFING -> NAV_TO_HOME
# transition. Once heading home or waiting at home, the FSM ignores battery readings
# until it's back at centre -- that's what makes this a state machine instead of a
# reactive "go wherever the battery says right now" controller like the old version.
#
# STRAFE PATTERN
# --------------
# On arrival at centre the robot moves to (centre_y + 0.25). From there each
# subsequent waypoint flips sides, i.e. -0.25 (a 0.5m move), then +0.25 (a 0.5m move),
# and so on -- the classic up-0.25/down-0.5/up-0.5/... ping-pong.
#
# HOME POSITION
# -------------
# Captured from the map->base_link TF at startup (assumes the robot starts at home --
# no separate home_x/home_y parameters). For competition, place the robot near the
# left- or right-most point along the middle of the arena (y ~ 4.1) before launch --
# whichever side matches your team colour, since that isn't known ahead of time.
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
# center_x / center_y   ->  set these in the launch file to match your AMCL map
# BATTERY_THRESHOLD     ->  change the 50.0 below if you want a different cutoff
# HOME_CAPTURE_DELAY_S  ->  how long to wait before locking in the home position
# STRAFE_AMPLITUDE_M    ->  how far off centre_y each strafe waypoint sits
# HOME_WAIT_S           ->  how long to hold at home after a successful arrival

import enum

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
# Needs to be long enough for SLAM/AMCL to produce a valid map->base_link TF.
HOME_CAPTURE_DELAY_S = 5.0

# Below this percentage the robot goes home. At or above it goes to/stays at centre.
BATTERY_THRESHOLD = 50.0

# Half the total strafe travel -- first move off centre is this far, every move after
# that is 2x this (peak to peak).
STRAFE_AMPLITUDE_M = 0.25

# How long to sit at home after a successful arrival before heading back to centre.
HOME_WAIT_S = 5.0


class State(enum.Enum):
    CAPTURING_HOME = enum.auto()
    NAV_TO_CENTER = enum.auto()
    STRAFING = enum.auto()
    NAV_TO_HOME = enum.auto()
    WAIT_AT_HOME = enum.auto()


class BatteryMissionController(Node):
    def __init__(self):
        super().__init__('battery_mission_controller')

        # These are set in the launch file — no need to touch this code
        self.declare_parameter('center_x', 3.0)
        self.declare_parameter('center_y', 0.0)
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

        self.state = State.CAPTURING_HOME
        # True -> next strafe waypoint is +STRAFE_AMPLITUDE_M off centre_y, False -> -.
        # Reset to True every time we (re)arrive at centre from NAV_TO_CENTER.
        self._strafe_up_next = True

        # handle to the active nav2 goal so we can cancel it if needed
        self._goal_handle = None
        self._current_goal_pose = None
        # Bumped every time a new goal is sent. Callbacks compare against this to
        # detect they've been superseded (goal cancelled, state changed) and bail out
        # instead of acting on stale results.
        self._goal_generation = 0

        self._home_wait_timer = None

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

        # Mission starts here: head to centre. If the battery is actually already low,
        # the next /battery_health reading will redirect us home mid-transit.
        self._enter_state(State.NAV_TO_CENTER)

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
        if self.home_pose is None:
            return  # still capturing home; mission hasn't started yet

        if self.battery < BATTERY_THRESHOLD and self.state in (State.NAV_TO_CENTER, State.STRAFING):
            self.get_logger().info(
                f'Battery {self.battery:.1f}% < {BATTERY_THRESHOLD}% -> heading home')
            self._enter_state(State.NAV_TO_HOME)

    # ------------------------------------------------------------------
    # STATE MACHINE

    def _enter_state(self, new_state: State):
        self.state = new_state
        self.get_logger().info(f'State -> {new_state.name}')

        if new_state == State.NAV_TO_CENTER:
            self._send_goal(self._pose_at(self.center_x, self.center_y))
        elif new_state == State.STRAFING:
            self._send_goal(self._next_strafe_pose())
        elif new_state == State.NAV_TO_HOME:
            self._send_goal(self.home_pose)
        elif new_state == State.WAIT_AT_HOME:
            self._home_wait_timer = self.create_timer(HOME_WAIT_S, self._home_wait_done)

    def _home_wait_done(self):
        self._home_wait_timer.cancel()
        self._home_wait_timer = None
        self._strafe_up_next = True  # first move after centre is always +STRAFE_AMPLITUDE_M
        self._enter_state(State.NAV_TO_CENTER)

    def _next_strafe_pose(self) -> PoseStamped:
        offset = STRAFE_AMPLITUDE_M if self._strafe_up_next else -STRAFE_AMPLITUDE_M
        self._strafe_up_next = not self._strafe_up_next
        return self._pose_at(self.center_x, self.center_y + offset)

    # ------------------------------------------------------------------
    # NAV2 COMMUNICATION: send/cancel goals and handle results

    def _send_goal(self, pose: PoseStamped):
        if not self._nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('NavigateToPose action server not available')
            return

        # Cancel whatever nav2 is currently doing before sending the new goal
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

        # Update the pose timestamp to the exact current time
        # This stops the planner from looking up transforms at t=0
        pose.header.stamp = self.get_clock().now().to_msg()

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self._current_goal_pose = pose
        self._goal_generation += 1
        my_generation = self._goal_generation

        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda f, gen=my_generation: self._goal_accepted_cb(f, gen))

    def _goal_accepted_cb(self, future, generation):
        if generation != self._goal_generation:
            return  # superseded before nav2 even accepted it
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().warn('Goal rejected by nav2')
            return
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, gen=generation: self._goal_result_cb(f, gen))

    def _goal_result_cb(self, future, generation):
        if generation != self._goal_generation:
            return  # stale result from a goal we've since cancelled/superseded
        self._goal_handle = None
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Reached goal for {self.state.name}')
            if self.state == State.NAV_TO_CENTER:
                self._strafe_up_next = True
                self._enter_state(State.STRAFING)
            elif self.state == State.STRAFING:
                self._enter_state(State.STRAFING)  # sends the next ping-pong waypoint
            elif self.state == State.NAV_TO_HOME:
                self._enter_state(State.WAIT_AT_HOME)
        else:
            self.get_logger().warn(
                f'Goal for {self.state.name} failed (status {status}), retrying')
            self._send_goal(self._current_goal_pose)

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
