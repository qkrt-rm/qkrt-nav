import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    keepout_mask = LaunchConfiguration("keepout_mask")
    use_keepout = LaunchConfiguration("use_keepout")
    use_battery_mission = LaunchConfiguration("use_battery_mission")
    center_x = LaunchConfiguration("center_x")
    center_y = LaunchConfiguration("center_y")
    lifecycle_nodes_base = [
        "controller_server",
        "planner_server",
        "smoother_server",
        "bt_navigator",
    ]
    lifecycle_nodes_keepout = [
        "keepout_mask_server",
        "keepout_filter_info_server",
    ]
    lifecycle_nodes = lifecycle_nodes_base + lifecycle_nodes_keepout
    # lifecycle_nodes = ["controller_server", "planner_server", "smoother_server", "bt_navigator", "behavior_server"
    sentry_navigation_pkg = get_package_share_directory("sentry_navigation")
    sentry_localization_pkg = get_package_share_directory("sentry_localization")
    bt_xml_path = os.path.join(
                sentry_navigation_pkg,
                "behavior_tree",
                "simple_navigation.xml") # This is for testing
    keepout_mask_path = PathJoinSubstitution([sentry_localization_pkg, "maps", keepout_mask])

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false"
    )

    keepout_mask_arg = DeclareLaunchArgument(
        "keepout_mask",
        default_value="ARCC20263v3Field_keepout.yaml"
    )

    use_battery_mission_arg = DeclareLaunchArgument(
        "use_battery_mission",
        default_value="false",
        description="Launch battery mission controller and dummy battery publisher."
    )

    center_x_arg = DeclareLaunchArgument(
        "center_x",
        default_value="6.0",
        description="X coordinate of arena center in map frame."
    )

    center_y_arg = DeclareLaunchArgument(
        "center_y",
        default_value="4.0",
        description="Y coordinate of arena center in map frame."
    )

    use_keepout_arg = DeclareLaunchArgument(
        "use_keepout",
        default_value="false",
        description="Enable arena keepout filter (starts mask servers + activates the "
                    "costmap filter). Set true when running in the arena."
    )

    nav2_controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        output="screen",
        parameters=[
            os.path.join(
                sentry_navigation_pkg,
                "config",
                "controller_server.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )
    
    nav2_planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            os.path.join(
                sentry_navigation_pkg,
                "config",
                "planner_server.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )

    # nav2_behaviors = Node(
    #     package="nav2_behaviors",
    #     executable="behavior_server",
    #     name="behavior_server",
    #     output="screen",
    #     parameters=[
    #         os.path.join(
    #             sentry_navigation_pkg,
    #             "config",
    #             "behavior_server.yaml"),
    #         {"use_sim_time": use_sim_time}
    #     ],
    # )
    nav2_bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            os.path.join(
                sentry_navigation_pkg,
                "config",
                "bt_navigator.yaml"
            ),
            {'use_sim_time': use_sim_time},
            {'default_nav_to_pose_bt_xml': bt_xml_path},
            {'default_nav_through_poses_bt_xml': bt_xml_path}
        ]
    )

    nav2_smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[
            os.path.join(
                sentry_navigation_pkg,
                "config",
                "smoother_server.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )

    keepout_mask_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="keepout_mask_server",
        output="screen",
        parameters=[
            {"yaml_filename": keepout_mask_path},
            {"topic_name": "keepout_mask"},
            {"frame_id": "map"},
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(use_keepout),
    )

    keepout_filter_info_server = Node(
        package="nav2_map_server",
        executable="costmap_filter_info_server",
        name="keepout_filter_info_server",
        output="screen",
        parameters=[
            {"filter_info_topic": "/costmap_filter_info"},
            {"mask_topic": "/keepout_mask"},
            {"type": 0},
            {"base": 0.0},
            {"multiplier": 1.0},
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(use_keepout),
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"bond_timeout": 30.0}
        ],
        condition=IfCondition(use_keepout),
    )

    nav2_lifecycle_manager_no_keepout = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation_base",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes_base},
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"bond_timeout": 30.0}
        ],
        condition=UnlessCondition(use_keepout),
    )

    battery_health_publisher = Node(
        package='sentry_navigation',
        executable='battery_health_publisher.py',
        name='battery_health_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_battery_mission)
    )

    # Cancel any goal left over from a previous session (e.g. a stale RViz "2D Nav
    # Goal" click, or a mission node that was killed mid-navigation) so a fresh
    # bt_navigator never picks up and immediately drives toward an old target.
    # A zero goal_id + zero stamp is the action protocol's "cancel all goals"
    # request. `ros2 service call` blocks until the service appears, so this
    # fires as soon as bt_navigator's action server comes up.
    cancel_stale_nav_goal = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call',
                     '/navigate_to_pose/_action/cancel_goal',
                     'action_msgs/srv/CancelGoal',
                     '{goal_info: {goal_id: {uuid: '
                     '[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}, '
                     'stamp: {sec: 0, nanosec: 0}}}'],
                output='screen',
            )
        ],
    )

    battery_mission_controller = Node(
        package='sentry_navigation',
        executable='battery_mission_controller.py',
        name='battery_mission_controller',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time, 'map_frame': 'odom', 'base_frame': 'base_link'}, #TODO: changed last minute
            {'center_x': center_x},
            {'center_y': center_y},
        ],
        condition=IfCondition(use_battery_mission)
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_battery_mission_arg,
        center_x_arg,
        center_y_arg,
        keepout_mask_arg,
        use_keepout_arg,
        nav2_controller_server,
        nav2_planner_server,
        nav2_smoother_server,
        keepout_mask_server,
        keepout_filter_info_server,
        # nav2_behaviors,
        nav2_bt_navigator,
        nav2_lifecycle_manager,
        nav2_lifecycle_manager_no_keepout,
        battery_health_publisher,
        battery_mission_controller,
        cancel_stale_nav_goal,
    ])
