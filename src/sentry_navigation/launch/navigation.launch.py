import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    keepout_mask = LaunchConfiguration("keepout_mask")
    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "smoother_server",
        "bt_navigator",
        "keepout_mask_server",
        "keepout_filter_info_server",
    ]
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
        default_value="keepout_mask.yaml"
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
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        keepout_mask_arg,
        nav2_controller_server,
        nav2_planner_server,
        nav2_smoother_server,
        keepout_mask_server,
        keepout_filter_info_server,
        # nav2_behaviors,
        nav2_bt_navigator,
        nav2_lifecycle_manager,
    ])
