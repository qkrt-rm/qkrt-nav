import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    pkg_localization = get_package_share_directory('sentry_localization')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    map_yaml = PathJoinSubstitution([pkg_localization, 'config', LaunchConfiguration('map')])
    amcl_config = LaunchConfiguration('amcl_config')
    slam_config = LaunchConfiguration('slam_config')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false'
    )

    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='false'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='sentry_map.yaml'
    )

    amcl_config_arg = DeclareLaunchArgument(
        'amcl_config',
        default_value=os.path.join(pkg_localization, 'config', 'amcl.yaml')
    )

    slam_config_arg = DeclareLaunchArgument(
        'slam_config',
        default_value=os.path.join(pkg_localization, 'config', 'slam_toolbox.yaml')
    )

    # SLAM Toolbox in mapping mode.
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(slam)
    )

    # Map server - used when slam=false to serve a pre-built map
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_yaml},
            {'use_sim_time': use_sim_time}
        ],
        condition=UnlessCondition(slam)
    )

    # AMCL localizes the robot within the static map using particle filtering
    # against live lidar scans. It publishes the map->odom TF.
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_config,
            {'use_sim_time': use_sim_time}
        ],
        condition=UnlessCondition(slam)
    )

    # Lifecycle manager brings up map_server and amcl in the correct order.
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'node_names': ['map_server', 'amcl']},
            {'use_sim_time': use_sim_time},
            {'autostart': True}
        ],
        condition=UnlessCondition(slam)
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_arg,
        map_arg,
        amcl_config_arg,
        slam_config_arg,
        slam_toolbox_node,
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
    ])
