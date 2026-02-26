import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_localization = get_package_share_directory('sentry_localization')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_config = LaunchConfiguration('slam_config')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false'
    )

    slam_config_arg = DeclareLaunchArgument(
        'slam_config',
        default_value=os.path.join(pkg_localization, 'config', 'slam_toolbox.yaml')
    )

    # SLAM Toolbox in mapping mode. Drive the robot around to build the map.
    # When done, save the map with:
    #   ros2 run nav2_map_server map_saver_cli -f ~/my_map
    # This writes my_map.yaml and my_map.pgm. Then update sentry_map.yaml
    # to point at the new image file, and relaunch in navigation mode.
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_config_arg,
        slam_toolbox_node,
    ])
