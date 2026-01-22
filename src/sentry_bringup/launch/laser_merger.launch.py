import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_bringup = get_package_share_directory('sentry_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )

    laser_merger_node = Node(
        package='sentry_bringup',
        executable='laser_merger.py',
        name='laser_merger',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'destination_frame': 'base_link'},
            {'scan_destination_topic': '/scan'},
            {'angle_min': -3.14159},
            {'angle_max': 3.14159},
            {'angle_increment': 0.01},
            {'range_min': 0.1},
            {'range_max': 10.0},
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        laser_merger_node,
    ])
