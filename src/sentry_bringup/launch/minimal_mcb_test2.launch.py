"""
Minimal launch file for testing MCB communication.
- Launches comm_hub to send velocity commands to MCB and receive odom data
- Launches keyboard teleop for control

NOTE: comm_hub uses /dev/ttyTHS1 at 115200 baud.

Usage:
  ros2 launch sentry_bringup minimal_mcb_test.launch.py

Echo sensor data:
  ros2 topic echo /mcb_odom
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Comm hub - sends velocity commands to MCB and receives odom data
    threaded_blocking = Node(
        package='sentry_communication',
        executable='threaded_blocking',
        name='threaded_blocking',
        output='screen'
    )


    return LaunchDescription([
        threaded_blocking
    ])
