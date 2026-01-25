"""
Minimal launch file for testing MCB communication.
- Launches sensor_publisher to receive data from MCB (publishes /odom, /imu)
- Launches comm_hub to send velocity commands to MCB
- Launches keyboard teleop for control

NOTE: Both sensor_publisher and comm_hub use /dev/ttyTHS1 at 115200 baud.
This is bidirectional communication on a single UART.

Usage:
  ros2 launch sentry_bringup minimal_mcb_test.launch.py

Echo sensor data:
  ros2 topic echo /odom
  ros2 topic echo /imu
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_drivers = get_package_share_directory('sentry_drivers')

    # Launch arguments
    serial_port = LaunchConfiguration('serial_port')
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyTHS1',
        description='Serial port for MCB communication'
    )

    # Sensor publisher - reads sensor data from MCB
    sensor_publisher = Node(
        package='sentry_drivers',
        executable='sensor_publisher.py',
        name='sensor_publisher',
        output='screen',
        parameters=[
            os.path.join(pkg_drivers, 'config', 'sensor_publisher.yaml'),
            {'serial_port': serial_port}
        ]
    )

    # Comm hub - sends velocity commands to MCB
    comm_hub = Node(
        package='sentry_communication',
        executable='comm_hub',
        name='comm_hub',
        output='screen'
    )

    # Keyboard teleop (opens in new xterm window)
    keyboard_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
    )

    return LaunchDescription([
        serial_port_arg,
        sensor_publisher,
        comm_hub,
        keyboard_teleop,
    ])
