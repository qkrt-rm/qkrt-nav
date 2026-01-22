import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_drivers = get_package_share_directory('sentry_drivers')

    serial_port = LaunchConfiguration('serial_port')
    publish_tf = LaunchConfiguration('publish_tf')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyTHS1'
    )

    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true'
    )

    sensor_publisher_node = Node(
        package='sentry_drivers',
        executable='sensor_publisher.py',
        name='sensor_publisher',
        output='screen',
        parameters=[
            os.path.join(pkg_drivers, 'config', 'sensor_publisher.yaml'),
            {'serial_port': serial_port},
            {'publish_tf': publish_tf},
        ]
    )

    return LaunchDescription([
        serial_port_arg,
        publish_tf_arg,
        sensor_publisher_node,
    ])
