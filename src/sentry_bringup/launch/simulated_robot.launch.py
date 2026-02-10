import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_bringup = get_package_share_directory('sentry_bringup')
    pkg_localization = get_package_share_directory('sentry_localization')
    pkg_navigation = get_package_share_directory('sentry_navigation')

    # Launch arguments
    slam = LaunchConfiguration('slam')
    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='Use SLAM for mapping. If false, use AMCL with a pre-built map.'
    )

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sentry_description"),
            "launch",
            "gazebo.launch.py"
        ),
    )

    # Laser merger node (inlined from laser_merger.launch.py)
    laser_merger = Node(
        package='sentry_bringup',
        executable='laser_merger.py',
        name='laser_merger',
        output='screen',
        parameters=[
            os.path.join(pkg_bringup, 'config', 'laser_merger.yaml'),
            {'use_sim_time': True}
        ]
    )

    global_localization = IncludeLaunchDescription(
        os.path.join(pkg_localization, "launch", "global_localization.launch.py"),
        launch_arguments={
            'use_sim_time': 'true',
            'slam': slam
        }.items()
    )

    local_localization = IncludeLaunchDescription(
        os.path.join(pkg_localization, "launch", "local_localization.launch.py"),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    navigation = IncludeLaunchDescription(
        os.path.join(pkg_navigation, "launch", "navigation.launch.py"),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    return LaunchDescription([
        slam_arg,
        gazebo,
        laser_merger,
        global_localization,
        local_localization,
        navigation,
    ])