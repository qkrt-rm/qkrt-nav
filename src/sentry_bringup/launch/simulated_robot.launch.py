import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_bringup = get_package_share_directory('sentry_bringup')
    pkg_description = get_package_share_directory('sentry_description')
    pkg_localization = get_package_share_directory('sentry_localization')

    slam = LaunchConfiguration('slam')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    robot_model = LaunchConfiguration('robot_model')
    world = LaunchConfiguration('world')

    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='Use SLAM for mapping. If false, use AMCL with a pre-built map.'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock from /clock.'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='sentry_map.yaml'
    )

    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='sentry_description_v2.urdf.xacro'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='comp_map.sdf'
    )

    gazebo = IncludeLaunchDescription(
        os.path.join(
            pkg_description,
            'launch',
            'gazebo.launch.py'
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'robot_model': robot_model,
            'world': world
        }.items()
    )

    # Global localization (SLAM or AMCL based on slam argument)
    global_localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory('sentry_localization'),
            'launch',
            'global_localization.launch.py'
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': slam,
            'map': map_yaml
        }.items()
    )

    # Include EKF in simulation to mirror real-robot localization behavior.
    local_localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory('sentry_localization'),
            'launch',
            'local_localization.launch.py'
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory('sentry_navigation'),
            'launch',
            'navigation.launch.py'
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    laser_merger = Node(
        package='sentry_bringup',
        executable='laser_merger.py',
        name='laser_merger',
        output='screen',
        parameters=[
            os.path.join(pkg_bringup, 'config', 'laser_merger.yaml'),
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        slam_arg,
        use_sim_time_arg,
        map_arg,
        robot_model_arg,
        world_arg,
        gazebo,
        laser_merger,
        global_localization,
        local_localization,
        navigation,
    ])
