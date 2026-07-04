import os
from launch import LaunchDescription
from launch.actions import TimerAction
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
    use_battery_mission = LaunchConfiguration('use_battery_mission')
    center_x = LaunchConfiguration('center_x')
    center_y = LaunchConfiguration('center_y')
    use_keepout = LaunchConfiguration('use_keepout')

    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='true',
        description='Use SLAM for mapping. If false, use AMCL with a pre-built map.'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock from /clock.'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='ARCC20263v3FieldCleaned.yaml'
    )

    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='sentry_description.urdf.xacro'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='comp_map.sdf'
    )

    use_battery_mission_arg = DeclareLaunchArgument(
        'use_battery_mission',
        default_value='false',
        description='Launch battery mission controller and dummy battery publisher.'
    )

    center_x_arg = DeclareLaunchArgument(
        'center_x',
        default_value='0.0',
        description='X coordinate of arena center in map frame.'
    )

    center_y_arg = DeclareLaunchArgument(
        'center_y',
        default_value='0.0',
        description='Y coordinate of arena center in map frame.'
    )

    use_keepout_arg = DeclareLaunchArgument(
        'use_keepout',
        default_value='false',
        description='Enable arena keepout filter in navigation. Set true in the arena.'
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
            'map': map_yaml,
            # Sim spawns the robot at a known pose, so seed AMCL instead of scattering.
            # Keep these in sync with the spawn pose in gazebo.launch.py (-x -y -Y).
            'seed_initial_pose': 'true',
            'initial_pose_x': '0.0',
            'initial_pose_y': '0.0',
            'initial_pose_yaw': '0.0',
        }.items()
    )

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
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_battery_mission': use_battery_mission,
            'center_x': center_x,
            'center_y': center_y,
            'use_keepout': use_keepout,
        }.items()
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
        use_battery_mission_arg,
        center_x_arg,
        center_y_arg,
        use_keepout_arg,
        gazebo,
        TimerAction(
            period=5.0,
            actions=[
                laser_merger,
                global_localization,
                local_localization,
                navigation,
            ],
        ),
    ])
