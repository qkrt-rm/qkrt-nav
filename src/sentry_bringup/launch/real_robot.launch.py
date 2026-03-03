import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_bringup = get_package_share_directory('sentry_bringup')
    pkg_description = get_package_share_directory('sentry_description')
    pkg_localization = get_package_share_directory('sentry_localization')
    pkg_navigation = get_package_share_directory('sentry_navigation')

    # Launch arguments
    slam = LaunchConfiguration("slam")
    slam_arg = DeclareLaunchArgument(
        "slam",
        default_value="false",
        description="Use SLAM for mapping. If false, use AMCL with a pre-built map."
    )

    map_yaml = LaunchConfiguration("map")
    map_arg = DeclareLaunchArgument(
        "map",
        default_value='sentry_map.yaml',
        description="Map filename in sentry_localization/config."
    )

    robot_model = LaunchConfiguration("robot_model")
    robot_model_path = PathJoinSubstitution([pkg_description, 'urdf', robot_model])
    robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value='sentry_description.urdf',
        description="Robot model filename in sentry_description/urdf."
    )

    use_nav = LaunchConfiguration("use_nav")
    use_nav_arg = DeclareLaunchArgument(
        "use_nav",
        default_value="true",
        description="Launch navigation stack"
    )

    # Robot state publisher (publishes URDF transforms: base_link → laser frames, etc.)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', robot_model_path]),
            'use_sim_time': False
        }]
    )

    # Comm hub - MCB communication (publishes /odom, /imu, subscribes /cmd_vel)
    comm_hub = Node(
        package='sentry_communication',
        executable='comm_hub',
        name='comm_hub',
        output='screen'
    )

    # LiDARs
    laser_driver_1 = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node_1",
        parameters=[os.path.join(pkg_bringup, "config", "rplidar_a1_1.yaml")],
        output="screen"
    )

    laser_driver_2 = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node_2",
        parameters=[os.path.join(pkg_bringup, "config", "rplidar_a1_2.yaml")],
        output="screen"
    )

    # Laser merger node 
    laser_merger = Node(
        package='sentry_bringup',
        executable='laser_merger.py',
        name='laser_merger',
        output='screen',
        parameters=[
            os.path.join(pkg_bringup, 'config', 'laser_merger.yaml'),
            {'use_sim_time': False}
        ]
    )

    # Controller (for sending commands to the robot)
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sentry_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_sentry_controller": "False",
            "use_python": "False"
        }.items(),
    )

    # Keyboard teleop
    keyboard_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',  # Opens in a new terminal window
        remappings=[('cmd_vel', 'cmd_vel')],
    )

    # Global localization (SLAM or AMCL based on slam argument)
    global_localization = IncludeLaunchDescription(
        os.path.join(pkg_localization, "launch", "global_localization.launch.py"),
        launch_arguments={
            'use_sim_time': 'false',
            'slam': slam,
            'map': map_yaml
        }.items()
    )

    # Local localization (EKF)
    local_localization = IncludeLaunchDescription(
        os.path.join(pkg_localization, "launch", "local_localization.launch.py"),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # Navigation (if use_nav is true)
    navigation = IncludeLaunchDescription(
        os.path.join(pkg_navigation, "launch", "navigation.launch.py"),
        condition=IfCondition(use_nav)
    )

    return LaunchDescription([
        # Arguments
        slam_arg,
        map_arg,
        robot_model_arg,
        use_nav_arg,
        # Robot description
        robot_state_publisher,
        # Drivers
        comm_hub,
        laser_driver_1,
        laser_driver_2,
        laser_merger,
        # Control
        controller,
        keyboard_teleop,
        # Localization
        global_localization,
        local_localization,
        # Navigation
        navigation,
    ])
