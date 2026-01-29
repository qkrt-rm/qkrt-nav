import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_bringup = get_package_share_directory('sentry_bringup')
    pkg_description = get_package_share_directory('sentry_description')
    pkg_localization = get_package_share_directory('sentry_localization')
    pkg_navigation = get_package_share_directory('sentry_navigation')
    # Launch arguments
    use_slam = LaunchConfiguration("use_slam")
    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false",
        description="Use SLAM instead of localization"
    )

    use_keyboard = LaunchConfiguration("use_keyboard")
    use_keyboard_arg = DeclareLaunchArgument(
        "use_keyboard",
        default_value="false",
        description="Use keyboard teleop instead of joystick"
    )

    use_nav = LaunchConfiguration("use_nav")
    use_nav_arg = DeclareLaunchArgument(
        "use_nav",
        default_value="true",
        description="Launch navigation stack"
    )

    # Comm hub - MCB communication (publishes /mcb_odom, subscribes /cmd_vel)
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

    # Laser merger
    laser_merger = IncludeLaunchDescription(
        os.path.join(pkg_bringup, "launch", "laser_merger.launch.py"),
        launch_arguments={'use_sim_time': 'false'}.items()
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

    # Joystick teleop (when not using keyboard)
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sentry_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={"use_sim_time": "False"}.items(),
        condition=UnlessCondition(use_keyboard)
    )

    # Keyboard teleop (when using keyboard)
    keyboard_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',  # Opens in a new terminal window
        remappings=[('cmd_vel', 'cmd_vel')],
        condition=IfCondition(use_keyboard)
    )

    # SLAM (if use_slam is true)
    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sentry_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )

    # Global localization (if not using SLAM)
    global_localization = IncludeLaunchDescription(
        os.path.join(pkg_localization, "launch", "global_localization.launch.py"),
        condition=UnlessCondition(use_slam)
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
        use_slam_arg,
        use_keyboard_arg,
        use_nav_arg,
        # Drivers
        comm_hub,
        laser_driver_1,
        laser_driver_2,
        laser_merger,
        # Control
        controller,
        joystick,
        keyboard_teleop,
        # Localization
        slam,
        global_localization,
        local_localization,
        # Navigation
        navigation,
    ])
