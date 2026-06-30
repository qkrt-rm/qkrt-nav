import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_bringup = get_package_share_directory('sentry_bringup')
    pkg_description = get_package_share_directory('sentry_description')
    pkg_localization = get_package_share_directory('sentry_localization')
    pkg_navigation = get_package_share_directory('sentry_navigation')

    # Launch arguments
    slam = LaunchConfiguration("slam")
    slam_arg = DeclareLaunchArgument(
        "slam",
        default_value="true",
        description="Use SLAM for mapping. If false, use AMCL with a pre-built map."
    )

    map_yaml = LaunchConfiguration("map")
    map_arg = DeclareLaunchArgument(
        "map",
        default_value='sentry_map.yaml',
        description="Map filename."
    )

    robot_model = LaunchConfiguration("robot_model")
    robot_model_path = PathJoinSubstitution([pkg_description, 'urdf', robot_model])
    robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value='sentry_description.urdf.xacro',
        description="Robot model filename in sentry_description/urdf."
    )

    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher")
    use_joint_state_publisher_arg = DeclareLaunchArgument(
        "use_joint_state_publisher",
        default_value="true",
        description="Publish default joint states for non-fixed joints when hardware does not.",
    )

    use_localization = LaunchConfiguration("use_localization")
    use_localization_arg = DeclareLaunchArgument(
        "use_localization",
        default_value="true",
        description="Launch global localization (SLAM or AMCL). Set false to skip map entirely."
    )

    use_keepout = LaunchConfiguration("use_keepout")
    use_keepout_arg = DeclareLaunchArgument(
        "use_keepout",
        default_value="true",
        description="Load arena keepout mask. Set false when not in the arena."
    )

    use_nav = LaunchConfiguration("use_nav")
    use_nav_arg = DeclareLaunchArgument(
        "use_nav",
        default_value="true",
        description="Launch navigation stack"
    )

    use_battery_mission = LaunchConfiguration("use_battery_mission")
    use_battery_mission_arg = DeclareLaunchArgument(
        "use_battery_mission",
        default_value="false",
        description="Launch battery mission controller and dummy battery publisher."
    )

    center_x = LaunchConfiguration("center_x")
    center_x_arg = DeclareLaunchArgument(
        "center_x",
        default_value="6.0",
        description="X coordinate of arena center in map frame."
    )

    center_y = LaunchConfiguration("center_y")
    center_y_arg = DeclareLaunchArgument(
        "center_y",
        default_value="4.0",
        description="Y coordinate of arena center in map frame."
    )

    display = LaunchConfiguration("display")
    display_arg = DeclareLaunchArgument(
        "display",
        default_value="false",
        description="Launch RViz for visualization"
    )

    # AMCL seeding: seed a known start pose at boot and skip the global scatter.
    # These default to the map origin (0,0,0) which is almost certainly NOT where the
    # robot physically starts — set initial_pose_x/y/yaw to the robot's real start spot
    # in the map frame (override on the command line from your startup script), e.g.:
    #   ros2 launch sentry_bringup real_robot.launch.py slam:=false \
    #        initial_pose_x:=1.5 initial_pose_y:=0.8 initial_pose_yaw:=3.14
    seed_initial_pose = LaunchConfiguration("seed_initial_pose")
    seed_initial_pose_arg = DeclareLaunchArgument(
        "seed_initial_pose",
        default_value="false",
        description="false = AMCL auto-localizes from an unknown start (global scatter). "
                    "Set true (with initial_pose_*) only when the start pose is known."
    )

    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_x_arg = DeclareLaunchArgument("initial_pose_x", default_value="0.0")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_y_arg = DeclareLaunchArgument("initial_pose_y", default_value="0.0")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    initial_pose_yaw_arg = DeclareLaunchArgument("initial_pose_yaw", default_value="0.0")

    # Robot state publisher (publishes URDF transforms: base_link → laser frames, etc.)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command([
                    'xacro ',
                    robot_model_path
                ]),
                value_type=str
            ),
            'use_sim_time': False
        }]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_joint_state_publisher),
    )

    # Comm hub - MCB communication (publishes /odom, /imu, subscribes /cmd_vel_rotated)
    comm_hub = Node(
        package='sentry_communication',
        executable='comm_hub',
        name='comm_hub',
        output='screen',
        remappings=[('cmd_vel', 'cmd_vel_rotated')]
    )

    vision_bridge = Node(
        package='sentry_communication',
        executable='vision_bridge_node',
        name='vision_bridge_node',
        output='screen'
    )

    # Rotate nav2's /cmd_vel (base_link frame) into gimbal_link frame before
    # sending to the MCB. Costmaps use base_link so the footprint stays fixed.
    cmd_vel_rotator = Node(
        package='sentry_bringup',
        executable='cmd_vel_gimbal_rotator.py',
        name='cmd_vel_gimbal_rotator',
        output='screen',
        parameters=[{
            'input_topic': '/cmd_vel',
            'output_topic': '/cmd_vel_rotated',
            'gimbal_joint_name': 'gimbal_joint',
        }]
    )

    # LiDARs
    # laser_driver_1 = Node(
    #     package="rplidar_ros",
    #     executable="rplidar_node",
    #     name="rplidar_node_1",
    #     parameters=[os.path.join(pkg_bringup, "config", "rplidar_a1_1.yaml")],
    #     remappings=[("/scan", "/scan_left")], 
    #     output="screen"
    # )

    # laser_driver_2 = Node(
    #     package="rplidar_ros",
    #     executable="rplidar_node",
    #     name="rplidar_node_2",
    #     parameters=[os.path.join(pkg_bringup, "config", "rplidar_a1_2.yaml")],
    #     remappings=[("/scan", "/scan_right")],
    #     output="screen"
    # )

    laser_driver_1 = Node(
        package='ldlidar_ros2',
        executable='ldlidar_ros2_node',
        name='ldlidar_node_1',
        parameters=[{
            'product_name': 'LDLiDAR_LD19',
            'topic_name': 'scan_right',
            'port_name': '/dev/ttyUSB1',
            'port_baudrate': 230400,    
            'laser_scan_dir': True,
            'enable_angle_crop_func': False,
            'frame_id': 'laser_frame_right'
        }],
        output='screen'
    )

    laser_driver_2 = Node(
        package='ldlidar_ros2',
        executable='ldlidar_ros2_node',
        name='ldlidar_node_2',
        parameters=[{
            'product_name': 'LDLiDAR_LD19',
            'topic_name': 'scan_left',
            'port_name': '/dev/ttyUSB0',
            'port_baudrate': 230400,
            'laser_scan_dir': True,
            'enable_angle_crop_func': False,
            'frame_id': 'laser_frame_left'
        }],
        output='screen'
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
            'map': map_yaml,
            # Seed AMCL at the robot's known start pose (set initial_pose_* below / on CLI).
            'seed_initial_pose': seed_initial_pose,
            'initial_pose_x': initial_pose_x,
            'initial_pose_y': initial_pose_y,
            'initial_pose_yaw': initial_pose_yaw,
        }.items(),
        condition=IfCondition(use_localization)
    )

    # Local localization (EKF)
    local_localization = IncludeLaunchDescription(
        os.path.join(pkg_localization, "launch", "local_localization.launch.py"),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # Navigation (if use_nav is true)
    navigation = IncludeLaunchDescription(
        os.path.join(pkg_navigation, "launch", "navigation.launch.py"),
        launch_arguments={
            'use_keepout': use_keepout,
            'use_battery_mission': use_battery_mission,
            'center_x': center_x,
            'center_y': center_y,
        }.items(),
        condition=IfCondition(use_nav)
    )

    # RViz (if display is true)
    rviz_config_path = os.path.join(pkg_description, 'rviz', 'sentry_config.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(display)
    )

    return LaunchDescription([
        # Arguments
        slam_arg,
        map_arg,
        robot_model_arg,
        use_joint_state_publisher_arg,
        use_localization_arg,
        use_keepout_arg,
        use_nav_arg,
        use_battery_mission_arg,
        center_x_arg,
        center_y_arg,
        display_arg,
        seed_initial_pose_arg,
        initial_pose_x_arg,
        initial_pose_y_arg,
        initial_pose_yaw_arg,
        # Robot description
        robot_state_publisher,
        #joint_state_publisher,
        # Drivers
        comm_hub,
        vision_bridge,
        cmd_vel_rotator,
        # laser_driver_1,
        laser_driver_2,
        laser_merger,
        # Control
        keyboard_teleop,
        # Localization
        global_localization,
        local_localization,
        # Navigation
        navigation,
        # Visualization
        rviz,
    ])
