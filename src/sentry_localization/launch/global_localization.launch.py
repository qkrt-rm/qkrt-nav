import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_localization = get_package_share_directory('sentry_localization')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    map_yaml = PathJoinSubstitution([pkg_localization, 'maps', LaunchConfiguration('map')])
    amcl_config = LaunchConfiguration('amcl_config')
    slam_config = LaunchConfiguration('slam_config')
    seed_initial_pose = LaunchConfiguration('seed_initial_pose')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false'
    )

    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='false'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='ARCC20263v3FieldCleaned.yaml'
    )

    amcl_config_arg = DeclareLaunchArgument(
        'amcl_config',
        default_value=os.path.join(pkg_localization, 'config', 'amcl.yaml')
    )

    slam_config_arg = DeclareLaunchArgument(
        'slam_config',
        default_value=os.path.join(pkg_localization, 'config', 'slam_toolbox.yaml')
    )

    # Seeding: true in sim (known spawn) seeds AMCL and skips the global scatter.
    # MUST stay false on the real robot, which starts at an unknown pose and needs
    # the scatter to auto-localize.
    seed_initial_pose_arg = DeclareLaunchArgument(
        'seed_initial_pose',
        default_value='false',
        description='Seed AMCL with a known start pose and skip global scatter. '
                    'Use only when the start pose is known (sim).'
    )
    initial_pose_x_arg = DeclareLaunchArgument('initial_pose_x', default_value='0.0')
    initial_pose_y_arg = DeclareLaunchArgument('initial_pose_y', default_value='0.0')
    initial_pose_yaw_arg = DeclareLaunchArgument('initial_pose_yaw', default_value='0.0')

    # SLAM Toolbox in mapping mode.
    # cwd='/tmp' prevents slam_toolbox from finding and reloading stale .posegraph
    # files left over from previous runs with bad odometry.
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', 'slam_toolbox:=WARN'],
        condition=IfCondition(slam),
        cwd='/tmp',
    )

    # Map server - used when slam=false to serve a pre-built map
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_yaml},
            {'use_sim_time': use_sim_time}
        ],
        condition=UnlessCondition(slam)
    )

    # AMCL localizes the robot within the static map using particle filtering
    # against live lidar scans. It publishes the map->odom TF.
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_config,
            {'use_sim_time': use_sim_time},
            # When seed_initial_pose:=false (real robot) this is False, identical to the
            # yaml, and the scatter below handles localization. When true (sim) AMCL
            # starts already converged at (initial_pose_x, y, yaw).
            {'set_initial_pose': ParameterValue(
                PythonExpression(["'", seed_initial_pose, "' == 'true'"]), value_type=bool)},
            {'initial_pose.x': ParameterValue(initial_pose_x, value_type=float)},
            {'initial_pose.y': ParameterValue(initial_pose_y, value_type=float)},
            {'initial_pose.z': 0.0},
            {'initial_pose.yaw': ParameterValue(initial_pose_yaw, value_type=float)},
        ],
        condition=UnlessCondition(slam)
    )

    # ~5 s after boot (once map_server + amcl are active), scatter particles across
    # the whole map so AMCL auto-localizes by scan-matching — no hardcoded start pose.
    # `ros2 service call` waits for the service to appear, so if AMCL isn't up at
    # exactly 5 s the call blocks until it is, then fires. Let the robot rotate after
    # this so the cloud collapses onto the true pose.
    global_localization_init = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call',
                     '/reinitialize_global_localization',
                     'std_srvs/srv/Empty'],
                output='screen',
            )
        ],
        # Only scatter when NOT seeding (and not in SLAM mode). On the real robot
        # seed_initial_pose is false, so this still fires exactly as before.
        condition=IfCondition(PythonExpression(
            ["'", slam, "' == 'false' and '", seed_initial_pose, "' == 'false'"])),
    )

    # Lifecycle manager brings up map_server and amcl in the correct order.
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'node_names': ['map_server', 'amcl']},
            {'use_sim_time': use_sim_time},
            {'autostart': True}
        ],
        condition=UnlessCondition(slam)
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_arg,
        map_arg,
        amcl_config_arg,
        slam_config_arg,
        seed_initial_pose_arg,
        initial_pose_x_arg,
        initial_pose_y_arg,
        initial_pose_yaw_arg,
        slam_toolbox_node,
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
        global_localization_init,
    ])
