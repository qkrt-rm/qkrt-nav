import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    mapping_mode = LaunchConfiguration('mapping_mode')

    mapping_mode_arg = DeclareLaunchArgument(
        'mapping_mode',
        default_value='true',
        description='Set true to run SLAM mapping. Set false to run AMCL navigation with a saved map. To save it, run ros2 run nav2_map_server map_saver_cli -f ~/my_map'
    )

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory('sentry_description'),
            'launch',
            'gazebo.launch.py'
        ),
    )

    # Mapping mode: SLAM toolbox builds the map live.
    # Drive the robot around, then save with:
    #   ros2 run nav2_map_server map_saver_cli -f ~/my_map
    mapping_localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory('sentry_localization'),
            'launch',
            'mapping.launch.py'
        ),
        condition=IfCondition(mapping_mode),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Navigation mode: AMCL localizes against the saved static map.
    amcl_localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory('sentry_localization'),
            'launch',
            'global_localization.launch.py'
        ),
        condition=UnlessCondition(mapping_mode),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # EKF (local_localization) is intentionally excluded from simulation.
    # The planar_move Gazebo plugin publishes perfect odometry and the
    # odom->base_link TF directly. Running EKF alongside it would create
    # a TF conflict. EKF is only needed on the real robot.

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory('sentry_navigation'),
            'launch',
            'navigation.launch.py'
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    laser_merger = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory('sentry_bringup'),
            'launch',
            'laser_merger.launch.py'
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    return LaunchDescription([
        mapping_mode_arg,
        gazebo,
        laser_merger,
        mapping_localization,
        amcl_localization,
        navigation,
    ])
