import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_localization = get_package_share_directory('sentry_localization')

    use_sim_time = LaunchConfiguration('use_sim_time')
    amcl_config = LaunchConfiguration('amcl_config')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false'
    )

    amcl_config_arg = DeclareLaunchArgument(
        'amcl_config',
        default_value=os.path.join(pkg_localization, 'config', 'amcl.yaml')
    )

    map_yaml_path = os.path.join(pkg_localization, 'config', 'sentry_map.yaml')

    # map_server loads the static map saved from a prior SLAM mapping session.
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_yaml_path},
            {'use_sim_time': use_sim_time}
        ]
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
            {'use_sim_time': use_sim_time}
        ]
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
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        amcl_config_arg,
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
    ])
