import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    pkg_localization = get_package_share_directory('sentry_localization')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    slam_config = LaunchConfiguration('slam_config')
    amcl_config = LaunchConfiguration('amcl_config')
    map_yaml = LaunchConfiguration('map_yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false'
    )

    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='Use SLAM for mapping. If false, use AMCL with a pre-built map.'
    )

    slam_config_arg = DeclareLaunchArgument(
        'slam_config',
        default_value=os.path.join(pkg_localization, 'config', 'slam_toolbox.yaml')
    )

    amcl_config_arg = DeclareLaunchArgument(
        'amcl_config',
        default_value=os.path.join(pkg_localization, 'config', 'amcl.yaml')
    )

    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=os.path.join(pkg_localization, 'config', 'sentry_map.yaml'),
        description='Path to the map YAML file for AMCL localization'
    )

    # SLAM Toolbox - used when slam=true for mapping
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(slam)
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

    # AMCL - used when slam=false for localization on a pre-built map
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_config,
            {'use_sim_time': use_sim_time}
        ],
        condition=UnlessCondition(slam)
    )

    # Lifecycle manager for map_server and AMCL (only when using AMCL)
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
        slam_config_arg,
        amcl_config_arg,
        map_yaml_arg,
        slam_toolbox_node,
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
    ])