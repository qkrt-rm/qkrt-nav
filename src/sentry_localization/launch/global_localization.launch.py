import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_localization = get_package_share_directory('sentry_localization')
    

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_config = LaunchConfiguration('slam_config')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false'
    )

    slam_config_arg = DeclareLaunchArgument(
        'slam_config',
        default_value=os.path.join(pkg_localization, 'config', 'slam_toolbox.yaml')
    )

    # SLAM Toolbox handles both mapping and localization.
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': use_sim_time}
        ]
    )

    # --- Disabled: using SLAM-only approach instead of AMCL + map_server ---
    # AMCL and map_server are not needed when SLAM handles both mapping and
    # localization. SLAM publishes /map and map->odom, so the navigation
    # stack's global costmap static layer picks up the map directly.
    #
    # amcl_config = LaunchConfiguration('amcl_config')
    # lifecycle_nodes = ['map_server', 'amcl']
    #
    # amcl_config_arg = DeclareLaunchArgument(
    #     'amcl_config',
    #     default_value=os.path.join(pkg_localization, 'config', 'amcl.yaml')
    # )
    #
    # map_yaml_path = os.path.join(pkg_localization, 'config', 'sentry_map.yaml')
    #
    # map_server_node = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     parameters=[
    #         {'yaml_filename': map_yaml_path},
    #         {'use_sim_time': use_sim_time}
    #     ]
    # )
    #
    # amcl_node = Node(
    #     package='nav2_amcl',
    #     executable='amcl',
    #     name='amcl',
    #     output='screen',
    #     parameters=[
    #         amcl_config,
    #         {'use_sim_time': use_sim_time}
    #     ]
    # )
    #
    # lifecycle_manager_node = Node(
    #     package='nav2_lifecycle_manager',
    #     executable='lifecycle_manager',
    #     name='lifecycle_manager_localization',
    #     output='screen',
    #     parameters=[
    #         {'node_names': lifecycle_nodes},
    #         {'use_sim_time': use_sim_time},
    #         {'autostart': True}
    #     ]
    # )
    # -----------------------------------------------------------------


    return LaunchDescription([
        use_sim_time_arg,
        slam_config_arg,
        #amcl_config_arg,
        slam_toolbox_node,
        #map_server_node,
        #amcl_node,
        #lifecycle_manager_node
    ])