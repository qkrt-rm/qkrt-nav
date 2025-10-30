from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get package share directories
    pkg_description = FindPackageShare(package='sentry_description').find('sentry_description')
    pkg_bringup = FindPackageShare(package='sentry_bringup').find('sentry_bringup')

    # File paths
    default_model_path = os.path.join(pkg_description, 'src', 'description', 'sentry_description.urdf')
    default_rviz_config_path = os.path.join(pkg_description, 'rviz', 'config.rviz')
    ekf_config_path = os.path.join(pkg_bringup, 'config', 'ekf.yaml')
    world_path = os.path.join(pkg_bringup, 'worlds', 'my_world.sdf')

    # Nodes

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])},
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}
                    
                    
                    ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'sentry', '-topic', 'robot_description'],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world_path],
        output='screen'
    )

    tf2_ros_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='map_to_base_link',
    arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            os.path.join(pkg_bringup, 'config', 'map_server.yaml'),
            {'yaml_filename': os.path.join(pkg_bringup, 'config', 'sentry_map.yaml')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            os.path.join(pkg_bringup, 'config', 'amcl.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ]
    )

    # Launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot model file'),
        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=default_rviz_config_path,
            description='Absolute path to RViz config file'),
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='True',
            description='Flag to enable use_sim_time'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        tf2_ros_node,
        robot_localization_node,
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
        rviz_node,
        gazebo
    ])
