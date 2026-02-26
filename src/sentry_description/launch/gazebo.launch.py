import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_description = get_package_share_directory('sentry_description')
    #old
    # urdf_path = os.path.join(pkg_description, 'urdf', 'sentry_description.urdf')
    urdf_path = os.path.join(pkg_description, 'urdf', 'sentry_description_v2.urdf.xacro')
    world_path = os.path.join(pkg_description, 'worlds', 'comp_map.sdf')
    rviz_config_path = os.path.join(pkg_description, 'rviz', 'sentry_config.rviz')
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_path
    )
    robot_description = Command(['xacro', ' ', urdf_path])
    # old
    # with open(urdf_path, 'r') as file:
    #     robot_description = file.read()
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'use_sim_time': True
        }]
    )
    
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', LaunchConfiguration('world')],
        output='screen'
    )
    
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )
    
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'sentry_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        world_arg,
        robot_state_publisher_node,
        gazebo_server,
        gazebo_client,
        spawn_entity,
        rviz_node,
    ])