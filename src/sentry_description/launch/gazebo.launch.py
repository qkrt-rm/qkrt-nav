import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_description = get_package_share_directory('sentry_description')
    
    urdf_path = os.path.join(pkg_description, 'urdf', 'sentry_description.urdf.xacro')
    world_path = os.path.join(pkg_description, 'worlds', 'my_world.sdf')
    #rviz_config_path = os.path.join(pkg_description, 'rviz', 'sentry_config.rviz')
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_path
    )
    
    # **Process Xacro to expand all macros and properties**
    robot_description = Command(['xacro ', urdf_path])
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
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
            '-z', '0.2'
        ],
        output='screen'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', urdf_path],
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