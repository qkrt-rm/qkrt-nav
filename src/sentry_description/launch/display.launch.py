#(RVIZ only)

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_description = get_package_share_directory('sentry_description')
    
    # urdf_path = os.path.join(pkg_description, 'urdf', 'sentry_description.urdf')
    urdf_path = os.path.join(pkg_description, 'urdf', 'sentry_description.urdf.xacro')

    rviz_config_path = os.path.join(pkg_description, 'rviz', 'sentry_config.rviz')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false'
    )
    robot_description = Command(['xacro', ' ', urdf_path])
    # with open(urdf_path, 'r') as file:
    #     robot_description = file.read()
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])