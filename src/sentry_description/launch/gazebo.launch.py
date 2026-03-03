import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_description = get_package_share_directory('sentry_description')
    rviz_config_path = os.path.join(pkg_description, 'rviz', 'sentry_config.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='sentry_description_v2.urdf.xacro'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='comp_map.sdf'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )

    robot_model_path = PathJoinSubstitution([pkg_description, 'urdf', LaunchConfiguration('robot_model')])
    world_path = PathJoinSubstitution([pkg_description, 'worlds', LaunchConfiguration('world')])
    robot_description = Command(['xacro ', robot_model_path])
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'use_sim_time': use_sim_time
        }]
    )
    
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
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
        ros_arguments=['-p', ['use_sim_time:=', use_sim_time]],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        robot_model_arg,
        world_arg,
        use_sim_time_arg,
        robot_state_publisher_node,
        gazebo_server,
        gazebo_client,
        spawn_entity,
        rviz_node,
    ])
