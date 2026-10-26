import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_localization = get_package_share_directory('sentry_localization')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false'
    )
    
    ekf_config = os.path.join(pkg_localization, 'config', 'ekf.yaml')
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'publish_tf': False},
            {'odom0': '/odom'},
            {'imu0': '/imu_comm_hub'},
            {'imu0_config': [False, False, False,
                             False, False, False,
                             False, False, False,
                             False, False, True,
                             False, False, False]},
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(use_sim_time)
    )

    ekf_node_real = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': use_sim_time}
        ],
        condition=UnlessCondition(use_sim_time)
    )


    return LaunchDescription([
        use_sim_time_arg,
        ekf_node,
        ekf_node_real,
    ])
