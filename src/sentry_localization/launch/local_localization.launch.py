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

    sim_comm_hub_bridge = Node(
        package='sentry_localization',
        executable='sim_comm_hub_bridge.py',
        name='sim_comm_hub_bridge',
        output='screen',
        parameters=[
            {'joint_states_topic': '/joint_states'},
            {'imu_in_topic': '/imu'},
            {'imu_out_topic': '/imu_comm_hub'},
            {'odom_topic': '/wheel_odom'},
            {'publish_odom_tf': False},
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(use_sim_time)
    )

    imu_compensator_real = Node(
        package='sentry_localization',
        executable='imu_turret_compensator.py',
        name='imu_turret_compensator',
        output='screen',
        parameters=[
            {'imu_topic': '/imu'},
            {'joint_states_topic': '/joint_states'},
            {'output_topic': '/imu_base_compensated'},
            {'base_frame': 'base_link'},
            {'yaw_joint_name': 'turret_shaft_joint'},
            {'pitch_joint_name': 'gimbal_joint'},
            {'yaw_axis_base': [0.0, 0.0, 1.0]},
            {'pitch_axis_yaw_frame': [0.0, 1.0, 0.0]},
            {'use_sim_time': use_sim_time},
        ],
        condition=UnlessCondition(use_sim_time)
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        sim_comm_hub_bridge,
        imu_compensator_real,
        ekf_node,
        ekf_node_real,
    ])
