import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_description = get_package_share_directory('sentry_description')
    pkg_localization = get_package_share_directory('sentry_localization')
    pkg_navigation = get_package_share_directory('sentry_navigation')
    
    # Display with RViz (robot description + visualization)
    display = IncludeLaunchDescription(
        os.path.join(pkg_description, "launch", "display.launch.py"),
        launch_arguments={'use_sim_time': 'false'}.items()
    )
    
    # Hardware drivers tbd
    # drivers = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("sentry_drivers"),
    #         "launch",
    #         "drivers.launch.py"
    #     ),
    # )
    

    # Real LiDAR driver (LDLiDAR LD06 via ldlidar_stl_ros2)
    lidar_driver = Node(
    package='ldlidar_stl_ros2',
    executable='ldlidar_stl_ros2_node',
    name='ldlidar',
    output='screen',
    parameters=[
        {'product_name': 'LDLiDAR_LD06'},
        {'topic_name': 'scan'},
        {'frame_id': 'base_laser'},
        {'port_name': '/dev/ttyUSB0'},
        {'port_baudrate': 230400},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},
    ]
)





    
    # Local localization (EKF)
    local_localization = IncludeLaunchDescription(
        os.path.join(pkg_localization, "launch", "local_localization.launch.py"),
        launch_arguments={'use_sim_time': 'false'}.items()
    )
    
    # Global localization (AMCL + Map)
    global_localization = IncludeLaunchDescription(
        os.path.join(pkg_localization, "launch", "global_localization.launch.py"),
        launch_arguments={'use_sim_time': 'false'}.items()
    )
    
    # Navigation stack
    navigation = IncludeLaunchDescription(
        os.path.join(pkg_navigation, "launch", "navigation.launch.py"),
        launch_arguments={'use_sim_time': 'false'}.items()
    )
    
    return LaunchDescription([
        display,
        # drivers,
        lidar_driver,
        local_localization,
        global_localization,
        navigation,
    ])