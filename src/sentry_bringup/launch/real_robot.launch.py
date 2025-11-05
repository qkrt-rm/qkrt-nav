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
    
    # LiDAR driver tbh
    # laser_driver = Node(
    #     package="rplidar_ros",
    #     executable="rplidar_node",
    #     name="rplidar_node",
    #     parameters=[{
    #         'serial_port': '/dev/ttyUSB0',
    #         'frame_id': 'laser_frame',
    #         'angle_compensate': True,
    #     }],
    #     output="screen"
    # )
    
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
        # laser_driver,
        local_localization,
        global_localization,
        navigation,
    ])