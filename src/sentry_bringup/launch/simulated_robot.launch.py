import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sentry_description"),
            "launch",
            "gazebo.launch.py"
        ),
    )
    
    global_localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sentry_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    local_localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sentry_localization"),
            "launch",
            "local_localization.launch.py"
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("sentry_navigation"),
            "launch",
            "navigation.launch.py"
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    return LaunchDescription([
        gazebo,
        global_localization,
        local_localization,
        navigation,
    ])