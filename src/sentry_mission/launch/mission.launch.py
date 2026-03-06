import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    mission_node = Node(
        package="sentry_mission",
        executable="mission_executor",
        name="mission_executor",
        output="screen"
    )
    
    return LaunchDescription([
        mission_node
    ])