from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    tag_size = LaunchConfiguration('tag_size')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'),

        DeclareLaunchArgument(
            'image_topic',
            default_value='/sentry/depth_camera/image_raw',
            description='Camera image topic'),

        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/sentry/depth_camera/camera_info',
            description='Camera info topic for intrinsics'),

        DeclareLaunchArgument(
            'tag_size',
            default_value='0.2',
            description='AprilTag side length in meters'),

        Node(
            package='sentry_vision',
            executable='detector',
            name='apriltag_detector',
            parameters=[{
                'use_sim_time': use_sim_time,
                'image_topic': image_topic,
                'camera_info_topic': camera_info_topic,
                'tag_size': tag_size,
            }],
            output='screen',
        ),
    ])
