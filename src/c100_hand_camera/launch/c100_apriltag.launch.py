#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('image_topic', default_value='/c100_hand_camera/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/c100_hand_camera/camera_info'),
        DeclareLaunchArgument('target_id', default_value='0'),
        DeclareLaunchArgument('tag_size', default_value='0.03'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
    ]
    detector = Node(
        package='c100_hand_camera',
        executable='c100_detect_apriltag',
        name='c100_apriltag_detector',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'target_id': LaunchConfiguration('target_id'),
            'tag_size': LaunchConfiguration('tag_size'),
            'publish_tf': LaunchConfiguration('publish_tf'),
        }],
    )
    return LaunchDescription(arguments + [detector])