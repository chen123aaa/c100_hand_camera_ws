from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('c100_hand_camera'),
        'config',
        'c100_hand_camera.yaml',
    ])

    device_arg = DeclareLaunchArgument(
        'device',
        default_value='',
        description='Required C100 hand camera video device path, for example /dev/video2.',
    )

    return LaunchDescription([
        device_arg,
        Node(
            package='c100_hand_camera',
            executable='c100_hand_camera_node',
            name='c100_hand_camera_node',
            output='screen',
            parameters=[
                config_file,
                {'device': LaunchConfiguration('device')},
            ],
        ),
    ])
