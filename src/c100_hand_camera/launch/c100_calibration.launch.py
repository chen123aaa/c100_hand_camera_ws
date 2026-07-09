from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('c100_hand_camera'),
        'config',
        'c100_calibration.yaml',
    ])

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/c100_hand_camera/image_raw',
        description='Image topic published by c100_hand_camera_node.',
    )

    output_path_arg = DeclareLaunchArgument(
        'output_path',
        default_value='calibration/c100_hand_camera_calibrated.yaml',
        description='Calibration YAML output path.',
    )
    debug_image_topic_arg = DeclareLaunchArgument(
        'debug_image_topic',
        default_value='/c100_calibration/debug_image',
        description='Annotated calibration debug image topic.',
    )

    target_samples_arg = DeclareLaunchArgument(
        'target_samples',
        default_value='25',
        description='Number of accepted chessboard samples before calibration.',
    )

    return LaunchDescription([
        image_topic_arg,
        debug_image_topic_arg,
        output_path_arg,
        target_samples_arg,
        Node(
            package='c100_hand_camera',
            executable='c100_calibrate_camera',
            name='c100_calibration_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'image_topic': LaunchConfiguration('image_topic'),
                    'debug_image_topic': LaunchConfiguration('debug_image_topic'),
                    'output_path': LaunchConfiguration('output_path'),
                    'target_samples': ParameterValue(
                        LaunchConfiguration('target_samples'),
                        value_type=int,
                    ),
                },
            ],
        ),
    ])
