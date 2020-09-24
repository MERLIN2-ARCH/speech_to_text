from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_speech_recognition',
            executable='stt_node',
            name='stt_node',
            namespace='speech_recognition',
        ),

        Node(
            package='ros2_speech_recognition',
            executable='nlp_node',
            name='nlp_node',
            namespace='speech_recognition',
        ),

        Node(
            package='ros2_speech_recognition',
            executable='parser_node',
            name='parser_node',
            namespace='speech_recognition',
        )

    ])
