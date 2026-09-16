from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='debug',
            executable='debug',
            arguments=['debug_1'],
            output='screen'
        )
    ])

