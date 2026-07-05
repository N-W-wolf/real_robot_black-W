from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    real_runner_node = Node(
        package='real_runner',
        executable='real_runner',
        name='real_runner',
        output='screen',
    )

    return LaunchDescription([real_runner_node])
