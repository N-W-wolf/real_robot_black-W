from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    imu_vqf_filter_node = Node(
        package='midware',
        executable='imu_vqf_filter',
        name='imu_vqf_filter',
        output='screen',
        parameters=[{
            'input_topic': '/_lowState/imu_raw',
            'output_topic': '/_lowState/imu',
            'tau_acc': 3.0,
            'initial_dt': 0.002,
            'pass_through': False,
            'log_period': 1.0,
        }]
    )

    return LaunchDescription([imu_vqf_filter_node])
