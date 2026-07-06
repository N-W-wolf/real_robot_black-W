from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    real_runner_node = Node(
        package='real_runner',
        executable='real_runner',
        name='real_runner',
        output='screen',
        parameters=[{
            'imu_port': '/dev/IMU_Link',
            'imu_baudrate': 460800,
            'imu_vqf_enabled': True,
            'imu_vqf_tau_acc': 3.0,
            'imu_vqf_dt': 0.002,
        }],
    )

    return LaunchDescription([real_runner_node])
