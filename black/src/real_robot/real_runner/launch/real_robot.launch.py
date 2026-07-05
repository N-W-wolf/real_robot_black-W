import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ag_launch_path = os.path.join(
        get_package_share_directory('ag_ros_node'),
        'launch',
        'AgRosNode.launch.py',
    )
    imu_launch_path = os.path.join(
        get_package_share_directory('midware'),
        'launch',
        'imu_vqf_filter.launch.py',
    )
    real_runner_launch_path = os.path.join(
        get_package_share_directory('real_runner'),
        'launch',
        'real_runner.launch.py',
    )

    ag_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ag_launch_path),
    )
    imu_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(imu_launch_path),
    )
    real_runner_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(real_runner_launch_path),
    )

    rname_arg = DeclareLaunchArgument('rname', default_value='black')
    rname = LaunchConfiguration('rname')

    robot_name = ParameterValue(Command(['echo -n ', rname]), value_type=str)
    gazebo_model_name = ParameterValue(Command(['echo -n ', rname, '_gazebo']), value_type=str)

    param_node = Node(
        package='demo_nodes_cpp',
        executable='parameter_blackboard',
        name='param_node',
        output='screen',
        parameters=[{
            'robot_name': robot_name,
            'gazebo_model_name': gazebo_model_name,
        }],
    )

    middleware_node = Node(
        package='midware',
        executable='middleware',
        name='middleware',
        output='screen',
    )

    return LaunchDescription([
        rname_arg,
        ag_launch_include,
        TimerAction(period=2.0, actions=[real_runner_launch_include]),
        TimerAction(period=4.0, actions=[imu_launch_include]),
        TimerAction(period=5.0, actions=[middleware_node, param_node]),
    ])
