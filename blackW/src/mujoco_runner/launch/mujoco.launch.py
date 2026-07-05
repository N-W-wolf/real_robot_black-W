import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    DeclareLaunchArgument('rname', default_value='black')
    rname = LaunchConfiguration("rname")

    robot_name = ParameterValue(Command(["echo -n ", rname]), value_type=str)
    gazebo_model_name = ParameterValue(Command(["echo -n ", rname, "_gazebo"]), value_type=str)

    param_node = Node(
        package="demo_nodes_cpp",
        executable="parameter_blackboard",
        name="param_node",
        output='screen',
        parameters=[{
            "robot_name": robot_name,
            "gazebo_model_name": gazebo_model_name,
        }],
    )

    mujoco_node=Node(
        package="mujoco_runner",
        executable="mm",
        name="mujoco_node",
        output='screen',
        parameters=[{
            "rname": robot_name
        }],
    )

    middleware_node = Node(
        package='midware',
        executable='middleware',
        name='middleware',
        output='screen'
    )

    return LaunchDescription([param_node,mujoco_node,middleware_node])