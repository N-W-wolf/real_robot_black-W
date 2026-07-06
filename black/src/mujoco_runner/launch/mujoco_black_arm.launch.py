from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    declare_rname = DeclareLaunchArgument("rname", default_value="black_with_arm")
    rname = LaunchConfiguration("rname")
    robot_name = TextSubstitution(text="black")
    gazebo_model_name = TextSubstitution(text="black_gazebo")

    param_node = Node(
        package="demo_nodes_cpp",
        executable="parameter_blackboard",
        name="param_node",
        output="screen",
        parameters=[{"robot_name": robot_name, "gazebo_model_name": gazebo_model_name}],
    )

    mujoco_node = Node(
        package="mujoco_runner",
        executable="mm_black_arm",
        name="mujoco_node",
        output="screen",
        parameters=[{"rname": rname}],
    )

    return LaunchDescription([declare_rname, param_node, mujoco_node])
