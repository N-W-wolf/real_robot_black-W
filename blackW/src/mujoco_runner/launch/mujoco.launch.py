from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from mujoco_runner import config


def generate_launch_description():
    rname_arg = DeclareLaunchArgument("rname", default_value=config.DEFAULT_ROBOT_NAME)
    scene_arg = DeclareLaunchArgument("scene", default_value=config.SCENE_DEFAULT)
    publish_odom_arg = DeclareLaunchArgument("publish_odom", default_value="false")
    publish_rate_hz_arg = DeclareLaunchArgument("publish_rate_hz", default_value="500.0")
    render_arg = DeclareLaunchArgument("render", default_value="true")
    render_rate_hz_arg = DeclareLaunchArgument("render_rate_hz", default_value="60.0")
    real_time_arg = DeclareLaunchArgument("real_time", default_value="true")
    publish_gap_model_arg = DeclareLaunchArgument("publish_gap_model", default_value="true")
    gap_model_topic_arg = DeclareLaunchArgument("gap_model_topic", default_value="/gap_model")
    gap_model_confidence_arg = DeclareLaunchArgument("gap_model_confidence", default_value="1.0")
    gap_missing_marker_arg = DeclareLaunchArgument("gap_missing_marker", default_value="-9999.0")
    enable_additional_noise_arg = DeclareLaunchArgument(
        "enable_additional_noise",
        default_value=str(config.ENABLE_ADDITIONAL_NOISE).lower(),
    )

    rname = LaunchConfiguration("rname")
    scene = LaunchConfiguration("scene")
    publish_odom = LaunchConfiguration("publish_odom")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    render = LaunchConfiguration("render")
    render_rate_hz = LaunchConfiguration("render_rate_hz")
    real_time = LaunchConfiguration("real_time")
    publish_gap_model = LaunchConfiguration("publish_gap_model")
    gap_model_topic = LaunchConfiguration("gap_model_topic")
    gap_model_confidence = LaunchConfiguration("gap_model_confidence")
    gap_missing_marker = LaunchConfiguration("gap_missing_marker")
    enable_additional_noise = LaunchConfiguration("enable_additional_noise")

    param_node = Node(
        package="demo_nodes_cpp",
        executable="parameter_blackboard",
        name="param_node",
        output="screen",
        parameters=[
            {
                "robot_name": ParameterValue(rname, value_type=str),
                "gazebo_model_name": ParameterValue([rname, "_gazebo"], value_type=str),
            }
        ],
    )

    mujoco_node = Node(
        package="mujoco_runner",
        executable="mm",
        name="mujoco_node",
        output="screen",
        parameters=[
            {
                "rname": ParameterValue(rname, value_type=str),
                "scene": ParameterValue(scene, value_type=str),
                "publish_odom": ParameterValue(publish_odom, value_type=bool),
                "publish_rate_hz": ParameterValue(publish_rate_hz, value_type=float),
                "render": ParameterValue(render, value_type=bool),
                "render_rate_hz": ParameterValue(render_rate_hz, value_type=float),
                "real_time": ParameterValue(real_time, value_type=bool),
                "publish_gap_model": ParameterValue(publish_gap_model, value_type=bool),
                "gap_model_topic": ParameterValue(gap_model_topic, value_type=str),
                "gap_model_confidence": ParameterValue(gap_model_confidence, value_type=float),
                "gap_missing_marker": ParameterValue(gap_missing_marker, value_type=float),
                "enable_additional_noise": ParameterValue(enable_additional_noise, value_type=bool),
            }
        ],
    )

    return LaunchDescription(
        [
            rname_arg,
            scene_arg,
            publish_odom_arg,
            publish_rate_hz_arg,
            render_arg,
            render_rate_hz_arg,
            real_time_arg,
            publish_gap_model_arg,
            gap_model_topic_arg,
            gap_model_confidence_arg,
            gap_missing_marker_arg,
            enable_additional_noise_arg,
            param_node,
            mujoco_node,
        ]
    )
