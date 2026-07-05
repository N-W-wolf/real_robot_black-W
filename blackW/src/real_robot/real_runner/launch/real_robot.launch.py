import os
from ament_index_python.packages import get_package_share_directory # 关键库 1
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription # 关键库 2
from launch.launch_description_sources import PythonLaunchDescriptionSource # 关键库 3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取 ag_ros_node 包的安装路径
    ag_pkg_dir = get_package_share_directory('ag_ros_node')
    
    ag_launch_path = os.path.join(ag_pkg_dir, 'launch', 'AgRosNode.launch.py')

    # 创建引用动作
    imu_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ag_launch_path),
    )

    rname_arg = DeclareLaunchArgument('rname', default_value='blackW')
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

    real_runner_node = Node(
        package='real_runner',
        executable='real_runner',
        name='real_runner',
        output='screen'
    )

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

    middleware_node = Node(
        package='midware',
        executable='middleware',
        name='middleware',
        output='screen'
    )

    return LaunchDescription([
        rname_arg,
        imu_launch_include,  # 启动 IMU
        imu_vqf_filter_node,  # IMU 姿态滤波
        real_runner_node,    # 启动主控
        middleware_node,      # 启动中间件
        param_node
    ])
