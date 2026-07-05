"""Launch a talker and a listener in a component container."""
 
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
 
 
def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            package="ag_ros_node",
            executable="ag_ros_node",
            namespace='',
            name="ag_ros_node",
            output="screen",
            #ag_ros_node 节点运行配置参数
            parameters=[
                #连接类型：serial port：0 , UDP：1
                {"ConnectionType": 0},
                
				#串口设备串  defaule: /dev/ttyACM0 /dev/IMU_Link
                {"UART_Port": "/dev/ttyACM0"},
				#串口波特率  default: 115200
                {"UART_Baudrate": 460800},
                #latency_timer :1 ~ 16, default:16 
                {"USB_LatencyTime": 16},

                #UDP addr  default 192.168.225.2
                {"UDP_Addr": "192.168.225.2"},
                #UDP port  default 12300
                {"UDP_Port": 12300},
		
				#5503协议：陀螺量程
                {"Grange04": 250.0},
				#AG041协议：加表量程
                {"Arange04": 4.0},
                
				#570D协议：陀螺量程
				{"Grange0B": 4.0},
				#570D协议：加表量程
				{"Arange0B": 4.0},

                #设置日志文件路径
                {"mLogPath":"./"},
                #default: close
                #{"LogInfo":"debug.log"},
                #设置日志打印等级：DEBUG:0 (save imu rawdata),INFO:1,WARNING:2,ERROR:3,FATAL:4
                #default:INFO
                {"LogLevel":1},

                #设置Data_IMU 发布频率分频系数(基础频率为100Hz(10ms间隔)，均匀抽样, 设置后频率为  1000/(10*N) Hz, 设置0时不输出)
                {"IMUFreqFactor":   1}, 

                #设置Data_GPS 发布频率分频系数(基础频率为100Hz(10ms间隔)，均匀抽样, 设置后频率为  1000/(10*N) Hz, 设置0时不输出)
                {"GPSFreqFactor":   5},

                #设置Data_odom 发布频率分频系数(基础频率为100Hz(10ms间隔)，均匀抽样, 设置后频率为  1000/(10*N) Hz,设置0时不输出)
                {"OdomFreqFactor":  5},

                #设置PboxSPD 发布频率分频系数(基础频率为100Hz(10ms间隔)，均匀抽样, 设置后频率为  1000/(10*N) Hz, 设置0时不输出)
                {"SpdFreqFactor":  5}
            ],
    )
    return launch.LaunchDescription([container])
