import rclpy
from rclpy.node import Node
import threading
import time
import numpy as np

import mujoco
import mujoco.viewer

from sensor_msgs.msg import Imu
from robot_msgs.msg import RobotState, MotorState,RobotCommand

from ament_index_python.packages import get_package_share_directory
import os

from . import config

from collections import deque
#from imu import IMUFusion


class MujocoNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        self.declare_parameter('rname','black').__init__
        rname = self.get_parameter('rname').value
        self.get_logger().info(f"loading robot is :{rname}")

        # 电机映射表
        # Index(i): 程序/消息中的逻辑电机ID
        # Value(phy_id): MuJoCo 仿真中的物理电机ID
        # 默认顺序：[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
        self.motor_map = config.MOTOR_MAPPING
        self.actuator_num = config.ACTUATOR_NUM
        # self.motor_map = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]

        if(config.USE_TERRAIN):
            path = os.path.join(get_package_share_directory('robot_description'), rname,config.SENCE_TERRAIN)
        else:
            path = os.path.join(get_package_share_directory('robot_description'), rname,config.SENCE_PLANE)

        self.model = mujoco.MjModel.from_xml_path(path)
        self.data = mujoco.MjData(self.model)
        self.data.ctrl[:] = config.DEFAULT_POS
        self.values = np.zeros(40)
        self.lock=threading.RLock()
        self.model.opt.timestep=config.SIM_DT

#motor letency
        # self.delay_steps = max(1, int(config.MOTOR_DELAY / self.model.opt.timestep))
        # self.ctrl_buffer = [deque([0.0]*self.delay_steps, maxlen=self.delay_steps) for _ in range(self.actuator_num)]
        # self.joint_state_buffer = [deque([ (0.0, 0.0) ] * self.delay_steps, maxlen=self.delay_steps) for _ in range(self.actuator_num)]
        min_delay = 0.0
        max_delay = config.MOTOR_DELAY
        self.delay_steps_list = []
        for i in range(self.actuator_num):
            delay_time = np.random.uniform(min_delay, max_delay)
            delay_steps = max(1, int(delay_time / self.model.opt.timestep))
            self.delay_steps_list.append(delay_steps)
        self.ctrl_buffer = [
            deque([0.0] * ds, maxlen=ds)
            for ds in self.delay_steps_list
        ]
        self.joint_state_buffer = [
            deque([(0.0, 0.0)] * ds, maxlen=ds)
            for ds in self.delay_steps_list
        ]       

###motor bais
        self.encoder_position_bias = np.random.uniform(-config.MOTOR_BIAS, config.MOTOR_BIAS, self.actuator_num)
        self.encoder_velocity_bias = np.random.uniform(-config.MOTOR_BIAS, config.MOTOR_BIAS, self.actuator_num)  

        if(rname=='a1'):
            self.model.opt.timestep=0.0005

        self.viewer=mujoco.viewer.launch_passive(self.model, self.data)

        self.imu_publisher = self.create_publisher(Imu, '/_lowState/imu_raw', 10)
        self.joint_publisher = self.create_publisher(RobotState, '/_lowState/joint', 10)
        
        self.joint_subscriber = self.create_subscription(RobotCommand,'/_lowCmd/command', self.joint_cmd_callback, 10)
        self.timer = self.create_timer(0.0001, self.publish_data)
        
        self.simulation_thread = threading.Thread(target=self.run_simulation)
        self.simulation_thread.daemon = True
        self.simulation_thread.start()

        self.render_thread = threading.Thread(target=self.run_render)
        self.render_thread.daemon = True
        self.render_thread.start()

    def get_base_position(self):
    # 注意：确保数组不越界，你代码里初始化是 np.zeros(40)，长度够用
        base_x = self.values[34]
        base_y = self.values[35]
        base_z = self.values[36]
        return base_x, base_y, base_z

    def run_render(self):
        while self.viewer.is_running():
            with self.lock:
                self.viewer.sync()
                #self.reset_delay()
            time.sleep(config.RENDER_DT)#<50hz

    def run_simulation(self):
        while self.viewer.is_running():
            step_start = time.perf_counter()
            with self.lock:
                for i in range(self.actuator_num):
                    self.data.ctrl[i] = self.ctrl_buffer[i][0]
                mujoco.mj_step(self.model, self.data)
                #self.values = self.data.sensordata
            ##motor letency
                delayed_values = np.array(self.data.sensordata, copy=True)
                for i in range(self.actuator_num):
                    q_real  = float(self.data.sensordata[i])
                    dq_real = float(self.data.sensordata[i + self.actuator_num])
                    self.joint_state_buffer[i].append((q_real, dq_real))
                for i in range(self.actuator_num):
                    delayed_values[i]      = self.joint_state_buffer[i][0][0] 
                    delayed_values[i + self.actuator_num] = self.joint_state_buffer[i][0][1] 
                self.values = delayed_values
                self.reset_delay()
            ###

            time_until_next_step = self.model.opt.timestep - (time.perf_counter() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    def joint_cmd_callback(self, msg):
        if self.data is not None:
            with self.lock:
                for i in range(self.actuator_num):
                    msg.motor_command[i].q+=self.encoder_position_bias[i]
                    # 获取映射后的物理ID
                    phy_id = self.motor_map[i]

                    # 反馈数据使用 phy_id 获取对应的物理电机状态
                    # self.values 存储的是物理顺序的数据
                    q_feedback = self.values[phy_id]
                    dq_feedback = self.values[phy_id + self.actuator_num]

                    u = (
                        msg.motor_command[i].tau
                        + msg.motor_command[i].kp
                        * (msg.motor_command[i].q - q_feedback) # 使用物理反馈
                        + msg.motor_command[i].kd
                        * (msg.motor_command[i].dq - dq_feedback) # 使用物理反馈
                    )
                    self.ctrl_buffer[phy_id].append(u)

    def publish_data(self):
        with self.lock:
            self.publish_imu_data()
            self.publish_joint_state()
            x, y, z = self.get_base_position()
            # self.get_logger().info(f"Base Pos: {x:.3f}, {y:.3f}, {z:.3f}")

    def publish_joint_state(self):
        robot_state_msg = RobotState()
        for i in range(self.actuator_num): # i 是逻辑ID
            # 获取对应的物理ID
            phy_id = self.motor_map[i]

            motor_state = MotorState() 

            motor_state.q = float(self.values[i])+np.random.normal(0, config.ENCODER_POSITION_NOISE) - self.encoder_position_bias[i]    
            motor_state.dq = float(self.values[self.actuator_num + i])+np.random.normal(0, config.ENCODER_VELOCITY_NOISE) + self.encoder_velocity_bias[i]    
            # 使用 phy_id 从物理数据源中读取数据
            # 注意：self.values 的索引是 [0~11]位置, [self.actuator_num~23]速度
            # 噪声也加在读取到的物理数据上
            
            # 读取位置 (phy_id)
            base_q = float(self.values[phy_id])
            # 读取速度 (phy_id + self.actuator_num)
            base_dq = float(self.values[self.actuator_num + phy_id])

            motor_state.q = base_q + np.random.normal(0, config.ENCODER_POSITION_NOISE) + self.encoder_position_bias[phy_id]    
            motor_state.dq = base_dq + np.random.normal(0, config.ENCODER_VELOCITY_NOISE) + self.encoder_velocity_bias[phy_id]    
            
            motor_state.ddq = 0.0                         
            motor_state.tau_est = 0.0                    
            motor_state.cur = 0.0                   

            robot_state_msg.motor_state.append(motor_state)
        self.joint_publisher.publish(robot_state_msg)

    def publish_imu_data(self):
        imu_msg = Imu()

        angle_noise_static = np.random.normal(0,0.7, 3)
        angle_noise_static_filtered = np.array([0.0,0.0,0.0])
        angle_noise_static_filtered = 0.05*angle_noise_static+0.95*angle_noise_static_filtered
        angle_noise = np.random.normal(angle_noise_static_filtered, config.NOISE_QUAT, 3)
        angle_noise_filtered = np.array([0.0,0.0,0.0])
        angle_noise_filtered = 0.1*angle_noise+0.9*angle_noise_filtered
        q_noised = self.quat_multiply(self.axis_angle_to_quat(angle_noise_filtered), np.array([self.values[24],self.values[25],self.values[26],self.values[27],]))
        q_norm = np.linalg.norm(q_noised)
        if q_norm > 0:
            q_noised /= q_norm
        else:
            q_noised = np.array([1.0, 0.0, 0.0, 0.0])

        imu_msg.orientation.w = q_noised[0]
        imu_msg.orientation.x = q_noised[1]
        imu_msg.orientation.y = q_noised[2]
        imu_msg.orientation.z = q_noised[3]

        # imu_msg.orientation.w = self.values[24]
        # imu_msg.orientation.x = self.values[25]
        # imu_msg.orientation.y = self.values[26]
        # imu_msg.orientation.z = self.values[27]
    #     gyro_meas = np.array([
    # self.values[28] + np.random.normal(0, config.NOISE_GYRO),
    # self.values[29] + np.random.normal(0, config.NOISE_GYRO),
    # self.values[30] + np.random.normal(0, config.NOISE_GYRO),])
    #     acc_meas = np.array([
    # self.values[31] + np.random.normal(0, config.NOISE_ACC),
    # self.values[32] + np.random.normal(0, config.NOISE_ACC),
    # self.values[33] + np.random.normal(0, config.NOISE_ACC),])
    #     q_noised = self.imu_efk.update(gyro_meas, acc_meas)
    #     imu_msg.orientation.w = q_noised[0]
    #     imu_msg.orientation.x = q_noised[1]
    #     imu_msg.orientation.y = q_noised[2]
    #     imu_msg.orientation.z = q_noised[3]

        imu_msg.angular_velocity.x = self.values[28]+np.random.normal(0,config.NOISE_GYRO)
        imu_msg.angular_velocity.y = self.values[29]+np.random.normal(0,config.NOISE_GYRO)
        imu_msg.angular_velocity.z = self.values[30]+np.random.normal(0,config.NOISE_GYRO)

        imu_msg.linear_acceleration.x = self.values[31]+np.random.normal(0,config.NOISE_ACC)
        imu_msg.linear_acceleration.y = self.values[32]+np.random.normal(0,config.NOISE_ACC)
        imu_msg.linear_acceleration.z = self.values[33]+np.random.normal(0,config.NOISE_ACC)

        self.imu_publisher.publish(imu_msg)

    @staticmethod
    def quat_multiply(q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])

    @staticmethod
    def axis_angle_to_quat(axis_angle):
        angle = np.linalg.norm(axis_angle)
        if angle < 1e-12:
            return np.array([1, 0, 0, 0])  # 无旋转

        axis = axis_angle / angle
        half = angle * 0.5
        return np.array([
            np.cos(half),
            axis[0] * np.sin(half),
            axis[1] * np.sin(half),
            axis[2] * np.sin(half),
        ])
    
    def reset_delay(self):
        self.ctrl_buffer = []
        self.joint_state_buffer = []
        for i in range(self.actuator_num):
            delay_time = np.random.uniform(0, config.MOTOR_DELAY)
            delay_steps = max(1, int(delay_time / self.model.opt.timestep))

            self.ctrl_buffer.append(
                deque([0.0] * delay_steps, maxlen=delay_steps)
            )
            self.joint_state_buffer.append(
                deque([(0.0, 0.0)] * delay_steps, maxlen=delay_steps)
            )

def main(args=None):
    rclpy.init(args=args)
    node = MujocoNode("mujoco_runner")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
