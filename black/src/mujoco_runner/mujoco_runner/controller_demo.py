#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_msgs.msg import RobotCommand, MotorCommand, RobotState
import threading
import math
import sys
import select
import termios
import tty

# --------------------- 获取按键工具函数 ---------------------
def get_key():
    """
    捕获单个按键，不需要回车
    """
    settings = termios.tcgetattr(sys.stdin)
    try:
        # 将终端设置为原始模式，不回显，不需要回车
        tty.setraw(sys.stdin.fileno())
        # select用于非阻塞检查，但这里我们需要阻塞等待按键，所以直接read
        # rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        # if rlist:
        key = sys.stdin.read(1)
        # else:
        #     key = ''
    finally:
        # 还原终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # 创建一个线程锁
        self.lock = threading.Lock()

        # 状态是否初始化
        self.motor_count = None            # 电机数量（首次收到状态后动态确定）
        self.got_first_state = False

        # 控制相关变量
        self.current_q = []               # 实时角度
        self.start_q = []                 # 插值起点
        self.target_q = [
                -0.0, -0.8014, 1.527, # FR
                 0.0,  0.8014, -1.527, # FL
                -0.0, -0.9, 1.527, # RR
                 0.0,  0.9, -1.527  #RL 
            ]
        self.control_state = "PASSIVE"    # PASSIVE / MOVE / HOLD
        self.start_time = 0.0
        self.duration = 3.0               # 插值时长

        # 订阅
        self.sub = self.create_subscription(
            RobotState,
            '/robot_joint_controller/state',
            self.state_callback,
            10
        )

        # 发布
        self.pub = self.create_publisher(
            RobotCommand,
            '/robot_joint_controller/command',
            10
        )

        # 500Hz 控制频率
        self.timer = self.create_timer(0.002, self.timer_callback)

        self.get_logger().info("=====================================")
        self.get_logger().info(" 机器人控制器已启动")
        self.get_logger().info(" 按 'g' 进入软启动，按 's' 返回被动模式")
        self.get_logger().info(" 按 Ctrl+C 退出")
        self.get_logger().info("=====================================")

    # ---------------------- 状态回调 ----------------------
    def state_callback(self, msg):
        """ 接收底层电机状态 """
        if not msg.motor_state:
            return

        # 第一次收到状态 → 初始化 motor 数量
        if not self.got_first_state:
            self.motor_count = len(msg.motor_state)
            self.get_logger().info(f"检测到 {self.motor_count} 个电机，初始化控制器。")

            # 初始化数组
            self.current_q = [0.0] * self.motor_count
            self.start_q = [0.0] * self.motor_count
            
            # 格式化打印
            target_str = " ".join([f"{self.target_q[i]:.2f}" for i in range(self.motor_count)])
            print(f'target_q: {target_str}')

            self.got_first_state = True

        # 读取当前角度
        for i in range(self.motor_count):
            self.current_q[i] = msg.motor_state[i].q

    # ---------------------- 控制循环 ----------------------
    def timer_callback(self):
        if not self.got_first_state:
            return

        msg = RobotCommand()

        # 获取时间
        t = self.get_clock().now().nanoseconds / 1e9

        # 生成控制命令数组
        cmd_q = [0.0] * self.motor_count
        cmd_kp = 0.0
        cmd_kd = 1.0

        # 加锁
        with self.lock:
            # ----------- 控制模式 ------------
            if self.control_state == "PASSIVE":
                cmd_q = self.current_q[:]
                cmd_kp = 0.0
                print(''.join(f"{i}:{self.current_q[i]:.2f} " for i in range(self.motor_count)), end='\r')

            elif self.control_state == "MOVE":
                dt = t - self.start_time
                # 这里的逻辑在锁内，保证 start_time 和 control_state 是匹配的
                if dt < self.duration:
                    alpha = dt / self.duration
                    smooth = (1 - math.cos(math.pi * alpha)) / 2.0
                    for i in range(self.motor_count):
                        cmd_q[i] = self.start_q[i] + (self.target_q[i] - self.start_q[i]) * smooth
                    cmd_kp = 40.0
                else:
                    self.control_state = "HOLD"
                    cmd_q = self.target_q[:]
                    cmd_kp = 40.0
                    self.get_logger().info("运动完成，进入 HOLD 模式。")

            elif self.control_state == "HOLD":
                cmd_q = self.target_q[:]
                cmd_kp = 40.0
        

        # ----------- 填充消息 ------------
        for i in range(self.motor_count):
            cmd = MotorCommand()
            cmd.q = float(cmd_q[i])
            cmd.dq = 0.0
            cmd.tau = 0.0
            cmd.kp = float(cmd_kp)
            cmd.kd = float(cmd_kd)
            msg.motor_command.append(cmd)

        self.pub.publish(msg)

    # ---------------------- 模式切换 ----------------------
    def start_moving(self):
        if not self.got_first_state:
            self.get_logger().warn("未收到状态，不能开始运动")
            return

        # 保证赋值过程不被打断
        with self.lock:
            self.start_q = self.current_q[:]
            self.start_time = self.get_clock().now().nanoseconds / 1e9
            self.control_state = "MOVE"

        self.get_logger().info("开始软启动插值动作")

    def return_to_listen(self):
        # 加锁
        with self.lock:
            self.control_state = "PASSIVE"
        self.get_logger().info("切换回 PASSIVE（阻尼模式）")


# --------------------- 键盘线程 ---------------------
def keyboard_listener(node):
    # 说明：在 Raw 模式下，Ctrl+C 也是一个普通字符 '\x03'，需要手动捕获退出
    try:
        while rclpy.ok():
            key = get_key()
            if key == 'g':
                node.start_moving()
            elif key == 's':
                node.return_to_listen()
            elif key == '\x03': # ASCII 3 是 Ctrl+C
                node.get_logger().info("检测到 Ctrl+C，准备退出...")
                break
    except Exception as e:
        print(e)

# --------------------- 主入口 ---------------------
def main(args=None):
    rclpy.init(args=args)
    node = Controller()

    # 启动键盘监听线程
    thread = threading.Thread(target=keyboard_listener, args=(node,), daemon=True)
    thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 确保销毁节点
        node.destroy_node()
        rclpy.shutdown()
        # 等待线程结束（可选，因为设置了 daemon=True，主线程结束它也会随之结束）
        # thread.join() 

if __name__ == '__main__':
    main()