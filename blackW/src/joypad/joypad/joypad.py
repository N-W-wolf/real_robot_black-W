import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pygame
from . import config

class JoyPublisher(Node):
    def __init__(self):
        super().__init__('joy_publisher')

        self.publisher_ = self.create_publisher(Joy, 'joy', 10)
        self.timer = self.create_timer(config.JOYPAD_DT, self.timer_callback)

        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No joystick detected")
            exit()

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        
        # 选择映射配置
        joystick_name = self.joystick.get_name()
        self.get_logger().info(f"Joystick detected: {joystick_name}")
        
        # 根据名字自动选择，或者默认使用盖世小鸡
        if "XBOX" in joystick_name.upper(): 
             self.map = config.StandardXbox() # 假设你有一个标准配置
        else:
             self.map = config.GaiShiXiaoJi()

        # 定义轴和按键的数量
        self.num_axes_output = 8  # 0-5是模拟轴，6-7是十字键
        self.num_buttons_output = 12 # 足够覆盖常用按键
        
        self.DPAD_X_AXIS = 6
        self.DPAD_Y_AXIS = 7

        # 初始化状态数组
        self.axes = [0.0] * self.num_axes_output
        self.buttons = [0] * self.num_buttons_output

    def timer_callback(self):
        # 1. 处理事件 (按键 + 十字键)
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN or event.type == pygame.JOYBUTTONUP:
                value = 1 if event.type == pygame.JOYBUTTONDOWN else 0
                mapped = self.map.map_button(event.button)
                if mapped < len(self.buttons):
                    self.buttons[mapped] = value

            elif event.type == pygame.JOYHATMOTION:
                x, y = event.value
                # 从 config 中获取 D-Pad 映射的目标轴索引
                # getattr(..., {}, 6) 的意思是：如果配置里没写，默认用 6 和 7
                dpad_config = getattr(self.map, 'dpad_mapping', {'x': 6, 'y': 7})
                target_x = dpad_config.get('x', 6)
                target_y = dpad_config.get('y', 7)

                # 填充数据
                if target_x < len(self.axes):
                    self.axes[target_x] = float(x)
                if target_y < len(self.axes):
                    self.axes[target_y] = float(y)

        # 2. 处理模拟轴 (循环部分)
        num_physical_axes = self.joystick.get_numaxes()
        for i in range(num_physical_axes):
            mapped = self.map.map_axis(i)

            dpad_config = getattr(self.map, 'dpad_mapping', {'x': 6, 'y': 7})
            dpad_x_idx = dpad_config.get('x', 6)
            dpad_y_idx = dpad_config.get('y', 7)
            
            # 只有当当前轴不是 D-Pad 占用的轴时，才更新
            if mapped < len(self.axes) and mapped != dpad_x_idx and mapped != dpad_y_idx:
                scale = getattr(self.map, 'axis_scales', {}).get(i, 1.0)
                raw_val = float(self.joystick.get_axis(i))

                # 应用缩放 (自动取反)
                self.axes[mapped] = raw_val * scale

        # 3. 发布消息
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "joy"
        msg.axes = self.axes
        msg.buttons = self.buttons
        self.publisher_.publish(msg)


def main(args=None):

    rclpy.init(args=args)
    node = JoyPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()