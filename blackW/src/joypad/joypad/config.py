JOYPAD_DT=0.01

class GamepadMapping:
    def __init__(self):
        if not hasattr(self, 'button_mapping'):
            self.button_mapping = {}
        if not hasattr(self, 'axis_mapping'):
            self.axis_mapping = {}
    
    def map_button(self, button_id):
        return self.button_mapping.get(button_id, button_id)
    
    def map_axis(self, axis_id):
        return self.axis_mapping.get(axis_id, axis_id)

class StandardXbox(GamepadMapping):
    def __init__(self):
        super().__init__()
        
        # 按钮映射 (Pygame ID -> ROS Message Index)
        # 基于标准 Xbox 控制器布局
        self.button_mapping = {
            0: 0,  # A (Cross)       -> 对应 C++ buttons[0]
            1: 1,  # B (Circle)      -> 对应 C++ buttons[1]
            2: 2,  # X (Square)      -> 对应 C++ buttons[2]
            3: 3,  # Y (Triangle)    -> 对应 C++ buttons[3]
            4: 4,  # LB (Left Bumper)-> 对应 C++ buttons[4]
            5: 5,  # RB (Right Bumper)-> 对应 C++ buttons[5]
            6: 6,  # Back / View     -> 对应 C++ 暂无直接逻辑
            7: 7,  # Start / Menu    -> 对应 C++ 暂无直接逻辑
            8: 8,  # Power / Xbox    -> 对应 C++ buttons[8] (Power)
            9: 9,  # LS (Left Stick Click)  -> 对应 C++ buttons[9]
            10: 10 # RS (Right Stick Click) -> 对应 C++ buttons[10]
        }
        
        # 轴映射 (Pygame ID -> ROS Message Index)
        self.axis_mapping = {
            0: 0, # Left Stick X (左右平移)
            1: 1, # Left Stick Y (前后移动)
            2: 2, # LT (Left Trigger)
            3: 3, # Right Stick X (旋转 Yaw)
            4: 4, # Right Stick Y (通常用于云台俯仰，你的C++里暂未使用)
            5: 5  # RT (Right Trigger)
        }
        
        # 缩放系数
        # 将 Y 轴 (ID 1 和 4) 设为 -1.0
        self.axis_scales = {
            0: -1.0,
            1: -1.0,
            3: -1.0,
            4: -1.0,
            2: 1.0, # LT 保持默认
            5: 1.0  # RT 保持默认
        }

        # D-Pad (Hat) 映射
        self.dpad_mapping = {
            'x': 6,  # Hat X -> Joy Axis 6 (左右)
            'y': 7   # Hat Y -> Joy Axis 7 (上下)
        }

class GaiShiXiaoJi(GamepadMapping):
    def __init__(self):
        super().__init__()
        self.button_mapping = {
            0: 0,
            1: 1,
            2: 2,
            3: 2,
            4: 3,
            5: 5,
            6: 4,
            7: 5,
            8: 8,
            9: 9,
            10: 6,
            11: 7,
            12: 8,
            13: 9,
            14: 10
        }
        self.axis_mapping = {
            3: 4,
            2: 3,
            5: 2,
            4: 5
        }

class ShaMoHu(GamepadMapping):
    def __init__(self):
        super().__init__()
        self.button_mapping = {
            0: 0,
            1: 1,
            2: 2,
            3: 2,
            4: 3,
            5: 5,
            6: 4,
            7: 5,
            8: 8,
            9: 9,
            10: 6,
            11: 7,
            12: 8,
            13: 9,
            14: 10
        }
        self.axis_mapping = {
            7: 4,
            6: 3,
            5: 2,
            4: 5
        }