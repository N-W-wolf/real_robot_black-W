import time

import pygame
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from . import config


class JoyPublisher(Node):
    def __init__(self):
        super().__init__('joy_publisher')

        self.publisher_ = self.create_publisher(Joy, 'joy', 10)
        self.timer = self.create_timer(config.JOYPAD_DT, self.timer_callback)

        self.num_axes_output = 8
        self.num_buttons_output = 12
        self.dpad_default_mapping = {'x': 6, 'y': 7}
        self.axes = [0.0] * self.num_axes_output
        self.buttons = [0] * self.num_buttons_output

        self.joystick = None
        self.joystick_instance_id = None
        self.map = config.GaiShiXiaoJi()
        self.next_reconnect_at = 0.0
        self.next_missing_warn_at = 0.0

        pygame.init()
        pygame.joystick.init()
        self._connect_first_available(log_missing=True)

    def _reset_state(self):
        self.axes = [0.0] * self.num_axes_output
        self.buttons = [0] * self.num_buttons_output

    def _select_mapping(self, joystick_name):
        if "XBOX" in joystick_name.upper():
            return config.StandardXbox()
        return config.GaiShiXiaoJi()

    def _connect_first_available(self, log_missing=False):
        now = time.monotonic()
        if now < self.next_reconnect_at:
            return False
        self.next_reconnect_at = now + 0.5

        pygame.joystick.quit()
        pygame.joystick.init()
        count = pygame.joystick.get_count()
        if count == 0:
            if log_missing and now >= self.next_missing_warn_at:
                self.get_logger().warn("No joystick detected; publishing neutral Joy until it reconnects")
                self.next_missing_warn_at = now + 5.0
            return False

        for index in range(count):
            try:
                joystick = pygame.joystick.Joystick(index)
                joystick.init()
                joystick_name = joystick.get_name()
                self.joystick = joystick
                self.joystick_instance_id = self._get_instance_id(joystick)
                self.map = self._select_mapping(joystick_name)
                self._reset_state()
                self.get_logger().warn(f"Joystick connected: {joystick_name}")
                return True
            except pygame.error as exc:
                self.get_logger().warn(f"Failed to initialize joystick {index}: {exc}")
        return False

    def _disconnect(self, reason):
        if self.joystick is not None:
            self.get_logger().warn(f"Joystick disconnected: {reason}; publishing neutral Joy")
            try:
                self.joystick.quit()
            except pygame.error:
                pass
        self.joystick = None
        self.joystick_instance_id = None
        self._reset_state()

    def _get_instance_id(self, joystick):
        get_instance_id = getattr(joystick, 'get_instance_id', None)
        if get_instance_id is None:
            return None
        try:
            return get_instance_id()
        except pygame.error:
            return None

    def _event_belongs_to_active_joystick(self, event):
        if self.joystick is None:
            return False
        if self.joystick_instance_id is None:
            return True
        return getattr(event, 'instance_id', self.joystick_instance_id) == self.joystick_instance_id

    def _handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rclpy.shutdown()
                return

            if event.type == getattr(pygame, 'JOYDEVICEADDED', object()):
                if self.joystick is None:
                    self._connect_first_available()
                continue

            if event.type == getattr(pygame, 'JOYDEVICEREMOVED', object()):
                if self._event_belongs_to_active_joystick(event):
                    self._disconnect("device removed")
                continue

            if not self._event_belongs_to_active_joystick(event):
                continue

            if event.type in (pygame.JOYBUTTONDOWN, pygame.JOYBUTTONUP):
                value = 1 if event.type == pygame.JOYBUTTONDOWN else 0
                mapped = self.map.map_button(event.button)
                if 0 <= mapped < len(self.buttons):
                    self.buttons[mapped] = value

            elif event.type == pygame.JOYHATMOTION:
                x, y = event.value
                dpad_config = getattr(self.map, 'dpad_mapping', self.dpad_default_mapping)
                target_x = dpad_config.get('x', 6)
                target_y = dpad_config.get('y', 7)

                if 0 <= target_x < len(self.axes):
                    self.axes[target_x] = float(x)
                if 0 <= target_y < len(self.axes):
                    self.axes[target_y] = float(y)

    def _poll_axes(self):
        if self.joystick is None:
            return

        get_attached = getattr(self.joystick, 'get_attached', None)
        try:
            if get_attached is not None and not get_attached():
                self._disconnect("device detached")
                return

            num_physical_axes = self.joystick.get_numaxes()
            dpad_config = getattr(self.map, 'dpad_mapping', self.dpad_default_mapping)
            dpad_x_idx = dpad_config.get('x', 6)
            dpad_y_idx = dpad_config.get('y', 7)

            for i in range(num_physical_axes):
                mapped = self.map.map_axis(i)
                if 0 <= mapped < len(self.axes) and mapped not in (dpad_x_idx, dpad_y_idx):
                    scale = getattr(self.map, 'axis_scales', {}).get(i, 1.0)
                    self.axes[mapped] = float(self.joystick.get_axis(i)) * scale

            new_buttons = [0] * self.num_buttons_output
            for i in range(self.joystick.get_numbuttons()):
                mapped = self.map.map_button(i)
                if 0 <= mapped < len(new_buttons):
                    new_buttons[mapped] = max(new_buttons[mapped], int(self.joystick.get_button(i)))
            self.buttons = new_buttons

            if 0 <= dpad_x_idx < len(self.axes):
                self.axes[dpad_x_idx] = 0.0
            if 0 <= dpad_y_idx < len(self.axes):
                self.axes[dpad_y_idx] = 0.0
            if self.joystick.get_numhats() > 0:
                x, y = self.joystick.get_hat(0)
                if 0 <= dpad_x_idx < len(self.axes):
                    self.axes[dpad_x_idx] = float(x)
                if 0 <= dpad_y_idx < len(self.axes):
                    self.axes[dpad_y_idx] = float(y)
        except pygame.error as exc:
            self._disconnect(str(exc))

    def _publish(self):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "joy_connected" if self.joystick is not None else "joy_disconnected"
        msg.axes = list(self.axes)
        msg.buttons = list(self.buttons)
        self.publisher_.publish(msg)

    def timer_callback(self):
        self._handle_events()
        if self.joystick is None:
            self._connect_first_available(log_missing=True)
        self._poll_axes()
        self._publish()

    def destroy_node(self):
        if self.joystick is not None:
            try:
                self.joystick.quit()
            except pygame.error:
                pass
        pygame.joystick.quit()
        pygame.quit()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoyPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
