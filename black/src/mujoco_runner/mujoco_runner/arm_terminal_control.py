import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_srvs.srv import SetBool


HELP_TEXT = """Terminal arm control:
  1/2/3 : select trajectory and start
  Space : toggle current trajectory
  h     : return arm to home pose
  n     : select next trajectory and start
  q     : quit terminal controller
"""


class ArmTerminalControl(Node):
    def __init__(self):
        super().__init__("arm_terminal_control")
        self.select_pub = self.create_publisher(Int32, "/arm_motion/select", 10)
        self.enable_client = self.create_client(SetBool, "/arm_motion/enable")
        self.current_traj = 0
        self.traj_count = 3
        self.arm_enabled = False

    def wait_until_ready(self):
        while rclpy.ok() and not self.enable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /arm_motion/enable service...")

    def set_enabled(self, enabled):
        request = SetBool.Request()
        request.data = bool(enabled)
        future = self.enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is None:
            error = future.exception()
            raise RuntimeError(f"Failed to call /arm_motion/enable: {error}")
        self.arm_enabled = bool(enabled)
        return future.result()

    def select_trajectory(self, traj_id, auto_enable=False):
        msg = Int32()
        msg.data = int(traj_id)
        self.select_pub.publish(msg)
        self.current_traj = int(traj_id)
        self.get_logger().info(f"Selected trajectory {self.current_traj + 1}")
        if auto_enable:
            response = self.set_enabled(True)
            self.get_logger().info(response.message)

    def handle_key(self, key):
        if key in ("1", "2", "3"):
            self.select_trajectory(int(key) - 1, auto_enable=True)
            return True

        if key == " ":
            response = self.set_enabled(not self.arm_enabled)
            self.get_logger().info(response.message)
            return True

        if key in ("h", "H"):
            response = self.set_enabled(False)
            self.get_logger().info(response.message)
            return True

        if key in ("n", "N"):
            next_traj = (self.current_traj + 1) % self.traj_count
            self.select_trajectory(next_traj, auto_enable=True)
            return True

        if key in ("q", "Q"):
            self.get_logger().info("Exiting terminal controller")
            return False

        return True


def main(args=None):
    rclpy.init(args=args)
    node = ArmTerminalControl()
    node.wait_until_ready()

    if not sys.stdin.isatty():
        node.get_logger().error("stdin is not a TTY, terminal hotkeys are unavailable")
        node.destroy_node()
        rclpy.shutdown()
        return

    print(HELP_TEXT, flush=True)
    old_settings = termios.tcgetattr(sys.stdin.fileno())

    try:
        tty.setcbreak(sys.stdin.fileno())
        keep_running = True
        while rclpy.ok() and keep_running:
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not ready:
                continue
            key = sys.stdin.read(1)
            keep_running = node.handle_key(key)
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
