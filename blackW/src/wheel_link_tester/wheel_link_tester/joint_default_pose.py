import math
import sys
import termios
import threading
import time
import traceback
import tty

import rclpy
from rclpy.node import Node

from robot_msgs.msg import MotorCommand
from robot_msgs.msg import RobotCommand
from robot_msgs.msg import RobotState


COMMAND_TOPIC = '/_lowCmd/command'
STATE_TOPIC = '/_lowState/joint'
MOTOR_COUNT = 16
RATE_HZ = 50
INTERPOLATION_DURATION_SEC = 5.0
STATE_TIMEOUT_SEC = 2.0
PASSIVE_KD = 1.0
SHUTDOWN_ZERO_SEC = 0.5
CONTROL_PASSIVE = 'PASSIVE'
CONTROL_MOVE = 'MOVE'
CONTROL_HOLD = 'HOLD'

DEFAULT_JOINT_TARGETS = {
    0: ('FR_hip_joint', -0.0),
    1: ('FR_thigh_joint', -0.8014),
    2: ('FR_calf_joint', 1.527),
    4: ('FL_hip_joint', 0.0),
    5: ('FL_thigh_joint', 0.8014),
    6: ('FL_calf_joint', -1.527),
    8: ('RR_hip_joint', -0.0),
    9: ('RR_thigh_joint', -0.8014),
    10: ('RR_calf_joint', 1.527),
    12: ('RL_hip_joint', 0.0),
    13: ('RL_thigh_joint', 0.8014),
    14: ('RL_calf_joint', -1.527),
}

WHEEL_INDICES = (3, 7, 11, 15)


def get_key() -> str:
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class JointDefaultPose(Node):
    def __init__(self):
        super().__init__('joint_default_pose')

        self.declare_parameter('topic', COMMAND_TOPIC)
        self.declare_parameter('rate', RATE_HZ)
        self.declare_parameter('kp', 80.0)
        self.declare_parameter('kd', 3.0)
        self.declare_parameter('state_topic', STATE_TOPIC)
        self.declare_parameter('duration', INTERPOLATION_DURATION_SEC)
        self.declare_parameter('state_timeout', STATE_TIMEOUT_SEC)
        self.declare_parameter('passive_kd', PASSIVE_KD)

        self.topic = str(self.get_parameter('topic').value)
        self.rate = int(self.get_parameter('rate').value)
        self.kp = float(self.get_parameter('kp').value)
        self.kd = float(self.get_parameter('kd').value)
        self.state_topic = str(self.get_parameter('state_topic').value)
        self.duration = float(self.get_parameter('duration').value)
        self.state_timeout = float(self.get_parameter('state_timeout').value)
        self.passive_kd = float(self.get_parameter('passive_kd').value)

        self._validate_parameters()

        self.command_pub = self.create_publisher(RobotCommand, self.topic, 10)
        self.state_sub = self.create_subscription(
            RobotState,
            self.state_topic,
            self._state_callback,
            10,
        )
        self.zero_msg = self._make_zero_command()

        self.last_state: RobotState | None = None
        self.last_state_at: float | None = None
        self.last_state_size: int | None = None
        self.start_time = time.monotonic()
        self.ramp_start_at: float | None = None
        self.ramp_start_q: dict[int, float] = {}
        self.waiting_error_logged = False
        self.state_stale_logged = False
        self.invalid_state_logged = False
        self.nonfinite_state_logged = False
        self.reached_target_logged = False
        self.lock = threading.Lock()
        self.control_state = CONTROL_PASSIVE
        self.shutdown_requested = False

        self.timer = self.create_timer(1.0 / self.rate, self._timer_callback)

        self.get_logger().debug(
            f'publishing {MOTOR_COUNT}-way interpolated joint pose command to '
            f'{self.topic} at {self.rate}Hz; subscribing state from '
            f'{self.state_topic}; duration={self.duration:.2f}s '
            f'kp={self.kp:.3f} kd={self.kd:.3f} '
            f'passive_kd={self.passive_kd:.3f}'
        )
        self.get_logger().debug(
            'joint slots: '
            + ', '.join(
                f'{index}:{name}={q:.4f}'
                for index, (name, q) in DEFAULT_JOINT_TARGETS.items()
            )
            + f'; wheel slots zeroed: {list(WHEEL_INDICES)}'
        )
        self.get_logger().debug(
            "keyboard: press 'g' to start interpolation, 's' for passive, "
            'Ctrl+C to exit'
        )

    def _reset_ramp(self):
        self.ramp_start_at = None
        self.ramp_start_q = {}
        self.reached_target_logged = False

    def _validate_parameters(self):
        if not self.topic:
            raise ValueError('topic must not be empty')
        if not self.state_topic:
            raise ValueError('state_topic must not be empty')
        if self.rate <= 0:
            raise ValueError('rate must be > 0')
        if self.kp < 0.0:
            raise ValueError('kp must be >= 0')
        if self.kd < 0.0:
            raise ValueError('kd must be >= 0')
        if self.duration <= 0.0:
            raise ValueError('duration must be > 0')
        if self.state_timeout <= 0.0:
            raise ValueError('state_timeout must be > 0')
        if self.passive_kd < 0.0:
            raise ValueError('passive_kd must be >= 0')

    def _state_callback(self, msg: RobotState):
        self.last_state = msg
        self.last_state_at = time.monotonic()

        state_size = len(msg.motor_state)
        if state_size != self.last_state_size:
            self.last_state_size = state_size
            if state_size == MOTOR_COUNT:
                self.get_logger().debug(f'received RobotState with {state_size} motors')
                self.invalid_state_logged = False
            else:
                self.get_logger().error(
                    'received RobotState with invalid motor_state size: '
                    f'expected {MOTOR_COUNT}, got {state_size}'
                )

    def _state_is_ready(self, now: float) -> bool:
        if self.last_state is None or self.last_state_at is None:
            if now - self.start_time >= self.state_timeout and not self.waiting_error_logged:
                self.waiting_error_logged = True
                self.get_logger().error(
                    f'no RobotState received on {self.state_topic} after '
                    f'{self.state_timeout:.2f}s; holding zero command'
                )
            return False

        if now - self.last_state_at > self.state_timeout:
            if not self.state_stale_logged:
                self.state_stale_logged = True
                self.get_logger().error(
                    'state timeout: no fresh RobotState for %.2fs; holding zero command'
                    % (now - self.last_state_at)
                )
            self._reset_ramp()
            return False
        self.state_stale_logged = False

        if len(self.last_state.motor_state) != MOTOR_COUNT:
            if not self.invalid_state_logged:
                self.invalid_state_logged = True
                self.get_logger().error(
                    f'latest RobotState size is {len(self.last_state.motor_state)}, '
                    f'expected {MOTOR_COUNT}; holding zero command'
                )
            self._reset_ramp()
            return False

        for index, motor_state in enumerate(self.last_state.motor_state):
            if not math.isfinite(float(motor_state.q)):
                if not self.nonfinite_state_logged:
                    self.nonfinite_state_logged = True
                    self.get_logger().error(
                        f'invalid non-finite q at motor index {index}; '
                        'holding zero command'
                    )
                self._reset_ramp()
                return False

        self.nonfinite_state_logged = False
        return True

    def _start_ramp(self, now: float):
        if self.last_state is None:
            return

        self.ramp_start_at = now
        self.ramp_start_q = {
            index: float(self.last_state.motor_state[index].q)
            for index in DEFAULT_JOINT_TARGETS
        }

        self.get_logger().debug(
            'starting %.2fs interpolation from current state to default joint pose'
            % self.duration
        )

    def _make_interpolated_command(self, alpha: float) -> RobotCommand:
        msg = RobotCommand()
        smooth_alpha = alpha * alpha * (3.0 - 2.0 * alpha)

        for index in range(MOTOR_COUNT):
            cmd = MotorCommand()
            cmd.dq = 0.0
            cmd.tau = 0.0

            if index in DEFAULT_JOINT_TARGETS:
                _, target_q = DEFAULT_JOINT_TARGETS[index]
                start_q = self.ramp_start_q[index]
                cmd.q = start_q + (float(target_q) - start_q) * smooth_alpha
                cmd.kp = float(self.kp)
                cmd.kd = float(self.kd)
            else:
                cmd.q = 0.0
                cmd.kp = 0.0
                cmd.kd = 0.0

            msg.motor_command.append(cmd)

        return msg

    def _make_target_command(self) -> RobotCommand:
        msg = RobotCommand()

        for index in range(MOTOR_COUNT):
            cmd = MotorCommand()
            cmd.dq = 0.0
            cmd.tau = 0.0

            if index in DEFAULT_JOINT_TARGETS:
                _, target_q = DEFAULT_JOINT_TARGETS[index]
                cmd.q = float(target_q)
                cmd.kp = float(self.kp)
                cmd.kd = float(self.kd)
            else:
                cmd.q = 0.0
                cmd.kp = 0.0
                cmd.kd = 0.0

            msg.motor_command.append(cmd)

        return msg

    def _make_passive_command(self) -> RobotCommand:
        if self.last_state is None:
            return self.zero_msg

        msg = RobotCommand()

        for index in range(MOTOR_COUNT):
            cmd = MotorCommand()
            cmd.dq = 0.0
            cmd.tau = 0.0

            if index in DEFAULT_JOINT_TARGETS:
                cmd.q = float(self.last_state.motor_state[index].q)
                cmd.kp = 0.0
                cmd.kd = float(self.passive_kd)
            else:
                cmd.q = 0.0
                cmd.kp = 0.0
                cmd.kd = 0.0

            msg.motor_command.append(cmd)

        return msg

    def _make_zero_command(self) -> RobotCommand:
        msg = RobotCommand()

        for _ in range(MOTOR_COUNT):
            cmd = MotorCommand()
            cmd.q = 0.0
            cmd.dq = 0.0
            cmd.tau = 0.0
            cmd.kp = 0.0
            cmd.kd = 0.0
            msg.motor_command.append(cmd)

        return msg

    def _timer_callback(self):
        now = time.monotonic()

        if self.shutdown_requested:
            self.command_pub.publish(self.zero_msg)
            return

        if not self._state_is_ready(now):
            self.command_pub.publish(self.zero_msg)
            return

        with self.lock:
            control_state = self.control_state

        if control_state == CONTROL_PASSIVE:
            self._reset_ramp()
            self.command_pub.publish(self._make_passive_command())
            return

        if control_state == CONTROL_HOLD:
            self.command_pub.publish(self._make_target_command())
            return

        if control_state != CONTROL_MOVE:
            self.get_logger().error(
                f'unknown control state {control_state!r}; switching to passive'
            )
            self.return_to_passive()
            self.command_pub.publish(self._make_passive_command())
            return

        if self.ramp_start_at is None:
            self._start_ramp(now)
        elapsed = now - self.ramp_start_at
        alpha = min(max(elapsed / self.duration, 0.0), 1.0)
        self.command_pub.publish(self._make_interpolated_command(alpha))

        if alpha >= 1.0 and not self.reached_target_logged:
            self.reached_target_logged = True
            with self.lock:
                if self.control_state == CONTROL_MOVE:
                    self.control_state = CONTROL_HOLD
            self.get_logger().debug('default joint pose reached; holding position')

    def start_moving(self):
        if not self._state_is_ready(time.monotonic()):
            self.get_logger().warn('state is not ready; cannot start interpolation')
            return

        with self.lock:
            self._reset_ramp()
            self.control_state = CONTROL_MOVE

        self.get_logger().debug('starting soft interpolation command')

    def return_to_passive(self):
        with self.lock:
            self.control_state = CONTROL_PASSIVE
            self._reset_ramp()

        self.get_logger().debug('switched to passive command')

    def request_shutdown(self):
        self.shutdown_requested = True
        self.get_logger().debug('shutdown requested; publishing zero commands')
        self.publish_zero_for(SHUTDOWN_ZERO_SEC)
        if rclpy.ok():
            rclpy.shutdown()

    def publish_zero_for(self, duration: float):
        end_time = time.monotonic() + duration
        period = 1.0 / self.rate
        while rclpy.ok() and time.monotonic() < end_time:
            self.command_pub.publish(self.zero_msg)
            time.sleep(period)


def keyboard_listener(node: JointDefaultPose):
    try:
        while rclpy.ok():
            key = get_key()
            if key == 'g':
                node.start_moving()
            elif key == 's':
                node.return_to_passive()
            elif key == '\x03':
                node.request_shutdown()
                break
    except Exception:
        if rclpy.ok():
            node.get_logger().error('keyboard listener failed')
            traceback.print_exc()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = JointDefaultPose()
        thread = threading.Thread(
            target=keyboard_listener,
            args=(node,),
            daemon=True,
        )
        thread.start()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().debug('interrupted; publishing shutdown zero commands')
            node.publish_zero_for(SHUTDOWN_ZERO_SEC)
    except Exception:
        if node is not None:
            node.get_logger().error('exception; publishing shutdown zero commands')
            node.publish_zero_for(SHUTDOWN_ZERO_SEC)
        traceback.print_exc()
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
