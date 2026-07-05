import math
import sys
import time
import traceback
from dataclasses import dataclass
from typing import Iterable

import rclpy
from rclpy.node import Node

from robot_msgs.msg import MotorCommand
from robot_msgs.msg import RobotCommand
from robot_msgs.msg import RobotState


MOTOR_COUNT = 16
WHEEL_INDICES = (3, 7, 11, 15)
VALID_MODES = ('passive', 'zero', 'single', 'all', 'sequence')
COMMAND_TOPIC = '/_lowCmd/command'
STATE_TOPIC = '/_lowState/joint'
ZERO_GAP_SEC = 1.0
SHUTDOWN_ZERO_SEC = 0.5
PRINT_INTERVAL_SEC = 1.0


@dataclass(frozen=True)
class Stage:
    name: str
    kind: str
    target_wheels: tuple[int, ...]
    duration: float


class WheelLinkTester(Node):
    def __init__(self):
        super().__init__('wheel_link_tester')

        self.declare_parameter('mode', 'sequence')
        self.declare_parameter('wheel', 0)
        self.declare_parameter('speed', 0.2)
        self.declare_parameter('kd', 0.5)
        self.declare_parameter('duration', 5.0)
        self.declare_parameter('rate', 50)
        self.declare_parameter('dq_threshold', 0.05)
        self.declare_parameter('unexpected_consecutive', 5)
        self.declare_parameter('unexpected_grace', 0.25)
        self.declare_parameter('state_timeout', 2.0)

        self.mode = self.get_parameter('mode').value
        self.mode = str(self.mode).lower()
        self.wheel = int(self.get_parameter('wheel').value)
        self.speed = float(self.get_parameter('speed').value)
        self.kd = float(self.get_parameter('kd').value)
        self.duration = float(self.get_parameter('duration').value)
        self.rate = int(self.get_parameter('rate').value)
        self.dq_threshold = float(self.get_parameter('dq_threshold').value)
        self.unexpected_consecutive = int(
            self.get_parameter('unexpected_consecutive').value
        )
        self.unexpected_grace = float(self.get_parameter('unexpected_grace').value)
        self.state_timeout = float(self.get_parameter('state_timeout').value)

        self._validate_parameters()

        self.command_pub = self.create_publisher(RobotCommand, COMMAND_TOPIC, 10)
        self.state_sub = self.create_subscription(
            RobotState,
            STATE_TOPIC,
            self._state_callback,
            10,
        )

        self.stages = self._build_stages()
        self.current_stage_index = -1
        self.current_stage: Stage | None = None
        self.stage_started_at: float | None = None
        self.stage_target_peaks: dict[int, float] = {}
        self.stage_unexpected_counts: dict[int, int] = {}
        self.last_print_at = 0.0

        self.last_state: RobotState | None = None
        self.last_state_at: float | None = None
        self.last_state_size: int | None = None
        self.start_time = time.monotonic()
        self.waiting_error_logged = False
        self.state_stale_logged = False
        self.invalid_state_logged = False
        self.active_disabled = False

        self.failed = False
        self.done = False
        self.stop_until: float | None = None

        self.timer = self.create_timer(1.0 / self.rate, self._timer_callback)

        self.get_logger().debug(
            f'wheel_link_tester mode={self.mode} wheel={self.wheel} '
            f'ros_index={WHEEL_INDICES[self.wheel]} speed={self.speed:.3f} '
            f'kd={self.kd:.3f} duration={self.duration:.2f}s '
            f'rate={self.rate}Hz dq_threshold={self.dq_threshold:.3f} '
            f'unexpected_consecutive={self.unexpected_consecutive} '
            f'unexpected_grace={self.unexpected_grace:.2f}s'
        )
        self.get_logger().debug(
            f'publishing {MOTOR_COUNT}-way RobotCommand to {COMMAND_TOPIC}; '
            f'subscribing RobotState from {STATE_TOPIC}'
        )

    def _validate_parameters(self):
        if self.mode not in VALID_MODES:
            raise ValueError(
                f"mode must be one of {', '.join(VALID_MODES)}, got {self.mode!r}"
            )
        if self.wheel < 0 or self.wheel >= len(WHEEL_INDICES):
            raise ValueError('wheel must be in range 0..3')
        if self.rate <= 0:
            raise ValueError('rate must be > 0')
        if self.duration <= 0.0:
            raise ValueError('duration must be > 0')
        if self.state_timeout <= 0.0:
            raise ValueError('state_timeout must be > 0')
        if self.dq_threshold < 0.0:
            raise ValueError('dq_threshold must be >= 0')
        if self.unexpected_consecutive <= 0:
            raise ValueError('unexpected_consecutive must be > 0')
        if self.unexpected_grace < 0.0:
            raise ValueError('unexpected_grace must be >= 0')
        if self.kd < 0.0:
            raise ValueError('kd must be >= 0')

    def _build_stages(self) -> list[Stage]:
        if self.mode == 'passive':
            return [Stage('passive feedback check', 'passive', (), self.duration)]
        if self.mode == 'zero':
            return [Stage('zero command check', 'zero', (), self.duration)]
        if self.mode == 'single':
            return [
                Stage(f'single wheel {self.wheel}', 'single', (self.wheel,), self.duration)
            ]
        if self.mode == 'all':
            return [Stage('all wheels', 'all', (0, 1, 2, 3), self.duration)]

        stages = [
            Stage('passive feedback check', 'passive', (), self.duration),
            Stage('zero command check', 'zero', (), self.duration),
            Stage('zero gap', 'gap', (), ZERO_GAP_SEC),
        ]
        for wheel in range(len(WHEEL_INDICES)):
            stages.append(
                Stage(f'single wheel {wheel}', 'single', (wheel,), self.duration)
            )
            stages.append(Stage('zero gap', 'gap', (), ZERO_GAP_SEC))
        stages.append(Stage('all wheels', 'all', (0, 1, 2, 3), self.duration))
        return stages

    def _state_callback(self, msg: RobotState):
        now = time.monotonic()
        self.last_state = msg
        self.last_state_at = now

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

    def _timer_callback(self):
        now = time.monotonic()

        if self.stop_until is not None:
            self._publish_zero()
            if now >= self.stop_until:
                self.done = True
                rclpy.shutdown()
            return

        if not self._state_is_ready(now):
            self._publish_zero()
            return

        if self.current_stage is None:
            self._start_next_stage(now)
            if self.stop_until is not None:
                self._publish_zero()
                return

        if self.current_stage is None:
            self._begin_shutdown_zeros(now)
            self._publish_zero()
            return

        target_indices = self._target_indices(self.current_stage)
        if self.current_stage.kind in ('single', 'all'):
            self._publish_command(target_indices)
        else:
            self._publish_zero()

        self._check_feedback(target_indices)
        if self.stop_until is not None:
            return

        self._print_feedback(now, target_indices)

        if self.stage_started_at is not None:
            elapsed = now - self.stage_started_at
            if elapsed >= self.current_stage.duration:
                self._finish_current_stage(now)

    def _state_is_ready(self, now: float) -> bool:
        if self.active_disabled:
            return False

        if self.last_state is None or self.last_state_at is None:
            if now - self.start_time >= self.state_timeout and not self.waiting_error_logged:
                self.waiting_error_logged = True
                self.active_disabled = True
                self.get_logger().error(
                    f'no RobotState received on {STATE_TOPIC} after '
                    f'{self.state_timeout:.2f}s; holding zero command and '
                    'not entering active tests'
                )
            return False

        if now - self.last_state_at > self.state_timeout:
            if not self.state_stale_logged:
                self.state_stale_logged = True
                self._fail_and_stop(
                    'state timeout: no fresh RobotState for %.2fs' % (now - self.last_state_at),
                    now,
                )
            return False
        self.state_stale_logged = False

        if len(self.last_state.motor_state) != MOTOR_COUNT:
            if not self.invalid_state_logged:
                self.invalid_state_logged = True
                self.get_logger().error(
                    f'latest RobotState size is {len(self.last_state.motor_state)}, '
                    f'expected {MOTOR_COUNT}; holding zero command'
                )
            return False

        for index, motor_state in enumerate(self.last_state.motor_state):
            values = (
                motor_state.q,
                motor_state.dq,
                motor_state.ddq,
                motor_state.tau_est,
                motor_state.cur,
            )
            if not all(math.isfinite(float(value)) for value in values):
                self._fail_and_stop(
                    'invalid non-finite state at motor index %d' % index,
                    now,
                )
                return False

        return True

    def _start_next_stage(self, now: float):
        self.current_stage_index += 1
        if self.current_stage_index >= len(self.stages):
            self.current_stage = None
            self._begin_shutdown_zeros(now)
            return

        self.current_stage = self.stages[self.current_stage_index]
        self.stage_started_at = now
        self.stage_target_peaks = {
            index: 0.0 for index in self._target_indices(self.current_stage)
        }
        self.stage_unexpected_counts = {}
        self.last_print_at = 0.0

        target_indices = self._target_indices(self.current_stage)
        if self.current_stage.kind == 'passive':
            self.get_logger().debug(
                f'stage {self.current_stage_index + 1}/{len(self.stages)}: '
                f'{self.current_stage.name} for {self.current_stage.duration:.2f}s; '
                'publishing zero commands. Manually rotate wheels and confirm '
                f'feedback indexes {list(WHEEL_INDICES)}.'
            )
        elif target_indices:
            self.get_logger().debug(
                f'stage {self.current_stage_index + 1}/{len(self.stages)}: '
                f'{self.current_stage.name} for {self.current_stage.duration:.2f}s; '
                f'target ros indexes={target_indices} dq={self.speed:.3f} '
                f'kd={self.kd:.3f}'
            )
        else:
            self.get_logger().debug(
                f'stage {self.current_stage_index + 1}/{len(self.stages)}: '
                f'{self.current_stage.name} for {self.current_stage.duration:.2f}s; '
                'publishing 16-way zero command'
            )

    def _finish_current_stage(self, now: float):
        if self.current_stage is None:
            return

        if self.current_stage.kind in ('single', 'all'):
            missing = [
                index for index, peak in self.stage_target_peaks.items()
                if peak < self.dq_threshold
            ]
            if missing:
                self._fail_and_stop(
                    'FAIL expected motion missing in %s: target indexes %s '
                    'did not exceed %.3f rad/s'
                    % (self.current_stage.name, missing, self.dq_threshold),
                    now,
                )
                return

        self.get_logger().debug(f'stage complete: {self.current_stage.name}')
        self.current_stage = None
        self.stage_started_at = None
        self._start_next_stage(now)

    def _target_indices(self, stage: Stage) -> tuple[int, ...]:
        return tuple(WHEEL_INDICES[wheel] for wheel in stage.target_wheels)

    def _make_command(self, target_indices: Iterable[int]) -> RobotCommand:
        target_set = set(target_indices)
        msg = RobotCommand()

        for index in range(MOTOR_COUNT):
            cmd = MotorCommand()
            cmd.q = 0.0
            cmd.tau = 0.0
            cmd.kp = 0.0
            if index in target_set:
                cmd.dq = float(self.speed)
                cmd.kd = float(self.kd)
            else:
                cmd.dq = 0.0
                cmd.kd = 0.0
            msg.motor_command.append(cmd)

        return msg

    def _publish_command(self, target_indices: Iterable[int]):
        self.command_pub.publish(self._make_command(target_indices))

    def _publish_zero(self):
        self.command_pub.publish(self._make_command(()))

    def publish_zero_for(self, duration: float):
        end_time = time.monotonic() + duration
        period = 1.0 / self.rate
        while rclpy.ok() and time.monotonic() < end_time:
            self._publish_zero()
            time.sleep(period)

    def _check_feedback(self, target_indices: tuple[int, ...]):
        if self.current_stage is None or self.last_state is None:
            return

        target_set = set(target_indices)
        elapsed = (
            time.monotonic() - self.stage_started_at
            if self.stage_started_at is not None
            else 0.0
        )
        for index, motor_state in enumerate(self.last_state.motor_state):
            abs_dq = abs(float(motor_state.dq))
            if index in target_set:
                self.stage_target_peaks[index] = max(
                    self.stage_target_peaks.get(index, 0.0),
                    abs_dq,
                )
                continue

            if (
                self.current_stage.kind in ('zero', 'single', 'all')
                and abs_dq > self.dq_threshold
            ):
                if elapsed < self.unexpected_grace:
                    self.stage_unexpected_counts[index] = 0
                    continue

                count = self.stage_unexpected_counts.get(index, 0) + 1
                self.stage_unexpected_counts[index] = count
                if count >= self.unexpected_consecutive:
                    self._fail_and_stop(
                        'FAIL unexpected sustained motion in %s: motor index %d '
                        'dq=%.4f exceeds %.4f for %d consecutive samples'
                        % (
                            self.current_stage.name,
                            index,
                            float(motor_state.dq),
                            self.dq_threshold,
                            count,
                        ),
                        time.monotonic(),
                    )
                    return
            else:
                self.stage_unexpected_counts[index] = 0

    def _print_feedback(self, now: float, target_indices: tuple[int, ...]):
        if self.last_state is None or now - self.last_print_at < PRINT_INTERVAL_SEC:
            return
        self.last_print_at = now

        wheel_parts = []
        for wheel, ros_index in enumerate(WHEEL_INDICES):
            dq = float(self.last_state.motor_state[ros_index].dq)
            wheel_parts.append(f'w{wheel}@{ros_index} dq={dq:+.3f}')

        target_set = set(target_indices)
        non_target_max_index = None
        non_target_max_dq = 0.0
        for index, motor_state in enumerate(self.last_state.motor_state):
            if index in target_set:
                continue
            abs_dq = abs(float(motor_state.dq))
            if abs_dq >= abs(non_target_max_dq):
                non_target_max_dq = float(motor_state.dq)
                non_target_max_index = index

        stage_name = self.current_stage.name if self.current_stage else 'stopping'
        self.get_logger().debug(
            f'{stage_name} | {" ".join(wheel_parts)} | '
            f'max non-target idx={non_target_max_index} dq={non_target_max_dq:+.3f}'
        )

    def _fail_and_stop(self, reason: str, now: float):
        if self.stop_until is not None:
            return
        self.failed = True
        self.get_logger().error(reason)
        self._begin_shutdown_zeros(now)

    def _begin_shutdown_zeros(self, now: float):
        if self.stop_until is None:
            self.stop_until = now + SHUTDOWN_ZERO_SEC
            self._publish_zero()
            self.get_logger().debug(
                f'publishing zero command for {SHUTDOWN_ZERO_SEC:.2f}s before shutdown'
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WheelLinkTester()
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
            if rclpy.ok() and node.stop_until is None:
                node.publish_zero_for(SHUTDOWN_ZERO_SEC)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 1 if node is not None and node.failed else 0


if __name__ == '__main__':
    sys.exit(main())
