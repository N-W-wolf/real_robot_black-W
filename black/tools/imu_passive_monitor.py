#!/usr/bin/env python3
import json
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Any, Deque, Dict, Optional

import rclpy
from rcl_interfaces.msg import Log
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

try:
    from robot_msgs.msg import RobotState
except ImportError:  # pragma: no cover - depends on sourced workspace
    RobotState = None


class PassiveImuMonitor(Node):
    def __init__(self) -> None:
        super().__init__("imu_passive_monitor")

        default_log_dir = str(Path(__file__).resolve().parent)
        self.warn_gap_sec = float(self.declare_parameter("warn_gap_sec", 0.25).value)
        self.state_warn_gap_sec = float(self.declare_parameter("state_warn_gap_sec", 0.25).value)
        self.event_window_sec = float(self.declare_parameter("event_window_sec", 3.0).value)
        self.log_dir = Path(self.declare_parameter("log_dir", default_log_dir).value)
        self.log_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = self.log_dir / f"imu_passive_monitor_{timestamp}.log"
        self.jsonl_path = self.log_dir / f"imu_passive_monitor_{timestamp}.jsonl"
        self.log_file = self.log_path.open("a", encoding="utf-8", buffering=1)
        self.jsonl_file = self.jsonl_path.open("a", encoding="utf-8", buffering=1)

        sensor_qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.raw_count = 0
        self.filtered_count = 0
        self.joint_count = 0
        self.last_raw_wall = 0.0
        self.last_filtered_wall = 0.0
        self.last_joint_wall = 0.0
        self.last_raw_msg_stamp_ns = 0
        self.last_filtered_msg_stamp_ns = 0

        self.last_report_wall = time.monotonic()
        self.last_report_raw_count = 0
        self.last_report_filtered_count = 0
        self.last_report_joint_count = 0

        self.raw_rate_hz = 0.0
        self.filtered_rate_hz = 0.0
        self.joint_rate_hz = 0.0

        self.raw_publishers = 0
        self.raw_subscribers = 0
        self.filtered_publishers = 0
        self.filtered_subscribers = 0
        self.joint_publishers = 0
        self.joint_subscribers = 0

        self.recent_events: Deque[Dict[str, Any]] = deque(maxlen=64)

        self.create_subscription(Imu, "/_lowState/imu_raw", self.on_raw_imu, sensor_qos)
        self.create_subscription(Imu, "/_lowState/imu", self.on_filtered_imu, sensor_qos)
        if RobotState is not None:
            self.create_subscription(RobotState, "/_lowState/joint", self.on_joint_state, 10)
        self.create_subscription(Log, "/rosout", self.on_rosout, 200)
        self.create_timer(1.0, self.report)

        self._log_text(f"log_file={self.log_path}")
        self._log_text(f"jsonl_file={self.jsonl_path}")
        self._log_text(
            f"robot_state_subscription={'enabled' if RobotState is not None else 'disabled'}"
        )

    def destroy_node(self) -> bool:
        try:
            self.log_file.close()
            self.jsonl_file.close()
        finally:
            return super().destroy_node()

    def on_raw_imu(self, msg: Imu) -> None:
        self.raw_count += 1
        self.last_raw_wall = time.monotonic()
        self.last_raw_msg_stamp_ns = self._stamp_to_ns(msg.header.stamp)

    def on_filtered_imu(self, msg: Imu) -> None:
        self.filtered_count += 1
        self.last_filtered_wall = time.monotonic()
        self.last_filtered_msg_stamp_ns = self._stamp_to_ns(msg.header.stamp)

    def on_joint_state(self, _: Any) -> None:
        self.joint_count += 1
        self.last_joint_wall = time.monotonic()

    def on_rosout(self, msg: Log) -> None:
        if not self._is_relevant_rosout(msg):
            return
        event = {
            "wall_time": datetime.now().isoformat(timespec="seconds"),
            "node": msg.name,
            "level": int(msg.level),
            "message": msg.msg,
            "file": msg.file,
            "function": msg.function,
            "line": int(msg.line),
            "wall_monotonic": time.monotonic(),
        }
        self.recent_events.append(event)
        self._log_text(
            f"rosout node={msg.name} level={int(msg.level)} msg={msg.msg}"
        )

    @staticmethod
    def _stamp_to_ns(stamp: Any) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @staticmethod
    def _level_name(level: int) -> str:
        return {
            10: "DEBUG",
            20: "INFO",
            30: "WARN",
            40: "ERROR",
            50: "FATAL",
        }.get(level, str(level))

    @staticmethod
    def _is_relevant_rosout(msg: Log) -> bool:
        name = (msg.name or "").lower()
        text = msg.msg or ""
        if name == "imu_passive_monitor":
            return False
        if "realrunner" in name or "real_runner" in name:
            return True
        return (
            "IMU 数据超时" in text
            or "IMU 节点已掉线或未启动" in text
            or "IMU 节点已上线" in text
            or "Control loop overtime" in text
        )

    def _age(self, wall_time: float) -> float:
        if wall_time <= 0.0:
            return float("inf")
        return time.monotonic() - wall_time

    def _refresh_graph(self) -> None:
        self.raw_publishers = self.count_publishers("/_lowState/imu_raw")
        self.raw_subscribers = self.count_subscribers("/_lowState/imu_raw")
        self.filtered_publishers = self.count_publishers("/_lowState/imu")
        self.filtered_subscribers = self.count_subscribers("/_lowState/imu")
        if RobotState is not None:
            self.joint_publishers = self.count_publishers("/_lowState/joint")
            self.joint_subscribers = self.count_subscribers("/_lowState/joint")
        else:
            self.joint_publishers = 0
            self.joint_subscribers = 0

    def _update_rates(self) -> None:
        now = time.monotonic()
        dt = now - self.last_report_wall
        if dt <= 0.0:
            return
        self.raw_rate_hz = (self.raw_count - self.last_report_raw_count) / dt
        self.filtered_rate_hz = (self.filtered_count - self.last_report_filtered_count) / dt
        self.joint_rate_hz = (self.joint_count - self.last_report_joint_count) / dt
        self.last_report_wall = now
        self.last_report_raw_count = self.raw_count
        self.last_report_filtered_count = self.filtered_count
        self.last_report_joint_count = self.joint_count

    def _latest_event(self) -> Optional[Dict[str, Any]]:
        if not self.recent_events:
            return None
        latest = self.recent_events[-1]
        if time.monotonic() - float(latest["wall_monotonic"]) > self.event_window_sec:
            return None
        return latest

    def _has_recent_timeout_event(self) -> bool:
        for event in reversed(self.recent_events):
            if time.monotonic() - float(event["wall_monotonic"]) > self.event_window_sec:
                break
            if "IMU 数据超时" in str(event["message"]):
                return True
        return False

    def infer_stage(self) -> str:
        raw_age = self._age(self.last_raw_wall)
        filtered_age = self._age(self.last_filtered_wall)
        joint_age = self._age(self.last_joint_wall)
        has_recent_timeout = self._has_recent_timeout_event()

        if self.raw_publishers == 0 and self.raw_count == 0:
            return "no_imu_raw_publisher_detected"
        if raw_age > self.warn_gap_sec:
            return "imu_raw_topic_stale"

        if self.filtered_publishers == 0 and self.filtered_count == 0:
            return "no_filtered_imu_publisher_detected"
        if has_recent_timeout and filtered_age <= self.warn_gap_sec:
            return "real_runner_reported_timeout_while_filtered_topic_recent"
        if has_recent_timeout and filtered_age > self.warn_gap_sec:
            return "real_runner_reported_timeout_after_filtered_topic_stall"
        if filtered_age > self.warn_gap_sec:
            return "imu_topic_stale_after_filter"

        if self.joint_publishers > 0 and joint_age > self.state_warn_gap_sec:
            return "real_runner_state_topic_stale"

        return "pipeline_ok"

    def _log_text(self, line: str) -> None:
        stamped = f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] {line}"
        self.get_logger().info(line)
        self.log_file.write(stamped + "\n")
        self.log_file.flush()

    def report(self) -> None:
        self._refresh_graph()
        self._update_rates()

        raw_age = self._age(self.last_raw_wall)
        filtered_age = self._age(self.last_filtered_wall)
        joint_age = self._age(self.last_joint_wall)
        stage = self.infer_stage()
        latest_event = self._latest_event()

        stage_line = (
            f"stage={stage} raw_count={self.raw_count} raw_rate={self.raw_rate_hz:.1f}Hz "
            f"raw_age={raw_age:.3f}s filtered_count={self.filtered_count} "
            f"filtered_rate={self.filtered_rate_hz:.1f}Hz filtered_age={filtered_age:.3f}s "
            f"joint_count={self.joint_count} joint_rate={self.joint_rate_hz:.1f}Hz "
            f"joint_age={joint_age:.3f}s raw_pub={self.raw_publishers} raw_sub={self.raw_subscribers} "
            f"filtered_pub={self.filtered_publishers} filtered_sub={self.filtered_subscribers} "
            f"joint_pub={self.joint_publishers} joint_sub={self.joint_subscribers}"
        )
        self._log_text(stage_line)

        if latest_event is not None:
            self._log_text(
                "latest_real_runner_event "
                f"level={self._level_name(int(latest_event['level']))} "
                f"node={latest_event['node']} msg={latest_event['message']}"
            )

        snapshot = {
            "wall_time": datetime.now().isoformat(timespec="seconds"),
            "stage": stage,
            "raw_count": self.raw_count,
            "filtered_count": self.filtered_count,
            "joint_count": self.joint_count,
            "raw_rate_hz": self.raw_rate_hz,
            "filtered_rate_hz": self.filtered_rate_hz,
            "joint_rate_hz": self.joint_rate_hz,
            "raw_age_sec": raw_age,
            "filtered_age_sec": filtered_age,
            "joint_age_sec": joint_age,
            "raw_publishers": self.raw_publishers,
            "raw_subscribers": self.raw_subscribers,
            "filtered_publishers": self.filtered_publishers,
            "filtered_subscribers": self.filtered_subscribers,
            "joint_publishers": self.joint_publishers,
            "joint_subscribers": self.joint_subscribers,
            "last_raw_msg_stamp_ns": self.last_raw_msg_stamp_ns,
            "last_filtered_msg_stamp_ns": self.last_filtered_msg_stamp_ns,
            "latest_real_runner_event": None
            if latest_event is None
            else {
                "wall_time": latest_event["wall_time"],
                "node": latest_event["node"],
                "level": latest_event["level"],
                "message": latest_event["message"],
                "file": latest_event["file"],
                "function": latest_event["function"],
                "line": latest_event["line"],
            },
        }
        self.jsonl_file.write(json.dumps(snapshot, ensure_ascii=False) + "\n")
        self.jsonl_file.flush()


def main() -> None:
    rclpy.init()
    node = PassiveImuMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
