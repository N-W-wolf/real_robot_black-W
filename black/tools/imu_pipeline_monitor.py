#!/usr/bin/env python3
import json
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String


class ImuPipelineMonitor(Node):
    def __init__(self) -> None:
        super().__init__("imu_pipeline_monitor")

        default_log_dir = str(Path(__file__).resolve().parent)
        self.warn_gap_sec = float(self.declare_parameter("warn_gap_sec", 0.25).value)
        self.diag_stale_sec = float(self.declare_parameter("diag_stale_sec", 2.0).value)
        self.log_dir = Path(self.declare_parameter("log_dir", default_log_dir).value)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = self.log_dir / f"imu_pipeline_monitor_{timestamp}.log"
        self.jsonl_path = self.log_dir / f"imu_pipeline_monitor_{timestamp}.jsonl"
        self.log_file = self.log_path.open("a", encoding="utf-8", buffering=1)
        self.jsonl_file = self.jsonl_path.open("a", encoding="utf-8", buffering=1)

        sensor_qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.raw_count = 0
        self.filtered_count = 0
        self.last_raw_wall = 0.0
        self.last_filtered_wall = 0.0
        self.source_diag: Optional[Dict[str, Any]] = None
        self.filter_diag: Optional[Dict[str, Any]] = None
        self.receiver_diag: Optional[Dict[str, Any]] = None
        self.source_diag_wall = 0.0
        self.filter_diag_wall = 0.0
        self.receiver_diag_wall = 0.0

        self.create_subscription(Imu, "/_lowState/imu_raw", self.on_raw_imu, sensor_qos)
        self.create_subscription(Imu, "/_lowState/imu", self.on_filtered_imu, sensor_qos)
        self.create_subscription(String, "/debug/imu_source_diag", self.on_source_diag, 10)
        self.create_subscription(String, "/debug/imu_filter_diag", self.on_filter_diag, 10)
        self.create_subscription(String, "/debug/imu_receiver_diag", self.on_receiver_diag, 10)
        self.create_timer(1.0, self.report)

        self._log_text(f"log_file={self.log_path}")
        self._log_text(f"jsonl_file={self.jsonl_path}")

    def destroy_node(self) -> bool:
        try:
            self.log_file.close()
            self.jsonl_file.close()
        finally:
            return super().destroy_node()

    def on_raw_imu(self, _: Imu) -> None:
        self.raw_count += 1
        self.last_raw_wall = time.monotonic()

    def on_filtered_imu(self, _: Imu) -> None:
        self.filtered_count += 1
        self.last_filtered_wall = time.monotonic()

    def on_source_diag(self, msg: String) -> None:
        self.source_diag = self._parse_diag(msg.data)
        self.source_diag_wall = time.monotonic()

    def on_filter_diag(self, msg: String) -> None:
        self.filter_diag = self._parse_diag(msg.data)
        self.filter_diag_wall = time.monotonic()

    def on_receiver_diag(self, msg: String) -> None:
        self.receiver_diag = self._parse_diag(msg.data)
        self.receiver_diag_wall = time.monotonic()

    def _parse_diag(self, payload: str) -> Optional[Dict[str, Any]]:
        try:
            return json.loads(payload)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"failed to parse diag payload: {exc}: {payload}")
            self._log_text(f"parse_error={exc}: {payload}")
            return None

    def _age(self, wall_time: float) -> float:
        if wall_time <= 0.0:
            return float("inf")
        return time.monotonic() - wall_time

    def _ns_age(self, diag: Optional[Dict[str, Any]], key: str) -> float:
        if not diag:
            return float("inf")
        ns = int(diag.get(key, 0) or 0)
        if ns <= 0:
            return float("inf")
        now_ns = int(diag.get("now_ns", 0) or 0)
        if now_ns <= 0:
            return float("inf")
        return max(0.0, (now_ns - ns) * 1e-9)

    def _log_text(self, line: str) -> None:
        stamped = f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] {line}"
        self.get_logger().info(line)
        self.log_file.write(stamped + "\n")
        self.log_file.flush()

    def infer_stage(self) -> str:
        source_age = self._age(self.source_diag_wall)
        filter_age = self._age(self.filter_diag_wall)
        receiver_age = self._age(self.receiver_diag_wall)
        raw_age = self._age(self.last_raw_wall)
        filtered_age = self._age(self.last_filtered_wall)

        if source_age > self.diag_stale_sec:
            return "source_diag_missing_or_source_node_stalled"

        if self.source_diag:
            connection_type = int(self.source_diag.get("connection_type", -1))
            serial_open = bool(self.source_diag.get("serial_open", False))
            udp_bound = bool(self.source_diag.get("udp_bound", False))
            read_age = self._ns_age(self.source_diag, "last_read_ns")
            parsed_age = self._ns_age(self.source_diag, "last_parsed_ns")
            publish_age = self._ns_age(self.source_diag, "last_publish_ns")

            if connection_type == 0 and not serial_open:
                return "imu_serial_not_open"
            if connection_type == 1 and not udp_bound:
                return "imu_udp_not_bound"
            if read_age > self.warn_gap_sec:
                return "no_hardware_bytes_read_recently"
            if parsed_age > self.warn_gap_sec and read_age <= self.warn_gap_sec:
                return "bytes_arrived_but_no_imu_frame_parsed"
            if publish_age > self.warn_gap_sec and parsed_age <= self.warn_gap_sec:
                return "imu_frame_parsed_but_imu_raw_not_published"

        if raw_age > self.warn_gap_sec:
            return "imu_raw_topic_stale_after_source"

        if filter_age > self.diag_stale_sec:
            return "imu_filter_diag_missing_or_filter_node_stalled"

        if self.filter_diag:
            input_age = self._ns_age(self.filter_diag, "last_input_arrival_ns")
            output_age = self._ns_age(self.filter_diag, "last_output_publish_ns")
            if input_age <= self.warn_gap_sec and output_age > self.warn_gap_sec:
                return "imu_filter_receives_input_but_no_output"

        if filtered_age > self.warn_gap_sec:
            return "imu_topic_stale_after_filter"

        if receiver_age > self.diag_stale_sec:
            return "real_runner_diag_missing_or_receiver_stalled"

        if self.receiver_diag:
            gap = float(self.receiver_diag.get("current_gap_sec", -1.0))
            imu_alive = bool(self.receiver_diag.get("imu_alive", True))
            if gap > self.warn_gap_sec or not imu_alive:
                return "real_runner_receiver_or_executor_delayed"

        return "pipeline_ok"

    def report(self) -> None:
        source_age = self._age(self.source_diag_wall)
        filter_age = self._age(self.filter_diag_wall)
        receiver_age = self._age(self.receiver_diag_wall)
        raw_age = self._age(self.last_raw_wall)
        filtered_age = self._age(self.last_filtered_wall)
        inferred = self.infer_stage()

        stage_line = (
            f"stage={inferred} raw_count={self.raw_count} raw_age={raw_age:.3f}s "
            f"filtered_count={self.filtered_count} filtered_age={filtered_age:.3f}s "
            f"source_diag_age={source_age:.3f}s filter_diag_age={filter_age:.3f}s "
            f"receiver_diag_age={receiver_age:.3f}s"
        )
        self._log_text(stage_line)

        if self.source_diag:
            self._log_text(
                "source "
                f"read_events={self.source_diag.get('read_event_count')} "
                f"parsed={self.source_diag.get('parsed_imu_count')} "
                f"published={self.source_diag.get('published_imu_raw_count')} "
                f"read_age={self._ns_age(self.source_diag, 'last_read_ns'):.3f}s "
                f"parsed_age={self._ns_age(self.source_diag, 'last_parsed_ns'):.3f}s "
                f"publish_age={self._ns_age(self.source_diag, 'last_publish_ns'):.3f}s"
            )

        if self.filter_diag:
            self._log_text(
                "filter "
                f"input={self.filter_diag.get('input_count')} "
                f"output={self.filter_diag.get('output_count')} "
                f"bad={self.filter_diag.get('bad_sample_count')} "
                f"input_age={self._ns_age(self.filter_diag, 'last_input_arrival_ns'):.3f}s "
                f"output_age={self._ns_age(self.filter_diag, 'last_output_publish_ns'):.3f}s"
            )

        if self.receiver_diag:
            self._log_text(
                "receiver "
                f"count={self.receiver_diag.get('received_imu_count')} "
                f"gap={float(self.receiver_diag.get('current_gap_sec', -1.0)):.3f}s "
                f"strikes={self.receiver_diag.get('timeout_strikes')} "
                f"alive={self.receiver_diag.get('imu_alive')}"
            )

        snapshot = {
            "wall_time": datetime.now().isoformat(timespec="seconds"),
            "stage": inferred,
            "raw_count": self.raw_count,
            "raw_age_sec": raw_age,
            "filtered_count": self.filtered_count,
            "filtered_age_sec": filtered_age,
            "source_diag_age_sec": source_age,
            "filter_diag_age_sec": filter_age,
            "receiver_diag_age_sec": receiver_age,
            "source_diag": self.source_diag,
            "filter_diag": self.filter_diag,
            "receiver_diag": self.receiver_diag,
        }
        self.jsonl_file.write(json.dumps(snapshot, ensure_ascii=False) + "\n")
        self.jsonl_file.flush()


def main() -> None:
    rclpy.init()
    node = ImuPipelineMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
