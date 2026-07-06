import os
import re
import threading
import time
from collections import deque

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from robot_msgs.msg import Gap, GapModel, MotorState, RobotCommand, RobotState
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger

from . import config


class MujocoNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        self.declare_parameter("rname", config.DEFAULT_ROBOT_NAME)
        self.declare_parameter("scene", config.SCENE_DEFAULT)
        self.declare_parameter("publish_odom", False)
        self.declare_parameter("publish_rate_hz", 500.0)
        self.declare_parameter("render", True)
        self.declare_parameter("render_rate_hz", config.DEFAULT_RENDER_RATE_HZ)
        self.declare_parameter("real_time", True)
        self.declare_parameter("publish_gap_model", True)
        self.declare_parameter("gap_model_topic", "/gap_model")
        self.declare_parameter("gap_model_confidence", 1.0)
        self.declare_parameter("gap_missing_marker", -9999.0)
        self.declare_parameter("enable_additional_noise", config.ENABLE_ADDITIONAL_NOISE)

        requested_robot_name = str(self.get_parameter("rname").value)
        scene = str(self.get_parameter("scene").value)
        self.publish_odom = bool(self.get_parameter("publish_odom").value)
        self.enable_render = bool(self.get_parameter("render").value)
        render_rate_hz = float(self.get_parameter("render_rate_hz").value)
        self.real_time = bool(self.get_parameter("real_time").value)
        self.publish_gap_model = bool(self.get_parameter("publish_gap_model").value)
        gap_model_topic = str(self.get_parameter("gap_model_topic").value)
        gap_model_confidence = float(self.get_parameter("gap_model_confidence").value)
        self.enable_additional_noise = bool(self.get_parameter("enable_additional_noise").value)
        self.gap_model_confidence = float(np.clip(gap_model_confidence, 0.0, 1.0))
        if gap_model_confidence != self.gap_model_confidence:
            self.get_logger().warn(
                f"Parameter 'gap_model_confidence'={gap_model_confidence} is out of [0,1]. "
                f"Clamped to {self.gap_model_confidence}."
            )

        self.gap_missing_marker = float(self.get_parameter("gap_missing_marker").value)

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if publish_rate_hz <= 0.0:
            self.get_logger().warn("Parameter 'publish_rate_hz' must be > 0. Fallback to 500 Hz.")
            publish_rate_hz = 500.0
        self.publish_period = 1.0 / publish_rate_hz

        if render_rate_hz <= 0.0:
            self.get_logger().warn(
                "Parameter 'render_rate_hz' must be > 0. "
                f"Fallback to {config.DEFAULT_RENDER_RATE_HZ} Hz."
            )
            render_rate_hz = config.DEFAULT_RENDER_RATE_HZ
        self.render_period = 1.0 / render_rate_hz

        self.requested_robot_name = requested_robot_name
        self.robot_name = config.resolve_robot_name(self.requested_robot_name)
        self.robot_profile = config.get_robot_profile(self.requested_robot_name)
        self.route_name = str(self.robot_profile.get("route_name", "legacy_legged"))
        self.actuator_num = int(self.robot_profile.get("actuator_num", config.ACTUATOR_NUM))
        self.motor_map = np.asarray(self.robot_profile.get("motor_mapping", config.MOTOR_MAPPING), dtype=np.int32)
        self.joint_pos_sensor_names = tuple(self.robot_profile.get("joint_pos_sensor_names", []))
        self.joint_vel_sensor_names = tuple(self.robot_profile.get("joint_vel_sensor_names", []))
        self.default_ctrl = np.asarray(self.robot_profile.get("default_pos", []), dtype=float)
        self.imu_attitude_lpf_alpha = float(
            np.clip(float(self.robot_profile.get("imu_attitude_lpf_alpha", 1.0)), 0.0, 1.0)
        )
        self.filtered_imu_quat = None
        self._validate_route_profile()

        self.get_logger().info(f"loading robot is :{self.robot_name}")
        if self.robot_name != self.requested_robot_name:
            self.get_logger().info(
                f"robot name alias: requested '{self.requested_robot_name}' -> resolved '{self.robot_name}'"
            )
        self.get_logger().info(f"control route is :{self.route_name}")
        self.get_logger().info(f"Additional noise is {'ENABLED' if self.enable_additional_noise else 'DISABLED'}.")
        if self.imu_attitude_lpf_alpha < 1.0:
            self.get_logger().info(f"IMU attitude low-pass alpha is {self.imu_attitude_lpf_alpha:.3f}.")

        robot_dir = os.path.join(get_package_share_directory("robot_description"), self.robot_name)
        path = self.resolve_scene_path(robot_dir, scene)
        self.get_logger().info(f"using scene is :{os.path.basename(path)}")

        self.model = mujoco.MjModel.from_xml_path(path)
        self.data = mujoco.MjData(self.model)
        self._align_route_with_model()
        self._set_initial_state()
        self.lock = threading.RLock()

        # Apply robot-specific timestep before building delay buffers.
        self.model.opt.timestep = config.get_sim_dt(self.requested_robot_name)

        # Sensor cache uses model's actual sensor layout instead of hard-coded indices.
        self.values = np.array(self.data.sensordata, copy=True)
        self._initialize_sensor_layout()
        self._initialize_backlash_model()

        self.trunk_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "trunk")
        if self.trunk_body_id < 0:
            self.get_logger().warn("Body 'trunk' not found in model. Ground-truth odom will use sensor fallback.")

        self.bridge_gaps_world = self._extract_bridge_gaps_world()

        self._initialize_delay_buffers()

        if self.enable_additional_noise:
            self.encoder_position_bias = np.random.uniform(-config.MOTOR_BIAS, config.MOTOR_BIAS, self.actuator_num)
            self.encoder_velocity_bias = np.random.uniform(-config.MOTOR_BIAS, config.MOTOR_BIAS, self.actuator_num)
        else:
            self.encoder_position_bias = np.zeros(self.actuator_num, dtype=float)
            self.encoder_velocity_bias = np.zeros(self.actuator_num, dtype=float)

        self.viewer = None
        if self.enable_render:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            if self.route_name == "wheel_leg_blackw":
                # Keep collision geoms active for physics but hide their render group.
                self.viewer.opt.geomgroup[3] = 0
                self.get_logger().info("blackW route: collision geoms (group 3) are hidden in renderer.")
            self.get_logger().info(f"Rendering is ENABLED at {render_rate_hz:.1f} Hz.")

        self.imu_publisher = self.create_publisher(Imu, "/_lowState/imu", 10)
        self.joint_publisher = self.create_publisher(RobotState, "/_lowState/joint", 10)
        self.odom_publisher = None
        if self.publish_odom:
            self.odom_publisher = self.create_publisher(Odometry, "/odom", 10)
            self.get_logger().info("Ground-truth odom publishing is ENABLED on '/odom'.")
        else:
            self.get_logger().info("Ground-truth odom publishing is DISABLED.")

        self.gap_model_publisher = None
        if self.publish_gap_model:
            self.gap_model_publisher = self.create_publisher(GapModel, gap_model_topic, 10)
            if self.bridge_gaps_world:
                self.get_logger().info(
                    f"GapModel publishing is ENABLED on '{gap_model_topic}' in fixed-two-gap mode "
                    f"(front+rear). source_gaps={len(self.bridge_gaps_world)}, "
                    f"missing_marker={self.gap_missing_marker}."
                )
            else:
                self.get_logger().warn(
                    f"GapModel publishing is ENABLED on '{gap_model_topic}', but no geoms named "
                    f"'bridge_<index>' were found. Both slots will publish marker {self.gap_missing_marker}."
                )
        else:
            self.get_logger().info("GapModel publishing is DISABLED.")

        self.joint_subscriber = self.create_subscription(
            RobotCommand, "/_lowCmd/command", self.joint_cmd_callback, 10
        )
        self.reset_default_pose_service = self.create_service(
            Trigger, "/mujoco_runner/reset_default_pose", self.reset_default_pose_callback
        )
        self.timer = self.create_timer(self.publish_period, self.publish_data)

        self.simulation_thread = threading.Thread(target=self.run_simulation, daemon=True)
        self.simulation_thread.start()

        if self.viewer is not None:
            self.render_thread = threading.Thread(target=self.run_render, daemon=True)
            self.render_thread.start()

    def _validate_route_profile(self):
        if self.actuator_num <= 0:
            self.get_logger().warn(
                f"Invalid actuator_num={self.actuator_num} in route profile '{self.route_name}'. "
                "Fallback to legacy profile."
            )
            fallback = config.ROBOT_PROFILES["default"]
            self.actuator_num = int(fallback["actuator_num"])
            self.motor_map = np.asarray(fallback["motor_mapping"], dtype=np.int32)
            self.joint_pos_sensor_names = tuple(fallback["joint_pos_sensor_names"])
            self.joint_vel_sensor_names = tuple(fallback["joint_vel_sensor_names"])
            self.default_ctrl = np.asarray(fallback["default_pos"], dtype=float)

        if self.motor_map.size != self.actuator_num:
            self.get_logger().warn(
                f"Route '{self.route_name}' motor_mapping size={self.motor_map.size} "
                f"!= actuator_num={self.actuator_num}. Fallback to identity mapping."
            )
            self.motor_map = np.arange(self.actuator_num, dtype=np.int32)

        invalid_mapping = (
            self.motor_map.size == 0
            or np.any(self.motor_map < 0)
            or np.any(self.motor_map >= self.actuator_num)
            or np.unique(self.motor_map).size != self.actuator_num
        )
        if invalid_mapping:
            self.get_logger().warn(
                f"Route '{self.route_name}' has invalid motor_mapping. Fallback to identity mapping."
            )
            self.motor_map = np.arange(self.actuator_num, dtype=np.int32)

        if len(self.joint_pos_sensor_names) != self.actuator_num:
            self.get_logger().warn(
                f"Route '{self.route_name}' joint_pos_sensor_names count={len(self.joint_pos_sensor_names)} "
                f"!= actuator_num={self.actuator_num}. Missing entries will use synthetic placeholders."
            )
            pos_names = list(self.joint_pos_sensor_names[: self.actuator_num])
            while len(pos_names) < self.actuator_num:
                pos_names.append(f"joint_{len(pos_names)}_pos")
            self.joint_pos_sensor_names = tuple(pos_names)

        if len(self.joint_vel_sensor_names) != self.actuator_num:
            self.get_logger().warn(
                f"Route '{self.route_name}' joint_vel_sensor_names count={len(self.joint_vel_sensor_names)} "
                f"!= actuator_num={self.actuator_num}. Missing entries will use synthetic placeholders."
            )
            vel_names = list(self.joint_vel_sensor_names[: self.actuator_num])
            while len(vel_names) < self.actuator_num:
                vel_names.append(f"joint_{len(vel_names)}_vel")
            self.joint_vel_sensor_names = tuple(vel_names)

        if self.default_ctrl.size < self.actuator_num:
            self.get_logger().warn(
                f"Route '{self.route_name}' default_pos length={self.default_ctrl.size} "
                f"< actuator_num={self.actuator_num}. Remaining controls set to 0."
            )
            padded = np.zeros(self.actuator_num, dtype=float)
            padded[: self.default_ctrl.size] = self.default_ctrl
            self.default_ctrl = padded
        elif self.default_ctrl.size > self.actuator_num:
            self.default_ctrl = self.default_ctrl[: self.actuator_num]

    def _align_route_with_model(self):
        model_actuator_num = int(self.model.nu)
        if model_actuator_num == self.actuator_num:
            return

        self.get_logger().warn(
            f"Model actuator count is {model_actuator_num}, route '{self.route_name}' expects {self.actuator_num}. "
            "Switching to identity mapping with model actuator count."
        )

        self.actuator_num = model_actuator_num
        self.motor_map = np.arange(self.actuator_num, dtype=np.int32)
        self.default_ctrl = np.zeros(self.actuator_num, dtype=float)
        pos_names = list(self.joint_pos_sensor_names[: self.actuator_num])
        vel_names = list(self.joint_vel_sensor_names[: self.actuator_num])
        while len(pos_names) < self.actuator_num:
            pos_names.append(f"joint_{len(pos_names)}_pos")
        while len(vel_names) < self.actuator_num:
            vel_names.append(f"joint_{len(vel_names)}_vel")
        self.joint_pos_sensor_names = tuple(pos_names)
        self.joint_vel_sensor_names = tuple(vel_names)

    def _set_initial_state(self):
        key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "default_pose")
        if key_id >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
        else:
            self.get_logger().warn("Keyframe 'default_pose' not found. Falling back to model default pose.")

        apply_count = min(self.data.ctrl.shape[0], self.default_ctrl.size)
        self.data.ctrl[:apply_count] = self.default_ctrl[:apply_count]

        mujoco.mj_forward(self.model, self.data)

    def reset_default_pose_callback(self, request, response):
        del request
        with self.lock:
            self._set_initial_state()
            self.reset_delay()
        response.success = True
        response.message = "reset to keyframe 'default_pose'"
        self.get_logger().info("Reset to keyframe 'default_pose' via ROS service.")
        return response

    def _resolve_sensor_slice(self, sensor_name: str, expected_dim: int = None, warn_on_missing: bool = True):
        sensor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
        if sensor_id < 0:
            if warn_on_missing:
                self.get_logger().warn(f"Sensor '{sensor_name}' not found in model.")
            return None

        adr = int(self.model.sensor_adr[sensor_id])
        dim = int(self.model.sensor_dim[sensor_id])

        if expected_dim is not None and dim != expected_dim:
            self.get_logger().warn(
                f"Sensor '{sensor_name}' dim={dim}, expected {expected_dim}. Using available dimension."
            )

        return slice(adr, adr + dim)

    def _initialize_sensor_layout(self):
        self.imu_quat_slice = self._resolve_sensor_slice("imu_quat", 4)
        self.imu_gyro_slice = self._resolve_sensor_slice("imu_gyro", 3)
        self.imu_acc_slice = self._resolve_sensor_slice("imu_acc", 3)
        self.frame_pos_slice = self._resolve_sensor_slice("frame_pos", 3)
        self.frame_vel_slice = self._resolve_sensor_slice("frame_vel", 3)

        pos_indices = []
        vel_indices = []

        for name in self.joint_pos_sensor_names:
            sensor_slice = self._resolve_sensor_slice(name, 1, warn_on_missing=False)
            if sensor_slice is None:
                pos_indices = []
                break
            pos_indices.append(sensor_slice.start)

        for name in self.joint_vel_sensor_names:
            sensor_slice = self._resolve_sensor_slice(name, 1, warn_on_missing=False)
            if sensor_slice is None:
                vel_indices = []
                break
            vel_indices.append(sensor_slice.start)

        if len(pos_indices) == self.actuator_num and len(vel_indices) == self.actuator_num:
            self.joint_pos_data_indices = pos_indices
            self.joint_vel_data_indices = vel_indices
            self.get_logger().info("Using name-based joint sensor layout.")
        else:
            self.joint_pos_data_indices = list(range(self.actuator_num))
            self.joint_vel_data_indices = list(range(self.actuator_num, 2 * self.actuator_num))
            self.get_logger().warn(
                "Could not resolve full name-based joint sensors. Falling back to contiguous layout assumption."
            )

        self.joint_pos_data_indices_np = np.asarray(self.joint_pos_data_indices, dtype=np.int32)
        self.joint_vel_data_indices_np = np.asarray(self.joint_vel_data_indices, dtype=np.int32)
        self.logical_joint_pos_indices = self.joint_pos_data_indices_np[self.motor_map]
        self.logical_joint_vel_indices = self.joint_vel_data_indices_np[self.motor_map]

    def _initialize_backlash_model(self):
        self.logical_joint_names = np.asarray(
            [self.joint_pos_sensor_names[int(phy_id)].replace("_pos", "") for phy_id in self.motor_map],
            dtype=object,
        )
        self.calf_physical_ids = np.asarray(
            [idx for idx, name in enumerate(self.joint_pos_sensor_names) if name.endswith("calf_pos")],
            dtype=np.int32,
        )
        self.calf_logical_ids = np.flatnonzero(np.isin(self.motor_map, self.calf_physical_ids)).astype(np.int32)
        self.calf_backlash_by_physical = np.zeros(self.actuator_num, dtype=float)

        if self.calf_physical_ids.size == 0:
            self.calf_backlash_by_logical = np.zeros(self.actuator_num, dtype=float)
            self.get_logger().warn("No calf joints detected for backlash model.")
            return

        if not self.enable_additional_noise:
            self.calf_backlash_by_logical = self.calf_backlash_by_physical[self.motor_map]
            self.get_logger().info("Calf backlash deadband is DISABLED.")
            return

        sampled_backlash_deg = np.random.uniform(
            config.CALF_BACKLASH_MIN_DEG,
            config.CALF_BACKLASH_MAX_DEG,
            self.calf_physical_ids.size,
        )
        self.calf_backlash_by_physical[self.calf_physical_ids] = np.deg2rad(sampled_backlash_deg)
        self.calf_backlash_by_logical = self.calf_backlash_by_physical[self.motor_map]

        sampled_info = ", ".join(
            f"{self.logical_joint_names[int(logical_id)]}:{np.rad2deg(self.calf_backlash_by_logical[int(logical_id)]):.2f}deg"
            for logical_id in self.calf_logical_ids
        )
        self.get_logger().info(f"Applied calf backlash deadband: {sampled_info}")

    def _initialize_delay_buffers(self):
        self.delay_steps_list = []
        self.ctrl_buffer = []
        self.joint_state_buffer = []
        self.delayed_ctrl_cache = np.zeros(self.actuator_num, dtype=float)
        self.delayed_joint_pos_cache = np.zeros(self.actuator_num, dtype=float)
        self.delayed_joint_vel_cache = np.zeros(self.actuator_num, dtype=float)

        min_delay = 0.0
        max_delay = config.MOTOR_DELAY if self.enable_additional_noise else 0.0

        for _ in range(self.actuator_num):
            if max_delay > 0.0:
                delay_time = np.random.uniform(min_delay, max_delay)
                delay_steps = max(1, int(delay_time / self.model.opt.timestep))
            else:
                delay_steps = 1
            self.delay_steps_list.append(delay_steps)
            self.ctrl_buffer.append(deque([0.0] * delay_steps, maxlen=delay_steps))
            self.joint_state_buffer.append(deque([(0.0, 0.0)] * delay_steps, maxlen=delay_steps))

    def _read_sensor_vector(self, sensor_slice, expected_dim: int, default=None):
        if default is None:
            default = np.zeros(expected_dim, dtype=float)

        if sensor_slice is None:
            return np.array(default, dtype=float)

        if len(self.values) < sensor_slice.stop:
            return np.array(default, dtype=float)

        vec = np.array(self.values[sensor_slice], dtype=float)
        if vec.size == expected_dim:
            return vec
        if vec.size > expected_dim:
            return vec[:expected_dim]

        out = np.array(default, dtype=float)
        out[: vec.size] = vec
        return out

    @staticmethod
    def resolve_scene_path(robot_dir: str, scene: str) -> str:
        scene_alias = {
            "flat": config.SCENE_FLAT,
            "plane": config.SCENE_FLAT,
            "obstacle": config.SCENE_OBSTACLE,
            "terrain": config.SCENE_TERRAIN,
            "robocon": config.SCENE_ROBOCON,
        }

        scene_key = scene.strip().lower()
        if os.path.isabs(scene):
            return scene

        scene_file = scene_alias.get(scene_key, scene)
        scene_path = os.path.join(robot_dir, scene_file)
        if os.path.isfile(scene_path):
            return scene_path

        return os.path.join(robot_dir, config.SCENE_FLAT)

    @staticmethod
    def _rotation_matrix_from_quat(quat_wxyz):
        q = np.array(quat_wxyz, dtype=float)
        norm = np.linalg.norm(q)
        if norm < 1e-12:
            return np.eye(3)

        w, x, y, z = q / norm
        return np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=float,
        )

    def _base_pose_world(self):
        if self.trunk_body_id >= 0:
            base_pos = np.array(self.data.xpos[self.trunk_body_id], dtype=float)
            base_quat = np.array(self.data.xquat[self.trunk_body_id], dtype=float)
            return base_pos, base_quat

        frame_pos = self._read_sensor_vector(self.frame_pos_slice, 3, default=[0.0, 0.0, 0.0])
        quat_raw = self._read_sensor_vector(self.imu_quat_slice, 4, default=[1.0, 0.0, 0.0, 0.0])
        return np.array(frame_pos, dtype=float), np.array(quat_raw, dtype=float)

    def _geom_footprint_xy_world(self, geom_id: int):
        center = np.array(self.data.geom_xpos[geom_id], dtype=float)
        xmat = np.array(self.data.geom_xmat[geom_id], dtype=float).reshape(3, 3)
        sx = float(self.model.geom_size[geom_id][0])
        sy = float(self.model.geom_size[geom_id][1])

        corners = []
        for sign_x in (-1.0, 1.0):
            for sign_y in (-1.0, 1.0):
                local = np.array([sign_x * sx, sign_y * sy, 0.0], dtype=float)
                world = center + xmat @ local
                corners.append(world[:2])

        return np.array(corners, dtype=float)

    def _extract_bridge_gaps_world(self):
        bridge_geoms = []
        for geom_id in range(self.model.ngeom):
            geom_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
            if geom_name is None:
                continue

            match = re.fullmatch(r"bridge_(\d+)", str(geom_name))
            if match is None:
                continue

            bridge_geoms.append((int(match.group(1)), geom_id))

        bridge_geoms.sort(key=lambda item: item[0])

        if len(bridge_geoms) < 2:
            return []

        gaps = []
        for (_, geom_a), (_, geom_b) in zip(bridge_geoms[:-1], bridge_geoms[1:]):
            corners_a = self._geom_footprint_xy_world(geom_a)
            corners_b = self._geom_footprint_xy_world(geom_b)

            center_a = np.mean(corners_a, axis=0)
            center_b = np.mean(corners_b, axis=0)
            dir_w = center_b - center_a
            dir_norm = np.linalg.norm(dir_w)
            if dir_norm < 1e-9:
                continue
            dir_w = dir_w / dir_norm
            lat_w = np.array([-dir_w[1], dir_w[0]], dtype=float)

            s_a = corners_a @ dir_w
            s_b = corners_b @ dir_w
            l_a = corners_a @ lat_w
            l_b = corners_b @ lat_w

            s_start = float(np.max(s_a))
            s_end = float(np.min(s_b))
            if s_end <= s_start + 1e-6:
                continue

            l_min = float(max(np.min(l_a), np.min(l_b)))
            l_max = float(min(np.max(l_a), np.max(l_b)))
            if l_max <= l_min + 1e-6:
                continue

            top_a = float(self.data.geom_xpos[geom_a][2] + self.model.geom_size[geom_a][2])
            top_b = float(self.data.geom_xpos[geom_b][2] + self.model.geom_size[geom_b][2])
            z_world = max(top_a, top_b)

            gaps.append(
                {
                    "s_start": s_start,
                    "s_end": s_end,
                    "l_min": l_min,
                    "l_max": l_max,
                    "z_world": z_world,
                    "dir_w": dir_w,
                    "lat_w": lat_w,
                }
            )

        return gaps

    def get_base_position(self):
        frame_pos = self._read_sensor_vector(self.frame_pos_slice, 3, default=[0.0, 0.0, 0.0])
        return float(frame_pos[0]), float(frame_pos[1]), float(frame_pos[2])

    def run_render(self):
        while rclpy.ok() and self.viewer is not None and self.viewer.is_running():
            with self.lock:
                self.viewer.sync()
            time.sleep(self.render_period)

    def run_simulation(self):
        while rclpy.ok() and (self.viewer is None or self.viewer.is_running()):
            step_start = time.perf_counter()
            with self.lock:
                for i in range(self.actuator_num):
                    self.delayed_ctrl_cache[i] = self.ctrl_buffer[i][0]
                self.data.ctrl[: self.actuator_num] = self.delayed_ctrl_cache

                mujoco.mj_step(self.model, self.data)

                delayed_values = np.array(self.data.sensordata, copy=True)
                joint_q = self.data.sensordata[self.joint_pos_data_indices_np]
                joint_dq = self.data.sensordata[self.joint_vel_data_indices_np]
                for i in range(self.actuator_num):
                    self.joint_state_buffer[i].append((float(joint_q[i]), float(joint_dq[i])))
                    delayed_state = self.joint_state_buffer[i][0]
                    self.delayed_joint_pos_cache[i] = delayed_state[0]
                    self.delayed_joint_vel_cache[i] = delayed_state[1]

                delayed_values[self.joint_pos_data_indices_np] = self.delayed_joint_pos_cache
                delayed_values[self.joint_vel_data_indices_np] = self.delayed_joint_vel_cache

                self.values = delayed_values

            if self.real_time:
                time_until_next_step = self.model.opt.timestep - (time.perf_counter() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)

    def joint_cmd_callback(self, msg):
        if self.data is None:
            return

        with self.lock:
            cmd_count = len(msg.motor_command)
            if cmd_count < self.actuator_num:
                self.get_logger().warn(
                    f"Received RobotCommand with {cmd_count} motors, expected at least {self.actuator_num}. Ignoring command."
                )
                return

            q_feedback = self.values[self.logical_joint_pos_indices]
            dq_feedback = self.values[self.logical_joint_vel_indices]

            cmd_q = np.fromiter((cmd.q for cmd in msg.motor_command[: self.actuator_num]), dtype=float)
            cmd_dq = np.fromiter((cmd.dq for cmd in msg.motor_command[: self.actuator_num]), dtype=float)
            cmd_tau = np.fromiter((cmd.tau for cmd in msg.motor_command[: self.actuator_num]), dtype=float)
            cmd_kp = np.fromiter((cmd.kp for cmd in msg.motor_command[: self.actuator_num]), dtype=float)
            cmd_kd = np.fromiter((cmd.kd for cmd in msg.motor_command[: self.actuator_num]), dtype=float)

            pos_error = cmd_q - q_feedback
            if self.calf_logical_ids.size > 0:
                calf_error = pos_error[self.calf_logical_ids]
                calf_deadband = self.calf_backlash_by_logical[self.calf_logical_ids]
                pos_error[self.calf_logical_ids] = np.where(
                    np.abs(calf_error) <= calf_deadband,
                    0.0,
                    calf_error - np.sign(calf_error) * calf_deadband,
                )

            u = cmd_tau + cmd_kp * pos_error + cmd_kd * (cmd_dq - dq_feedback)
            for logical_id, phy_id in enumerate(self.motor_map):
                self.ctrl_buffer[int(phy_id)].append(float(u[logical_id]))

    def publish_data(self):
        with self.lock:
            stamp_msg = self.get_clock().now().to_msg()
            self.publish_imu_data()
            self.publish_joint_state()
            if self.publish_odom:
                self.publish_ground_truth_odom(stamp_msg)
            if self.gap_model_publisher is not None:
                self.publish_gap_model_data(stamp_msg)

    def publish_joint_state(self):
        robot_state_msg = RobotState()
        base_q = self.values[self.logical_joint_pos_indices]
        base_dq = self.values[self.logical_joint_vel_indices]
        if self.enable_additional_noise:
            q_noise = np.random.normal(0.0, config.ENCODER_POSITION_NOISE, self.actuator_num)
            dq_noise = np.random.normal(0.0, config.ENCODER_VELOCITY_NOISE, self.actuator_num)
        else:
            q_noise = np.zeros(self.actuator_num, dtype=float)
            dq_noise = np.zeros(self.actuator_num, dtype=float)

        for i in range(self.actuator_num):
            motor_state = MotorState()
            motor_state.q = float(base_q[i] + q_noise[i] + self.encoder_position_bias[i])
            motor_state.dq = float(base_dq[i] + dq_noise[i] + self.encoder_velocity_bias[i])
            motor_state.ddq = 0.0
            motor_state.tau_est = 0.0
            motor_state.cur = 0.0

            robot_state_msg.motor_state.append(motor_state)

        self.joint_publisher.publish(robot_state_msg)

    def publish_imu_data(self):
        imu_msg = Imu()

        quat_raw = self._read_sensor_vector(self.imu_quat_slice, 4, default=[1.0, 0.0, 0.0, 0.0])
        gyro_raw = self._read_sensor_vector(self.imu_gyro_slice, 3)
        acc_raw = self._read_sensor_vector(self.imu_acc_slice, 3)

        if self.enable_additional_noise:
            angle_noise_static = np.random.normal(0, 0.7, 3)
            angle_noise_static_filtered = 0.05 * angle_noise_static
            angle_noise = np.random.normal(angle_noise_static_filtered, config.NOISE_QUAT, 3)
            angle_noise_filtered = 0.1 * angle_noise
            q_noised = self.quat_multiply(self.axis_angle_to_quat(angle_noise_filtered), quat_raw)
        else:
            q_noised = np.array(quat_raw, dtype=float)

        q_noised = self._normalize_quat(q_noised)
        q_output = self._low_pass_imu_attitude(q_noised)

        imu_msg.orientation.w = float(q_output[0])
        imu_msg.orientation.x = float(q_output[1])
        imu_msg.orientation.y = float(q_output[2])
        imu_msg.orientation.z = float(q_output[3])

        if self.enable_additional_noise:
            gyro = gyro_raw + np.random.normal(0.0, config.NOISE_GYRO, 3)
            acc = acc_raw + np.random.normal(0.0, config.NOISE_ACC, 3)
        else:
            gyro = gyro_raw
            acc = acc_raw

        imu_msg.angular_velocity.x = float(gyro[0])
        imu_msg.angular_velocity.y = float(gyro[1])
        imu_msg.angular_velocity.z = float(gyro[2])

        imu_msg.linear_acceleration.x = float(acc[0])
        imu_msg.linear_acceleration.y = float(acc[1])
        imu_msg.linear_acceleration.z = float(acc[2])

        self.imu_publisher.publish(imu_msg)

    def publish_ground_truth_odom(self, stamp_msg=None):
        if self.odom_publisher is None:
            return

        if stamp_msg is None:
            stamp_msg = self.get_clock().now().to_msg()

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp_msg
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base"

        if self.trunk_body_id >= 0:
            odom_msg.pose.pose.position.x = float(self.data.xpos[self.trunk_body_id][0])
            odom_msg.pose.pose.position.y = float(self.data.xpos[self.trunk_body_id][1])
            odom_msg.pose.pose.position.z = float(self.data.xpos[self.trunk_body_id][2])
            odom_msg.pose.pose.orientation.w = float(self.data.xquat[self.trunk_body_id][0])
            odom_msg.pose.pose.orientation.x = float(self.data.xquat[self.trunk_body_id][1])
            odom_msg.pose.pose.orientation.y = float(self.data.xquat[self.trunk_body_id][2])
            odom_msg.pose.pose.orientation.z = float(self.data.xquat[self.trunk_body_id][3])

            body_vel = np.zeros(6, dtype=float)
            mujoco.mj_objectVelocity(
                self.model,
                self.data,
                mujoco.mjtObj.mjOBJ_BODY,
                self.trunk_body_id,
                body_vel,
                1,
            )
            odom_msg.twist.twist.angular.x = float(body_vel[0])
            odom_msg.twist.twist.angular.y = float(body_vel[1])
            odom_msg.twist.twist.angular.z = float(body_vel[2])
            odom_msg.twist.twist.linear.x = float(body_vel[3])
            odom_msg.twist.twist.linear.y = float(body_vel[4])
            odom_msg.twist.twist.linear.z = float(body_vel[5])
        else:
            frame_pos = self._read_sensor_vector(self.frame_pos_slice, 3)
            frame_vel = self._read_sensor_vector(self.frame_vel_slice, 3)
            quat_raw = self._read_sensor_vector(self.imu_quat_slice, 4, default=[1.0, 0.0, 0.0, 0.0])
            gyro_raw = self._read_sensor_vector(self.imu_gyro_slice, 3)

            odom_msg.pose.pose.position.x = float(frame_pos[0])
            odom_msg.pose.pose.position.y = float(frame_pos[1])
            odom_msg.pose.pose.position.z = float(frame_pos[2])
            odom_msg.pose.pose.orientation.w = float(quat_raw[0])
            odom_msg.pose.pose.orientation.x = float(quat_raw[1])
            odom_msg.pose.pose.orientation.y = float(quat_raw[2])
            odom_msg.pose.pose.orientation.z = float(quat_raw[3])
            odom_msg.twist.twist.linear.x = float(frame_vel[0])
            odom_msg.twist.twist.linear.y = float(frame_vel[1])
            odom_msg.twist.twist.linear.z = float(frame_vel[2])
            odom_msg.twist.twist.angular.x = float(gyro_raw[0])
            odom_msg.twist.twist.angular.y = float(gyro_raw[1])
            odom_msg.twist.twist.angular.z = float(gyro_raw[2])

        self.odom_publisher.publish(odom_msg)

    def _build_missing_gap_marker(self):
        marker = float(self.gap_missing_marker)
        gap_msg = Gap()
        gap_msg.xs = marker
        gap_msg.xe = marker
        gap_msg.ymin = marker
        gap_msg.ymax = marker
        gap_msg.confidence = 0.0
        gap_msg.has_bridge_dir_b = False
        gap_msg.bridge_dir_b = [0.0, 0.0]
        return gap_msg

    @staticmethod
    def _intersect_base_x_axis_with_polygon(corners_xy, eps=1e-9):
        intersections = []
        n = int(corners_xy.shape[0])

        for i in range(n):
            x1, y1 = corners_xy[i]
            x2, y2 = corners_xy[(i + 1) % n]

            if abs(y1) <= eps and abs(y2) <= eps:
                intersections.extend([float(x1), float(x2)])
                continue

            if (y1 > eps and y2 > eps) or (y1 < -eps and y2 < -eps):
                continue

            dy = y2 - y1
            if abs(dy) <= eps:
                continue

            t = -y1 / dy
            if t < -eps or t > 1.0 + eps:
                continue

            t = float(np.clip(t, 0.0, 1.0))
            intersections.append(float(x1 + t * (x2 - x1)))

        if len(intersections) == 0:
            return []

        intersections.sort()
        unique = [intersections[0]]
        for x in intersections[1:]:
            if abs(x - unique[-1]) > 1e-7:
                unique.append(x)

        return unique

    def publish_gap_model_data(self, stamp_msg=None):
        if self.gap_model_publisher is None:
            return

        if stamp_msg is None:
            stamp_msg = self.get_clock().now().to_msg()

        gap_model_msg = GapModel()
        gap_model_msg.header.stamp = stamp_msg
        gap_model_msg.header.frame_id = "base_link"

        base_pos, base_quat = self._base_pose_world()
        rot_bw = self._rotation_matrix_from_quat(base_quat).T

        gap_candidates = []

        for gap_world in self.bridge_gaps_world:
            dir_w = gap_world["dir_w"]
            lat_w = gap_world["lat_w"]
            s_start = gap_world["s_start"]
            s_end = gap_world["s_end"]
            l_min = gap_world["l_min"]
            l_max = gap_world["l_max"]
            z_world = gap_world["z_world"]
            corners_world = np.array(
                [
                    [
                        dir_w[0] * s_start + lat_w[0] * l_min,
                        dir_w[1] * s_start + lat_w[1] * l_min,
                        z_world,
                    ],
                    [
                        dir_w[0] * s_start + lat_w[0] * l_max,
                        dir_w[1] * s_start + lat_w[1] * l_max,
                        z_world,
                    ],
                    [
                        dir_w[0] * s_end + lat_w[0] * l_min,
                        dir_w[1] * s_end + lat_w[1] * l_min,
                        z_world,
                    ],
                    [
                        dir_w[0] * s_end + lat_w[0] * l_max,
                        dir_w[1] * s_end + lat_w[1] * l_max,
                        z_world,
                    ],
                ],
                dtype=float,
            )

            corners_base = np.array([rot_bw @ (corner - base_pos) for corner in corners_world], dtype=float)

            # a: base origin; b/c: intersections of base-x axis with gap boundary.
            gap_polygon_xy = corners_base[[0, 1, 3, 2], :2]
            x_intersections = self._intersect_base_x_axis_with_polygon(gap_polygon_xy)
            if len(x_intersections) < 2:
                continue

            xs = float(x_intersections[0])
            xe = float(x_intersections[-1])

            gap_msg = Gap()
            gap_msg.xs = xs
            gap_msg.xe = xe
            gap_msg.ymin = float(np.min(corners_base[:, 1]))
            gap_msg.ymax = float(np.max(corners_base[:, 1]))
            gap_msg.confidence = self.gap_model_confidence

            bridge_dir_base = rot_bw @ np.array([dir_w[0], dir_w[1], 0.0], dtype=float)
            bridge_dir_xy = bridge_dir_base[:2]
            bridge_norm = np.linalg.norm(bridge_dir_xy)
            if bridge_norm > 1e-9:
                bridge_dir_xy = bridge_dir_xy / bridge_norm
                gap_msg.has_bridge_dir_b = True
                gap_msg.bridge_dir_b = [float(bridge_dir_xy[0]), float(bridge_dir_xy[1])]
            else:
                gap_msg.has_bridge_dir_b = False
                gap_msg.bridge_dir_b = [0.0, 0.0]

            gap_candidates.append((xs, xe, gap_msg))

        front_msg = self._build_missing_gap_marker()
        back_msg = self._build_missing_gap_marker()

        front_candidates = [item for item in gap_candidates if item[1] >= 0.0]
        if front_candidates:
            _, _, front_msg = min(front_candidates, key=lambda item: item[0])

        back_candidates = [item for item in gap_candidates if item[1] <= 0.0]
        if back_candidates:
            _, _, back_msg = max(back_candidates, key=lambda item: item[1])

        gap_model_msg.gaps.append(front_msg)
        gap_model_msg.gaps.append(back_msg)

        self.gap_model_publisher.publish(gap_model_msg)

    @staticmethod
    def quat_multiply(q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array(
            [
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            ]
        )

    @staticmethod
    def _normalize_quat(quat):
        quat = np.array(quat, dtype=float)
        norm = np.linalg.norm(quat)
        if norm < 1e-12:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        return quat / norm

    def _low_pass_imu_attitude(self, quat):
        quat = self._normalize_quat(quat)
        alpha = self.imu_attitude_lpf_alpha
        if alpha >= 1.0 or self.filtered_imu_quat is None:
            self.filtered_imu_quat = quat
            return quat

        previous = self._normalize_quat(self.filtered_imu_quat)
        if np.dot(previous, quat) < 0.0:
            quat = -quat

        dot = float(np.clip(np.dot(previous, quat), -1.0, 1.0))
        if dot > 0.9995:
            filtered = self._normalize_quat((1.0 - alpha) * previous + alpha * quat)
        else:
            theta_0 = np.arccos(dot)
            sin_theta_0 = np.sin(theta_0)
            theta = theta_0 * alpha
            scale_previous = np.cos(theta) - dot * np.sin(theta) / sin_theta_0
            scale_current = np.sin(theta) / sin_theta_0
            filtered = self._normalize_quat(scale_previous * previous + scale_current * quat)

        self.filtered_imu_quat = filtered
        return filtered

    @staticmethod
    def axis_angle_to_quat(axis_angle):
        angle = np.linalg.norm(axis_angle)
        if angle < 1e-12:
            return np.array([1, 0, 0, 0])

        axis = axis_angle / angle
        half = angle * 0.5
        return np.array(
            [
                np.cos(half),
                axis[0] * np.sin(half),
                axis[1] * np.sin(half),
                axis[2] * np.sin(half),
            ]
        )

    def reset_delay(self):
        self._initialize_delay_buffers()
        self.filtered_imu_quat = None


def main(args=None):
    rclpy.init(args=args)
    node = MujocoNode("mujoco_runner")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
