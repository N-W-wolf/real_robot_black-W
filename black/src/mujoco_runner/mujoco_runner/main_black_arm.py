import os
import threading
import time
from collections import deque

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from robot_msgs.msg import MotorState, RobotCommand, RobotState
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32
from std_srvs.srv import SetBool

from . import config
from .arm_trajectory_library import (
    ARM_DAMPING,
    ARM_HOME_POSE,
    ARM_JOINT_NAMES,
    ARM_STIFFNESS,
    ARM_TRAJECTORY_LIBRARY,
)

KEY_SPACE = 32
KEY_1 = 49
KEY_2 = 50
KEY_3 = 51
KEY_H_UPPER = 72
KEY_N_UPPER = 78
KEY_h_LOWER = 104
KEY_n_LOWER = 110


class MujocoNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        self.declare_parameter("rname", "black_with_arm")
        rname = self.get_parameter("rname").value
        self.get_logger().info(f"loading robot is :{rname}")

        if config.USE_TERRAIN:
            path = os.path.join(
                get_package_share_directory("robot_description"),
                rname,
                config.SENCE_TERRAIN,
            )
        else:
            path = os.path.join(
                get_package_share_directory("robot_description"),
                rname,
                config.SENCE_PLANE,
            )

        self.model = mujoco.MjModel.from_xml_path(path)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)
        self.model.opt.timestep = config.SIM_DT
        self.control_dt = getattr(config, "CONTROL_DT", self.model.opt.timestep)
        self.sim_substeps = max(1, int(round(self.control_dt / self.model.opt.timestep)))
        self.viewer = mujoco.viewer.launch_passive(
            self.model,
            self.data,
            key_callback=self.viewer_key_callback,
        )
        self.next_render_time = time.perf_counter()

        self.lock = threading.RLock()
        self.motor_map = config.MOTOR_MAPPING
        self.leg_actuator_num = config.ACTUATOR_NUM
        self.total_actuator_num = self.model.nu
        self.arm_actuator_num = self.total_actuator_num - self.leg_actuator_num
        if self.arm_actuator_num != len(ARM_JOINT_NAMES):
            raise RuntimeError(
                f"Expected {len(ARM_JOINT_NAMES)} arm actuators, got {self.arm_actuator_num}"
            )

        self.values = np.array(self.data.sensordata, copy=True)
        self.sensor_joint_num = self.total_actuator_num
        self.imu_sensor_offset = 2 * self.sensor_joint_num
        self.base_pos_offset = self.imu_sensor_offset + 10

        self.arm_qpos_indices = []
        self.arm_qvel_indices = []
        for joint_name in ARM_JOINT_NAMES:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id < 0:
                raise RuntimeError(f"Failed to find arm joint {joint_name}")
            self.arm_qpos_indices.append(self.model.jnt_qposadr[joint_id])
            self.arm_qvel_indices.append(self.model.jnt_dofadr[joint_id])

        self.arm_home_pose = np.array(ARM_HOME_POSE, dtype=float)
        self.arm_kp = np.array(ARM_STIFFNESS, dtype=float)
        self.arm_kd = np.array(ARM_DAMPING, dtype=float)
        self.arm_torque_limit = np.full(self.arm_actuator_num, 12.0, dtype=float)
        self.arm_enabled = False
        self.arm_selected_traj = 0
        self.arm_elapsed = 0.0
        self.arm_trajectory_refs = []
        self.arm_segment_durations = []
        self._build_arm_trajectory_library()

        self.ctrl_buffer = []
        self.joint_state_buffer = []
        self.reset_delay()

        self.encoder_position_bias = np.random.uniform(
            -config.MOTOR_BIAS, config.MOTOR_BIAS, self.leg_actuator_num
        )
        self.encoder_velocity_bias = np.random.uniform(
            -config.MOTOR_BIAS, config.MOTOR_BIAS, self.leg_actuator_num
        )

        if rname == "a1":
            self.model.opt.timestep = 0.0005
            self.sim_substeps = max(1, int(round(self.control_dt / self.model.opt.timestep)))

        self.imu_publisher = self.create_publisher(Imu, "/_lowState/imu", 10)
        self.joint_publisher = self.create_publisher(RobotState, "/_lowState/joint", 10)
        self.joint_subscriber = self.create_subscription(
            RobotCommand, "/_lowCmd/command", self.joint_cmd_callback, 10
        )
        self.arm_select_subscriber = self.create_subscription(
            Int32, "/arm_motion/select", self.arm_select_callback, 10
        )
        self.arm_enable_service = self.create_service(
            SetBool, "/arm_motion/enable", self.arm_enable_callback
        )
        self.timer = self.create_timer(config.PUBLISH_DT, self.publish_data)
        self.get_logger().info(
            "Arm hotkeys: 1/2/3 select and start trajectory, Space toggles current trajectory, H returns arm home"
        )

    def _build_arm_trajectory_library(self):
        self.arm_trajectory_refs = []
        self.arm_segment_durations = []
        for traj_cfg in ARM_TRAJECTORY_LIBRARY:
            refs = [self.arm_home_pose.copy()]
            for waypoint in traj_cfg["waypoints"]:
                refs.append(np.array([waypoint[name] for name in ARM_JOINT_NAMES], dtype=float))
            if len(refs) > 1:
                refs.append(self.arm_home_pose.copy())
            self.arm_trajectory_refs.append(np.stack(refs, axis=0))
            self.arm_segment_durations.append(max(float(traj_cfg["segment_duration"]), 1e-3))

    def _compute_arm_reference(self):
        if not self.arm_enabled or not self.arm_trajectory_refs:
            return self.arm_home_pose.copy()

        traj = self.arm_trajectory_refs[self.arm_selected_traj]
        num_segments = traj.shape[0] - 1
        if num_segments <= 0:
            return self.arm_home_pose.copy()

        segment_duration = self.arm_segment_durations[self.arm_selected_traj]
        cycle_duration = segment_duration * num_segments
        phase_time = np.fmod(max(self.arm_elapsed, 0.0), cycle_duration)
        segment_idx = min(int(np.floor(phase_time / segment_duration)), num_segments - 1)
        alpha = (phase_time - segment_idx * segment_duration) / segment_duration
        return traj[segment_idx] + alpha * (traj[segment_idx + 1] - traj[segment_idx])

    def _compute_arm_ctrl(self):
        q_ref = self._compute_arm_reference()
        q_cur = self.data.qpos[self.arm_qpos_indices]
        dq_cur = self.data.qvel[self.arm_qvel_indices]
        ctrl = self.arm_kp * (q_ref - q_cur) - self.arm_kd * dq_cur
        return np.clip(ctrl, -self.arm_torque_limit, self.arm_torque_limit)

    def _set_arm_enabled(self, enabled):
        self.arm_enabled = bool(enabled)
        self.arm_elapsed = 0.0
        return self.arm_enabled

    def _select_arm_trajectory(self, traj_id, auto_enable=False):
        if traj_id < 0 or traj_id >= len(self.arm_trajectory_refs):
            self.get_logger().warn(
                f"Invalid arm trajectory id {traj_id}, valid range is [0, {len(self.arm_trajectory_refs) - 1}]"
            )
            return False

        self.arm_selected_traj = traj_id
        self.arm_elapsed = 0.0
        if auto_enable:
            self.arm_enabled = True

        state_text = " and started" if auto_enable else ""
        self.get_logger().info(
            f"Selected arm trajectory {traj_id}: {ARM_TRAJECTORY_LIBRARY[traj_id]['name']}{state_text}"
        )
        return True

    def viewer_key_callback(self, keycode):
        with self.lock:
            if keycode == KEY_SPACE:
                enabled = self._set_arm_enabled(not self.arm_enabled)
                state_text = "enabled" if enabled else "disabled and returning home"
                self.get_logger().info(f"Arm trajectory {state_text}")
                return

            if keycode in (KEY_H_UPPER, KEY_h_LOWER):
                self._set_arm_enabled(False)
                self.get_logger().info("Arm returned to home pose")
                return

            if keycode in (KEY_1, KEY_2, KEY_3):
                self._select_arm_trajectory(keycode - KEY_1, auto_enable=True)
                return

            if keycode in (KEY_N_UPPER, KEY_n_LOWER):
                next_id = (self.arm_selected_traj + 1) % len(self.arm_trajectory_refs)
                self._select_arm_trajectory(next_id, auto_enable=True)

    def arm_enable_callback(self, request, response):
        with self.lock:
            arm_enabled = self._set_arm_enabled(request.data)
        response.success = True
        response.message = (
            "arm trajectory enabled"
            if arm_enabled
            else "arm trajectory disabled and returning home"
        )
        self.get_logger().info(response.message)
        return response

    def arm_select_callback(self, msg):
        with self.lock:
            self._select_arm_trajectory(int(msg.data))

    def get_base_position(self, values):
        base_x = values[self.base_pos_offset]
        base_y = values[self.base_pos_offset + 1]
        base_z = values[self.base_pos_offset + 2]
        return base_x, base_y, base_z

    def run_render(self):
        while self.viewer.is_running():
            with self.lock:
                self.viewer.sync()
            time.sleep(config.RENDER_DT)

    def run_simulation(self):
        while rclpy.ok() and self.viewer.is_running():
            step_start = time.perf_counter()
            with self.lock:
                for i in range(self.leg_actuator_num):
                    self.data.ctrl[i] = self.ctrl_buffer[i][0]
                self.data.ctrl[self.leg_actuator_num : self.total_actuator_num] = self._compute_arm_ctrl()

                for _ in range(self.sim_substeps):
                    mujoco.mj_step(self.model, self.data)
                self.arm_elapsed += self.model.opt.timestep * self.sim_substeps

                delayed_values = np.array(self.data.sensordata, copy=True)
                for i in range(self.leg_actuator_num):
                    q_real = float(self.data.sensordata[i])
                    dq_real = float(self.data.sensordata[i + self.sensor_joint_num])
                    self.joint_state_buffer[i].append((q_real, dq_real))
                for i in range(self.leg_actuator_num):
                    delayed_values[i] = self.joint_state_buffer[i][0][0]
                    delayed_values[i + self.sensor_joint_num] = self.joint_state_buffer[i][0][1]
                self.values = delayed_values

            now = time.perf_counter()
            if now >= self.next_render_time:
                with self.lock:
                    self.viewer.sync()
                self.next_render_time = now + config.RENDER_DT

            time_until_next_step = self.control_dt - (time.perf_counter() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    def joint_cmd_callback(self, msg):
        if self.data is None:
            return

        with self.lock:
            cmd_count = min(len(msg.motor_command), self.leg_actuator_num)
            for i in range(cmd_count):
                cmd_q = msg.motor_command[i].q + self.encoder_position_bias[i]
                phy_id = self.motor_map[i]
                q_feedback = self.values[phy_id]
                dq_feedback = self.values[phy_id + self.sensor_joint_num]

                u = (
                    msg.motor_command[i].tau
                    + msg.motor_command[i].kp * (cmd_q - q_feedback)
                    + msg.motor_command[i].kd * (msg.motor_command[i].dq - dq_feedback)
                )
                self.ctrl_buffer[phy_id].append(u)

    def publish_data(self):
        with self.lock:
            values = np.array(self.values, copy=True)
        self.publish_imu_data(values)
        self.publish_joint_state(values)
        x, y, z = self.get_base_position(values)
        # self.get_logger().info(f"Base Pos: {x:.3f}, {y:.3f}, {z:.3f}")

    def publish_joint_state(self, values):
        robot_state_msg = RobotState()
        for i in range(self.leg_actuator_num):
            phy_id = self.motor_map[i]

            motor_state = MotorState()
            base_q = float(values[phy_id])
            base_dq = float(values[self.sensor_joint_num + phy_id])

            motor_state.q = (
                base_q
                + np.random.normal(0, config.ENCODER_POSITION_NOISE)
                + self.encoder_position_bias[phy_id]
            )
            motor_state.dq = (
                base_dq
                + np.random.normal(0, config.ENCODER_VELOCITY_NOISE)
                + self.encoder_velocity_bias[phy_id]
            )
            motor_state.ddq = 0.0
            motor_state.tau_est = 0.0
            motor_state.cur = 0.0

            robot_state_msg.motor_state.append(motor_state)
        self.joint_publisher.publish(robot_state_msg)

    def publish_imu_data(self, values):
        imu_msg = Imu()

        angle_noise_static = np.random.normal(0, 0.7, 3)
        angle_noise_static_filtered = np.array([0.0, 0.0, 0.0])
        angle_noise_static_filtered = 0.05 * angle_noise_static + 0.95 * angle_noise_static_filtered
        angle_noise = np.random.normal(angle_noise_static_filtered, config.NOISE_QUAT, 3)
        angle_noise_filtered = np.array([0.0, 0.0, 0.0])
        angle_noise_filtered = 0.1 * angle_noise + 0.9 * angle_noise_filtered
        quat_base = np.array(
            [
                values[self.imu_sensor_offset],
                values[self.imu_sensor_offset + 1],
                values[self.imu_sensor_offset + 2],
                values[self.imu_sensor_offset + 3],
            ]
        )
        q_noised = self.quat_multiply(self.axis_angle_to_quat(angle_noise_filtered), quat_base)
        q_norm = np.linalg.norm(q_noised)
        if q_norm > 0:
            q_noised /= q_norm
        else:
            q_noised = np.array([1.0, 0.0, 0.0, 0.0])

        imu_msg.orientation.w = q_noised[0]
        imu_msg.orientation.x = q_noised[1]
        imu_msg.orientation.y = q_noised[2]
        imu_msg.orientation.z = q_noised[3]

        imu_msg.angular_velocity.x = values[self.imu_sensor_offset + 4] + np.random.normal(
            0, config.NOISE_GYRO
        )
        imu_msg.angular_velocity.y = values[self.imu_sensor_offset + 5] + np.random.normal(
            0, config.NOISE_GYRO
        )
        imu_msg.angular_velocity.z = values[self.imu_sensor_offset + 6] + np.random.normal(
            0, config.NOISE_GYRO
        )

        imu_msg.linear_acceleration.x = values[self.imu_sensor_offset + 7] + np.random.normal(
            0, config.NOISE_ACC
        )
        imu_msg.linear_acceleration.y = values[self.imu_sensor_offset + 8] + np.random.normal(
            0, config.NOISE_ACC
        )
        imu_msg.linear_acceleration.z = values[self.imu_sensor_offset + 9] + np.random.normal(
            0, config.NOISE_ACC
        )

        self.imu_publisher.publish(imu_msg)

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
    def axis_angle_to_quat(axis_angle):
        angle = np.linalg.norm(axis_angle)
        if angle < 1e-12:
            return np.array([1.0, 0.0, 0.0, 0.0])

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
        self.ctrl_buffer = []
        self.joint_state_buffer = []
        for i in range(self.leg_actuator_num):
            delay_time = np.random.uniform(0, config.MOTOR_DELAY)
            delay_steps = max(1, int(delay_time / self.control_dt))
            self.ctrl_buffer.append(
                deque([float(self.data.ctrl[i])] * delay_steps, maxlen=delay_steps)
            )
            self.joint_state_buffer.append(
                deque(
                    [
                        (
                            float(self.values[i]),
                            float(self.values[i + self.sensor_joint_num]),
                        )
                    ]
                    * delay_steps,
                    maxlen=delay_steps,
                )
            )


def main(args=None):
    rclpy.init(args=args)
    node = MujocoNode("mujoco_runner_black_arm")
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.run_simulation()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
