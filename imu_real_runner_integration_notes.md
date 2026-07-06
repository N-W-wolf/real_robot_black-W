# IMU / real_runner integration notes

Goal: reduce the robot-side runtime to `real_runner` plus the top-level
`rl_sar` controller node.

Planned data path:

- `real_runner` opens the IMU serial device directly.
- IMU serial alias remains `/dev/IMU_Link`, matching the previous
  `ag_ros_node` launch file.
- `real_runner` parses AB5465 IMU frames, applies the same axis/sign mapping,
  runs the existing VQF orientation filter, publishes `/_lowState/imu` for
  observers and `rl_sar`, and uses the same in-process IMU data for its safety
  watchdog.
- `rl_sar` publishes commands directly to `/_lowCmd/command`.
- `rl_sar` subscribes directly to `/_lowState/joint` and `/_lowState/imu`.

Nodes removed from the robot-side launch path:

- `ag_ros_node`
- `imu_vqf_filter`
- `middleware`
- `param_node`
