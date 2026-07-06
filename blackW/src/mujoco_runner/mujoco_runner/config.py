# Simulation timing
SIM_DT = 0.005
A1_SIM_DT = 0.0005
DEFAULT_RENDER_RATE_HZ = 60.0
RENDER_DT = 1.0 / DEFAULT_RENDER_RATE_HZ

# Scene selection
SCENE_FLAT = 'scene_flat.xml'
SCENE_OBSTACLE = 'scene.xml'
SCENE_TERRAIN = 'scene_terrain.xml'
SCENE_ROBOCON = 'scene_robocon.xml'
SCENE_DEFAULT = 'terrain'
DEFAULT_ROBOT_NAME = "blackW"

# Backward-compatibility aliases
SENCE_TERRAIN = SCENE_TERRAIN
SENCE_PLANE = SCENE_FLAT

# Artificial sensor/actuator non-ideal effects
ENABLE_ADDITIONAL_NOISE = False

# IMU noise
NOISE_QUAT = 8.36e-2
NOISE_GYRO = 1.94e-1
NOISE_ACC = 5.88e-2

# Actuator / encoder model
MOTOR_DELAY = 0.003  # seconds
ENCODER_POSITION_NOISE = 0.001
ENCODER_VELOCITY_NOISE = 0.01
MOTOR_BIAS = 0.02

CALF_BACKLASH_MIN_DEG = 0.0
CALF_BACKLASH_MAX_DEG = 0.0

LEGGED_JOINT_POS_SENSOR_NAMES = [
    "FL_hip_pos",
    "FL_thigh_pos",
    "FL_calf_pos",
    "FR_hip_pos",
    "FR_thigh_pos",
    "FR_calf_pos",
    "RL_hip_pos",
    "RL_thigh_pos",
    "RL_calf_pos",
    "RR_hip_pos",
    "RR_thigh_pos",
    "RR_calf_pos",
]

LEGGED_JOINT_VEL_SENSOR_NAMES = [
    "FL_hip_vel",
    "FL_thigh_vel",
    "FL_calf_vel",
    "FR_hip_vel",
    "FR_thigh_vel",
    "FR_calf_vel",
    "RL_hip_vel",
    "RL_thigh_vel",
    "RL_calf_vel",
    "RR_hip_vel",
    "RR_thigh_vel",
    "RR_calf_vel",
]

BLACKW_JOINT_POS_SENSOR_NAMES = [
    "FL_hip_pos",
    "FL_thigh_pos",
    "FL_calf_pos",
    "FL_wheel_pos",
    "FR_hip_pos",
    "FR_thigh_pos",
    "FR_calf_pos",
    "FR_wheel_pos",
    "RR_hip_pos",
    "RR_thigh_pos",
    "RR_calf_pos",
    "RR_wheel_pos",
    "RL_hip_pos",
    "RL_thigh_pos",
    "RL_calf_pos",
    "RL_wheel_pos",
]

BLACKW_JOINT_VEL_SENSOR_NAMES = [
    "FL_hip_vel",
    "FL_thigh_vel",
    "FL_calf_vel",
    "FL_wheel_vel",
    "FR_hip_vel",
    "FR_thigh_vel",
    "FR_calf_vel",
    "FR_wheel_vel",
    "RR_hip_vel",
    "RR_thigh_vel",
    "RR_calf_vel",
    "RR_wheel_vel",
    "RL_hip_vel",
    "RL_thigh_vel",
    "RL_calf_vel",
    "RL_wheel_vel",
]

GO2W_JOINT_POS_SENSOR_NAMES = [
    "FL_hip_joint_pos",
    "FL_thigh_joint_pos",
    "FL_calf_joint_pos",
    "FL_wheel_joint_pos",
    "FR_hip_joint_pos",
    "FR_thigh_joint_pos",
    "FR_calf_joint_pos",
    "FR_wheel_joint_pos",
    "RL_hip_joint_pos",
    "RL_thigh_joint_pos",
    "RL_calf_joint_pos",
    "RL_wheel_joint_pos",
    "RR_hip_joint_pos",
    "RR_thigh_joint_pos",
    "RR_calf_joint_pos",
    "RR_wheel_joint_pos",
]

GO2W_JOINT_VEL_SENSOR_NAMES = [
    "FL_hip_joint_vel",
    "FL_thigh_joint_vel",
    "FL_calf_joint_vel",
    "FL_wheel_joint_vel",
    "FR_hip_joint_vel",
    "FR_thigh_joint_vel",
    "FR_calf_joint_vel",
    "FR_wheel_joint_vel",
    "RL_hip_joint_vel",
    "RL_thigh_joint_vel",
    "RL_calf_joint_vel",
    "RL_wheel_joint_vel",
    "RR_hip_joint_vel",
    "RR_thigh_joint_vel",
    "RR_calf_joint_vel",
    "RR_wheel_joint_vel",
]

LEGGED_PROFILE = {
    "route_name": "legacy_legged",
    "actuator_num": 12,
    "joint_pos_sensor_names": LEGGED_JOINT_POS_SENSOR_NAMES,
    "joint_vel_sensor_names": LEGGED_JOINT_VEL_SENSOR_NAMES,
    # Index(i): logical motor id in ROS messages
    # Value: physical motor id in MuJoCo
    # logical order: FR, FL, RR, RL
    "motor_mapping": [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8],
    # Physical actuator order in model is FL, FR, RL, RR.
    "default_pos": [
        0.0, -0.44, 0.05,
        -0.0, 0.44, -0.05,
        0.0, -0.44, -0.05,
        -0.0, 0.44, -0.05,
    ],
}

BLACKW_PROFILE = {
    "route_name": "wheel_leg_blackw",
    "actuator_num": 16,
    "joint_pos_sensor_names": BLACKW_JOINT_POS_SENSOR_NAMES,
    "joint_vel_sensor_names": BLACKW_JOINT_VEL_SENSOR_NAMES,
    "imu_attitude_lpf_alpha": 0.01,
    # logical order: FL, FR, RL, RR, each leg has hip/thigh/calf/wheel.
    # physical order in blackW model: FL, FR, RR, RL.
    "motor_mapping": [4, 5, 6, 7, 0, 1, 2, 3, 8, 9, 10, 11, 12, 13, 14, 15],
    "default_pos": [
        0.0, 0.9, -1.82, 0.0,
        0.0, -0.9, 1.82, 0.0,
        0.0, 0.9, -1.82, 0.0,
        0.0, -0.9, 1.82, 0.0,
    ],
}

GO2W_PROFILE = {
    "route_name": "wheel_leg_go2w",
    "actuator_num": 16,
    "joint_pos_sensor_names": GO2W_JOINT_POS_SENSOR_NAMES,
    "joint_vel_sensor_names": GO2W_JOINT_VEL_SENSOR_NAMES,
    # HIMLoco Go2W policy order and copied MJCF physical order are both
    # FL, FR, RL, RR, each leg has hip/thigh/calf/wheel.
    "motor_mapping": list(range(16)),
    "default_pos": [
        0.0, 0.8, -1.5, 0.0,
        0.0, 0.8, -1.5, 0.0,
        0.0, 0.8, -1.5, 0.0,
        0.0, 0.8, -1.5, 0.0,
    ],
}

ROBOT_PROFILES = {
    "default": LEGGED_PROFILE,
    "black": LEGGED_PROFILE,
    "a1": LEGGED_PROFILE,
    "blackw": BLACKW_PROFILE,
    "go2w": GO2W_PROFILE,
}

ROBOT_NAME_ALIASES = {
    "blackw": "blackW",
}


def resolve_robot_name(robot_name: str) -> str:
    normalized = str(robot_name).strip() or DEFAULT_ROBOT_NAME
    return ROBOT_NAME_ALIASES.get(normalized.lower(), normalized)


def get_robot_profile(robot_name: str):
    key = str(robot_name).strip().lower() or DEFAULT_ROBOT_NAME.lower()
    return ROBOT_PROFILES.get(key, ROBOT_PROFILES["default"])


def get_sim_dt(robot_name: str) -> float:
    return A1_SIM_DT if str(robot_name).strip().lower() == "a1" else SIM_DT


# Legacy exports kept for backward compatibility.
DEFAULT_POS = LEGGED_PROFILE["default_pos"]
ACTUATOR_NUM = LEGGED_PROFILE["actuator_num"]
MOTOR_MAPPING = LEGGED_PROFILE["motor_mapping"]
