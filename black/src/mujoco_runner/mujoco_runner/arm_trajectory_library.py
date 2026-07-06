"""Scripted arm trajectories for the black_with_arm MuJoCo runner."""

ARM_JOINT_NAMES = (
    "arm_yaw_joint",
    "arm_pitch_1_joint",
    "arm_pitch_2_joint",
    "arm_pitch_3_joint",
    "arm_roll_joint",
)

ARM_HOME_POSE = (0.0, 3.14, -2.81, -0.34, 0.0)
ARM_STIFFNESS = (30.0, 30.0, 30.0, 20.0, 10.0)
ARM_DAMPING = (1.0, 1.0, 1.0, 0.8, 0.5)


def _wp(yaw, p1, p2, p3, roll=0.0):
    return {
        "arm_yaw_joint": yaw,
        "arm_pitch_1_joint": p1,
        "arm_pitch_2_joint": p2,
        "arm_pitch_3_joint": p3,
        "arm_roll_joint": roll,
    }


def _concat(*segments):
    waypoints = []
    for segment in segments:
        waypoints.extend(segment)
    return waypoints


def _traj(name, segment_duration, *segments):
    return {
        "name": name,
        "segment_duration": segment_duration,
        "waypoints": _concat(*segments),
    }


GRASP_FRONT = [
    _wp(0.00, 2.70, -2.35, -0.05, 0.0),
    _wp(0.05, 0.28, -1.65, -0.22, 1.0),
    _wp(0.05, 1.08, -1.62, -0.50, 2.0),
    _wp(0.00, 3.01, -2.41, -0.63, -1.0),
]

TRANSFER_LEFT = [
    _wp(0.5, 2.56, -2.18, 0.08, 2.0),
    _wp(1.5, 0.74, -1.62, -0.63, 0.0),
]

TRANSFER_RIGHT = [
    _wp(-0.5, 2.56, -2.18, 0.08, -2.0),
    _wp(-1.5, 0.74, -1.62, -0.63, 0.0),
]

PLACE_LEFT = [
    _wp(1.45, 0.62, -1.52, -0.55, 0.8),
    _wp(1.45, 0.42, -1.38, -0.42, 0.3),
    _wp(1.45, 0.42, -1.38, -0.42, -0.2),
    _wp(0.90, 1.10, -1.75, -0.55, -0.5),
]

PLACE_RIGHT = [
    _wp(-1.45, 0.62, -1.52, -0.55, -0.8),
    _wp(-1.45, 0.42, -1.38, -0.42, -0.3),
    _wp(-1.45, 0.42, -1.38, -0.42, 0.2),
    _wp(-0.90, 1.10, -1.75, -0.55, 0.5),
]

LIFT_HOLD_FRONT = [
    _wp(0.00, 2.72, -2.28, -0.58, -0.8),
    _wp(0.00, 2.68, -2.24, -0.52, -0.2),
]

RETURN_FROM_HOLD = [
    _wp(0.00, 2.10, -1.85, -0.35, 0.3),
]


ARM_TRAJECTORY_LIBRARY = [
    _traj("grasp_transfer_left_place", 0.9, GRASP_FRONT, TRANSFER_LEFT, PLACE_LEFT),
    _traj("grasp_transfer_right_place", 0.9, GRASP_FRONT, TRANSFER_RIGHT, PLACE_RIGHT),
    _traj("grasp_lift_hold", 1.0, GRASP_FRONT, LIFT_HOLD_FRONT, RETURN_FROM_HOLD),
]
