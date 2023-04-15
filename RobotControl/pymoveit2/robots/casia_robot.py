from typing import List

MOVE_GROUP_ARM: str = "arm_group"
MOVE_GROUP_GRIPPER: str = "hand_group"

MAX_OPEN_JOINT = -37.0 / 180 * 3.141592653
OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [MAX_OPEN_JOINT, MAX_OPEN_JOINT]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.0, 0.0]


def joint_names(prefix: str = "") -> List[str]:
    return [
        prefix + "joint1",
        prefix + "joint2",
        prefix + "joint3",
        prefix + "joint4",
        prefix + "joint5",
        prefix + "joint6",
    ]


def base_link_name(prefix: str = "") -> str:
    return prefix + "dummy_link"


def end_effector_name(prefix: str = "") -> str:
    return prefix + "hand_tcp"


def gripper_joint_names(prefix: str = "") -> List[str]:
    return [
        prefix + "dh_gripper_finger1_joint"
    ]
