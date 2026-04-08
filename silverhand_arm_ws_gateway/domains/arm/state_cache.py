from __future__ import annotations

from dataclasses import dataclass, field

from .protocol import GROUP_ARM, GROUP_GRIPPER


ARM_JOINT_NAMES = (
    "arm_joint_1",
    "arm_joint_2",
    "arm_joint_3",
    "arm_joint_4",
    "arm_joint_5",
    "arm_joint_6",
)

GRIPPER_JOINT_NAMES = (
    "hand_left_finger_joint",
    "hand_right_finger_joint",
)


@dataclass(slots=True)
class GroupJointState:
    group_name: str
    names: tuple[str, ...]
    positions_rad: list[float]
    velocities_rad_s: list[float]
    pending_positions_rad: list[float] | None = None


@dataclass(slots=True)
class GatewayStateCache:
    groups: dict[str, GroupJointState] = field(default_factory=dict)
    planning_state: str = "idle"
    execution_state: str = "idle"
    estop_active: bool = False

    @classmethod
    def create_default(cls) -> "GatewayStateCache":
        return cls(
            groups={
                GROUP_ARM: GroupJointState(
                    group_name=GROUP_ARM,
                    names=ARM_JOINT_NAMES,
                    positions_rad=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    velocities_rad_s=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                ),
                GROUP_GRIPPER: GroupJointState(
                    group_name=GROUP_GRIPPER,
                    names=GRIPPER_JOINT_NAMES,
                    positions_rad=[0.0, 0.0],
                    velocities_rad_s=[0.0, 0.0],
                ),
            }
        )
