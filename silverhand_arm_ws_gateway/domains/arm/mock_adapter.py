from __future__ import annotations

import asyncio
import contextlib
import logging
import math
from typing import Any

from ...core.adapter_base import EventSink, RobotAdapter
from ...core.config import GatewayConfig
from .protocol import (
    GROUP_ARM,
    GROUP_GRIPPER,
    as_dict,
    as_group_name,
    make_message,
)
from .state_cache import GatewayStateCache, GroupJointState


LOGGER = logging.getLogger("silverhand_arm_ws_gateway.mock")


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def quaternion_to_xyz_euler(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class MockRobotAdapter(RobotAdapter):
    server_name = "silverhand_arm_ws_gateway"
    groups = (GROUP_ARM, GROUP_GRIPPER)

    def __init__(self, config: GatewayConfig) -> None:
        self._config = config
        self._event_sink: EventSink | None = None
        self._state = GatewayStateCache.create_default()
        self._execution_task: asyncio.Task[None] | None = None

    async def start(self, event_sink: EventSink) -> None:
        self._event_sink = event_sink
        LOGGER.info("Mock adapter started")
        await self._publish_full_state()

    async def stop(self) -> None:
        LOGGER.info("Mock adapter stopping")
        if self._execution_task is not None:
            self._execution_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._execution_task
            self._execution_task = None

    async def handle_message(self, message: dict[str, Any]) -> None:
        message_type = message.get("type")
        payload = as_dict(message.get("payload", {}), field_name="payload")

        if message_type == "set_joint_goal":
            await self._handle_set_joint_goal(payload)
            return
        if message_type == "set_pose_goal":
            await self._handle_set_pose_goal(payload)
            return
        if message_type == "plan":
            await self._handle_plan(payload)
            return
        if message_type == "execute":
            await self._handle_execute(payload)
            return
        if message_type == "stop":
            await self._handle_stop()
            return
        if message_type == "estop":
            await self._handle_estop()
            return
        if message_type == "reset_estop":
            await self._handle_reset_estop()
            return

    async def _handle_set_joint_goal(self, payload: dict[str, Any]) -> None:
        goal = self._extract_goal_payload(payload)
        group_name = as_group_name(goal)
        state = self._require_group(group_name)
        positions = goal.get("positions_rad")
        if not isinstance(positions, list) or len(positions) != len(state.positions_rad):
            raise ValueError(f"positions_rad for {group_name!r} must have length {len(state.positions_rad)}")
        state.pending_positions_rad = [float(value) for value in positions]
        LOGGER.info("Accepted joint goal for %s: current=%s target=%s", group_name, _round_list(state.positions_rad), _round_list(state.pending_positions_rad))
        await self._publish_planning_state("goal_set", group_name=group_name, message="Goal accepted.")

    async def _handle_set_pose_goal(self, payload: dict[str, Any]) -> None:
        goal = self._extract_goal_payload(payload)
        group_name = as_group_name(goal)
        if group_name != GROUP_ARM:
            raise ValueError("Pose goals are only supported for group_name='arm' in mock mode.")

        position = as_dict(goal.get("position_m"), field_name="position_m")
        orientation = as_dict(goal.get("orientation_q"), field_name="orientation_q")
        state = self._require_group(GROUP_ARM)
        state.pending_positions_rad = self._mock_pose_to_joint_goal(position, orientation)
        LOGGER.info(
            "Accepted pose goal for arm: position=%s orientation=%s -> target=%s",
            _round_dict(position),
            _round_dict(orientation),
            _round_list(state.pending_positions_rad),
        )
        await self._publish_planning_state("goal_set", group_name=GROUP_ARM, message="Pose goal accepted.")

    async def _handle_plan(self, payload: dict[str, Any]) -> None:
        options = self._extract_options_payload(payload)
        group_name = as_group_name(options)
        self._require_group(group_name)
        LOGGER.info("Planning requested for %s", group_name)
        await self._publish_planning_state("planning", group_name=group_name, message="Planning started.")
        await asyncio.sleep(0.1)
        await self._publish_planning_state("planned", group_name=group_name, message="Plan ready.")

    async def _handle_execute(self, payload: dict[str, Any]) -> None:
        if self._state.estop_active:
            await self._publish_execution_state("estop_active", group_name=payload.get("group_name", GROUP_ARM), message="E-STOP is active.")
            return

        group_name = as_group_name(payload)
        state = self._require_group(group_name)
        if state.pending_positions_rad is None:
            LOGGER.info("Execute requested for %s with no pending goal; returning no-op success", group_name)
            await self._publish_execution_state("succeeded", group_name=group_name, message="No-op execute.")
            return

        if self._execution_task is not None:
            self._execution_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._execution_task

        LOGGER.info("Execute requested for %s: current=%s target=%s", group_name, _round_list(state.positions_rad), _round_list(state.pending_positions_rad))
        self._execution_task = asyncio.create_task(self._execute_group(state))

    async def _handle_stop(self) -> None:
        LOGGER.info("Stop requested")
        if self._execution_task is not None:
            self._execution_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._execution_task
            self._execution_task = None
        for group in self._state.groups.values():
            group.velocities_rad_s = [0.0 for _ in group.velocities_rad_s]
        await self._publish_execution_state("stopped", group_name=GROUP_ARM, message="Motion stopped.")
        await self._publish_joint_states()

    async def _handle_estop(self) -> None:
        self._state.estop_active = True
        LOGGER.warning("Emergency stop requested")
        await self._handle_stop()
        await self._publish_execution_state("estop_active", group_name=GROUP_ARM, message="Emergency stop active.")

    async def _handle_reset_estop(self) -> None:
        self._state.estop_active = False
        LOGGER.info("Emergency stop reset")
        await self._publish_execution_state("idle", group_name=GROUP_ARM, message="Emergency stop reset.")

    async def _execute_group(self, state: GroupJointState) -> None:
        assert state.pending_positions_rad is not None
        target = list(state.pending_positions_rad)
        dt = self._config.mock_step_s
        max_step = max(self._config.mock_max_joint_velocity_rad_s * dt, 1e-4)

        await self._publish_execution_state("executing", group_name=state.group_name, message="Executing target.")
        max_iterations = max(self._config.mock_steps_per_execute * 20, 1)
        for iteration in range(max_iterations):
            deltas = [target_value - current_value for target_value, current_value in zip(target, state.positions_rad)]
            if all(abs(delta) <= 1e-4 for delta in deltas):
                break

            new_positions: list[float] = []
            velocities: list[float] = []
            for current_value, delta in zip(state.positions_rad, deltas):
                step = clamp(delta, -max_step, max_step)
                next_value = current_value + step
                new_positions.append(next_value)
                velocities.append(step / dt)

            state.positions_rad = new_positions
            state.velocities_rad_s = velocities
            if iteration == 0 or iteration % 10 == 0:
                LOGGER.info(
                    "Progress %s: positions=%s remaining=%s",
                    state.group_name,
                    _round_list(state.positions_rad),
                    _round_list(deltas),
                )
            await self._publish_joint_state(state)
            await asyncio.sleep(dt)

        state.positions_rad = target
        state.velocities_rad_s = [0.0 for _ in target]
        state.pending_positions_rad = None
        await self._publish_joint_state(state)
        LOGGER.info("Target reached for %s: final=%s", state.group_name, _round_list(state.positions_rad))
        await self._publish_execution_state("succeeded", group_name=state.group_name, message="Target reached.")
        self._execution_task = None

    def _mock_pose_to_joint_goal(self, position: dict[str, Any], orientation: dict[str, Any]) -> list[float]:
        x = float(position.get("x", 0.0))
        y = float(position.get("y", 0.0))
        z = float(position.get("z", 0.0))
        qx = float(orientation.get("x", 0.0))
        qy = float(orientation.get("y", 0.0))
        qz = float(orientation.get("z", 0.0))
        qw = float(orientation.get("w", 1.0))

        radius = math.hypot(x, y)
        roll, pitch, yaw = quaternion_to_xyz_euler(qx, qy, qz, qw)

        q1 = math.atan2(y, x) if radius > 1e-6 else 0.0
        q2 = clamp((z - 0.18) * 4.0, -2.2, 2.2)
        q3 = clamp((0.42 - radius) * 5.0, -2.0, 2.0)
        q4 = clamp(roll, -math.pi, math.pi)
        q5 = clamp(pitch + math.pi / 2.0, -math.pi, math.pi)
        q6 = clamp(yaw, -math.pi, math.pi)
        return [q1, q2, q3, q4, q5, q6]

    def _require_group(self, group_name: str) -> GroupJointState:
        state = self._state.groups.get(group_name)
        if state is None:
            raise ValueError(f"Unknown group {group_name!r}")
        return state

    async def _publish_full_state(self) -> None:
        await self._publish_planning_state("idle", group_name=GROUP_ARM, message="Idle.")
        await self._publish_execution_state("idle", group_name=GROUP_ARM, message="Idle.")
        await self._publish_joint_states()

    async def _publish_joint_states(self) -> None:
        for state in self._state.groups.values():
            await self._publish_joint_state(state)

    async def _publish_joint_state(self, state: GroupJointState) -> None:
        LOGGER.debug("Emit joint_state %s: positions=%s velocities=%s", state.group_name, _round_list(state.positions_rad), _round_list(state.velocities_rad_s))
        await self._emit(
            make_message(
                "joint_state",
                {
                    "group_name": state.group_name,
                    "name": list(state.names),
                    "position_rad": list(state.positions_rad),
                    "velocity_rad_s": list(state.velocities_rad_s),
                },
            )
        )

    async def _publish_planning_state(self, status: str, *, group_name: str, message: str) -> None:
        self._state.planning_state = status
        LOGGER.info("Planning state %s -> %s (%s)", group_name, status, message)
        await self._emit(
            make_message(
                "planning_state",
                {
                    "command_id": "mock",
                    "group_name": group_name,
                    "status": status,
                    "message": message,
                },
            )
        )

    async def _publish_execution_state(self, status: str, *, group_name: str, message: str) -> None:
        self._state.execution_state = status
        LOGGER.info("Execution state %s -> %s (%s)", group_name, status, message)
        await self._emit(
            make_message(
                "execution_state",
                {
                    "command_id": "mock",
                    "group_name": group_name,
                    "status": status,
                    "message": message,
                },
            )
        )

    def _extract_goal_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        nested_goal = payload.get("goal")
        if isinstance(nested_goal, dict):
            return nested_goal
        return payload

    def _extract_options_payload(self, payload: dict[str, Any]) -> dict[str, Any]:
        nested_options = payload.get("options")
        if isinstance(nested_options, dict):
            return nested_options
        return payload

    async def _emit(self, message: dict[str, Any]) -> None:
        if self._event_sink is None:
            return
        await self._event_sink(message)


def _round_list(values: list[float], digits: int = 3) -> list[float]:
    return [round(value, digits) for value in values]


def _round_dict(values: dict[str, Any], digits: int = 3) -> dict[str, Any]:
    rounded: dict[str, Any] = {}
    for key, value in values.items():
        rounded[key] = round(float(value), digits) if isinstance(value, (int, float)) else value
    return rounded
