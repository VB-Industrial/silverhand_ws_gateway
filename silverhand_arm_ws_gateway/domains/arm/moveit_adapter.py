from __future__ import annotations

import asyncio
import contextlib
import logging
import threading
from collections.abc import Callable
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from sensor_msgs.msg import JointState

from ...core.adapter_base import EventSink, RobotAdapter
from ...core.config import GatewayConfig
from .protocol import (
    GROUP_ARM,
    GROUP_GRIPPER,
    as_dict,
    as_group_name,
    make_fault_state,
    make_message,
)
from .state_cache import ARM_JOINT_NAMES, GRIPPER_JOINT_NAMES


LOGGER = logging.getLogger("silverhand_arm_ws_gateway.moveit")

ROS_ARM_JOINT_NAMES = (
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
)

WS_ARM_JOINT_NAMES = ARM_JOINT_NAMES
ROS_GRIPPER_JOINT_NAMES = GRIPPER_JOINT_NAMES

WS_TO_ROS_ARM_JOINT = dict(zip(WS_ARM_JOINT_NAMES, ROS_ARM_JOINT_NAMES, strict=True))
ROS_TO_WS_ARM_JOINT = dict(zip(ROS_ARM_JOINT_NAMES, WS_ARM_JOINT_NAMES, strict=True))

MOVEIT_GROUP_BY_WS_GROUP = {
    GROUP_ARM: "silverhand_arm",
    GROUP_GRIPPER: "silverhand_hand",
}

JOINT_STATE_TOPIC = "/joint_states"


class MoveItRobotAdapter(RobotAdapter):
    server_name = "silverhand_arm_ws_gateway"
    groups = (GROUP_ARM, GROUP_GRIPPER)

    def __init__(self, config: GatewayConfig) -> None:
        self._config = config
        self._event_sink: EventSink | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._node: Node | None = None
        self._action_client: ActionClient | None = None
        self._spin_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._rclpy_owned = False
        self._state_lock = threading.Lock()
        self._arm_positions_by_ros_name: dict[str, float] = {name: 0.0 for name in ROS_ARM_JOINT_NAMES}
        self._arm_velocities_by_ros_name: dict[str, float] = {name: 0.0 for name in ROS_ARM_JOINT_NAMES}
        self._gripper_positions_by_ros_name: dict[str, float] = {name: 0.0 for name in ROS_GRIPPER_JOINT_NAMES}
        self._gripper_velocities_by_ros_name: dict[str, float] = {name: 0.0 for name in ROS_GRIPPER_JOINT_NAMES}
        self._pending_goals: dict[str, list[float] | None] = {
            GROUP_ARM: None,
            GROUP_GRIPPER: None,
        }
        self._active_goal_handle: ClientGoalHandle | None = None
        self._active_group_name: str | None = None
        self._estop_active = False

    async def start(self, event_sink: EventSink) -> None:
        self._event_sink = event_sink
        self._loop = asyncio.get_running_loop()

        if not rclpy.ok():
            rclpy.init(args=None)
            self._rclpy_owned = True

        self._node = rclpy.create_node("silverhand_arm_ws_gateway_moveit")
        self._node.create_subscription(JointState, JOINT_STATE_TOPIC, self._on_joint_state, 10)
        self._action_client = ActionClient(self._node, MoveGroup, self._config.move_group_action_name)

        self._spin_thread = threading.Thread(target=self._spin_worker, name="silverhand_ws_gateway_moveit_spin", daemon=True)
        self._spin_thread.start()

        await self._publish_planning_state("idle", group_name=GROUP_ARM, message="Idle.")
        await self._publish_execution_state("idle", group_name=GROUP_ARM, message="Idle.")
        await self._publish_joint_state_arm()
        await self._publish_joint_state_gripper()

        action_ready = await self._wait_for_action_server(2.0)
        if action_ready:
            await self._emit(
                make_fault_state(
                    "moveit_adapter_ready",
                    f"MoveIt adapter connected to {self._config.move_group_action_name}.",
                    severity="info",
                    active=False,
                )
            )
        else:
            await self._emit(
                make_fault_state(
                    "move_action_unavailable",
                    f"MoveIt action server {self._config.move_group_action_name} is unavailable.",
                    severity="warning",
                    active=False,
                )
            )

    async def stop(self) -> None:
        self._stop_event.set()
        await self._cancel_active_goal()

        if self._spin_thread is not None:
            self._spin_thread.join(timeout=1.0)
            self._spin_thread = None

        if self._node is not None:
            self._node.destroy_node()
            self._node = None

        self._action_client = None

        if self._rclpy_owned and rclpy.ok():
            rclpy.shutdown()
            self._rclpy_owned = False

    async def handle_message(self, message: dict[str, Any]) -> None:
        message_type = message.get("type")
        payload = as_dict(message.get("payload", {}), field_name="payload")

        if message_type == "set_joint_goal":
            await self._handle_set_joint_goal(payload)
            return
        if message_type == "set_pose_goal":
            await self._emit_unsupported_pose_goal()
            return
        if message_type == "plan":
            await self._handle_plan(payload)
            return
        if message_type == "execute":
            await self._handle_execute(payload)
            return
        if message_type == "stop":
            await self._handle_stop(payload)
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
        joint_names = goal.get("joint_names")
        positions_rad = goal.get("positions_rad")
        if not isinstance(joint_names, list) or not isinstance(positions_rad, list):
            raise ValueError("goal.joint_names and goal.positions_rad must be arrays.")
        if len(joint_names) != len(positions_rad):
            raise ValueError("goal.joint_names and goal.positions_rad must have the same length.")

        mapped_positions = self._map_ws_joint_goal(group_name, joint_names, positions_rad)
        with self._state_lock:
            self._pending_goals[group_name] = mapped_positions

        LOGGER.info("Accepted MoveIt joint goal for %s: target=%s", group_name, _round_list(mapped_positions))
        await self._publish_planning_state("goal_set", group_name=group_name, message="Goal accepted.")

    async def _handle_plan(self, payload: dict[str, Any]) -> None:
        options = self._extract_options_payload(payload)
        group_name = as_group_name(options)
        goal = self._get_pending_goal(group_name)
        if goal is None:
            await self._publish_planning_state("failed", group_name=group_name, message="No pending goal to plan.")
            return
        await self._run_move_group(group_name, goal, plan_only=True)

    async def _handle_execute(self, payload: dict[str, Any]) -> None:
        group_name = as_group_name(payload)
        if self._estop_active:
            await self._publish_execution_state("estop_active", group_name=group_name, message="Emergency stop active.")
            return

        goal = self._get_pending_goal(group_name)
        if goal is None:
            await self._publish_execution_state("succeeded", group_name=group_name, message="No-op execute.")
            return
        await self._run_move_group(group_name, goal, plan_only=False)

    async def _handle_stop(self, payload: dict[str, Any]) -> None:
        group_name = as_group_name(payload)
        await self._cancel_active_goal()
        await self._publish_execution_state("stopped", group_name=group_name, message="Motion stopped.")

    async def _handle_estop(self) -> None:
        self._estop_active = True
        await self._cancel_active_goal()
        await self._publish_execution_state("estop_active", group_name=GROUP_ARM, message="Emergency stop active.")

    async def _handle_reset_estop(self) -> None:
        self._estop_active = False
        await self._publish_execution_state("idle", group_name=GROUP_ARM, message="Emergency stop reset.")

    async def _emit_unsupported_pose_goal(self) -> None:
        await self._emit(
            make_fault_state(
                "pose_goal_unmapped",
                "Pose goals are not mapped in moveit_adapter yet. Send joint goals instead.",
                severity="warning",
                active=False,
            )
        )
        await self._publish_execution_state("failed", group_name=GROUP_ARM, message="Pose goals are not mapped yet.")

    async def _run_move_group(self, group_name: str, target_positions: list[float], *, plan_only: bool) -> None:
        if self._action_client is None:
            await self._publish_execution_state("failed", group_name=group_name, message="MoveIt action client is not ready.")
            return

        action_ready = await self._wait_for_action_server(1.0)
        if not action_ready:
            await self._publish_execution_state("failed", group_name=group_name, message="MoveIt action server unavailable.")
            return

        await self._cancel_active_goal()

        goal_message = self._build_move_group_goal(group_name, target_positions, plan_only=plan_only)
        if plan_only:
            await self._publish_planning_state("planning", group_name=group_name, message="MoveIt planning started.")
        else:
            await self._publish_execution_state("executing", group_name=group_name, message="MoveIt plan+execute started.")

        LOGGER.info(
            "Sending MoveGroup goal for %s: target=%s plan_only=%s",
            group_name,
            _round_list(target_positions),
            plan_only,
        )

        send_goal_future = self._action_client.send_goal_async(goal_message)
        goal_handle = await self._await_rclpy_future(send_goal_future)
        if goal_handle is None or not goal_handle.accepted:
            LOGGER.warning("MoveGroup goal rejected for %s (plan_only=%s)", group_name, plan_only)
            if plan_only:
                await self._publish_planning_state("failed", group_name=group_name, message="MoveIt goal rejected.")
            else:
                await self._publish_execution_state("failed", group_name=group_name, message="MoveIt goal rejected.")
            return

        LOGGER.info("MoveGroup goal accepted for %s (plan_only=%s)", group_name, plan_only)
        self._active_goal_handle = goal_handle
        self._active_group_name = group_name
        asyncio.create_task(self._wait_for_goal_result(goal_handle, group_name, plan_only=plan_only))

    def _build_move_group_goal(self, group_name: str, target_positions: list[float], *, plan_only: bool) -> MoveGroup.Goal:
        moveit_group_name = MOVEIT_GROUP_BY_WS_GROUP[group_name]
        ros_joint_names = self._ros_joint_names_for_group(group_name)
        constraints = Constraints()
        constraints.name = f"{group_name}_joint_goal"
        constraints.joint_constraints = []
        for joint_name, target_position in zip(ros_joint_names, target_positions, strict=True):
            constraint = JointConstraint()
            constraint.joint_name = joint_name
            constraint.position = float(target_position)
            constraint.tolerance_above = 1e-3
            constraint.tolerance_below = 1e-3
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)

        goal = MoveGroup.Goal()
        goal.request.group_name = moveit_group_name
        goal.request.num_planning_attempts = 1
        goal.request.allowed_planning_time = 2.0
        goal.request.max_velocity_scaling_factor = 1.0
        goal.request.max_acceleration_scaling_factor = 1.0
        goal.request.goal_constraints = [constraints]
        # Let MoveIt use the current monitored robot state as the start state.
        # Manually injecting a sampled JointState here caused intermittent
        # START_STATE_INVALID / out-of-bounds failures in combined plan+execute.
        goal.request.start_state.is_diff = True
        goal.planning_options.plan_only = plan_only
        goal.planning_options.replan = False
        return goal

    def _spin_worker(self) -> None:
        assert self._node is not None
        while rclpy.ok() and not self._stop_event.is_set():
            rclpy.spin_once(self._node, timeout_sec=0.1)

    def _on_joint_state(self, message: JointState) -> None:
        names = list(message.name)
        if not names:
            return

        arm_positions: dict[str, float] = {}
        arm_velocities: dict[str, float] = {}
        gripper_positions: dict[str, float] = {}
        gripper_velocities: dict[str, float] = {}

        for index, name in enumerate(names):
            if name in ROS_TO_WS_ARM_JOINT:
                ws_name = ROS_TO_WS_ARM_JOINT[name]
                arm_positions[ws_name] = float(message.position[index]) if index < len(message.position) else 0.0
                arm_velocities[ws_name] = float(message.velocity[index]) if index < len(message.velocity) else 0.0
                continue
            if name in ROS_GRIPPER_JOINT_NAMES:
                gripper_positions[name] = float(message.position[index]) if index < len(message.position) else 0.0
                gripper_velocities[name] = float(message.velocity[index]) if index < len(message.velocity) else 0.0

        with self._state_lock:
            for ros_name, ws_name in ROS_TO_WS_ARM_JOINT.items():
                if ws_name in arm_positions:
                    self._arm_positions_by_ros_name[ros_name] = arm_positions[ws_name]
                    self._arm_velocities_by_ros_name[ros_name] = arm_velocities.get(ws_name, 0.0)
            for ros_name in ROS_GRIPPER_JOINT_NAMES:
                if ros_name in gripper_positions:
                    self._gripper_positions_by_ros_name[ros_name] = gripper_positions[ros_name]
                    self._gripper_velocities_by_ros_name[ros_name] = gripper_velocities.get(ros_name, 0.0)

        if arm_positions:
            self._emit_from_thread(
                make_message(
                    "joint_state",
                    {
                        "group_name": GROUP_ARM,
                        "name": list(WS_ARM_JOINT_NAMES),
                        "position_rad": [arm_positions.get(name, 0.0) for name in WS_ARM_JOINT_NAMES],
                        "velocity_rad_s": [arm_velocities.get(name, 0.0) for name in WS_ARM_JOINT_NAMES],
                    },
                )
            )

        if gripper_positions:
            self._emit_from_thread(
                make_message(
                    "joint_state",
                    {
                        "group_name": GROUP_GRIPPER,
                        "name": list(ROS_GRIPPER_JOINT_NAMES),
                        "position_rad": [gripper_positions.get(name, 0.0) for name in ROS_GRIPPER_JOINT_NAMES],
                        "velocity_rad_s": [gripper_velocities.get(name, 0.0) for name in ROS_GRIPPER_JOINT_NAMES],
                    },
                )
            )

    async def _wait_for_goal_result(self, goal_handle: ClientGoalHandle, group_name: str, *, plan_only: bool) -> None:
        try:
            result_future = goal_handle.get_result_async()
            wrapped = await self._await_rclpy_future(result_future)
            result = wrapped.result
            status = wrapped.status
            self._active_goal_handle = None
            self._active_group_name = None

            planning_time = getattr(result, "planning_time", None)
            error_code_val = getattr(result.error_code, "val", None)
            error_message = result.error_code.message or _moveit_error_name(error_code_val)
            LOGGER.info(
                "MoveGroup result for %s: status=%s error_code=%s message=%s plan_only=%s planning_time=%s",
                group_name,
                status,
                error_code_val,
                error_message,
                plan_only,
                f"{planning_time:.3f}s" if isinstance(planning_time, (int, float)) else "n/a",
            )

            if status == GoalStatus.STATUS_CANCELED:
                if plan_only:
                    await self._publish_planning_state("failed", group_name=group_name, message="MoveIt goal canceled.")
                else:
                    await self._publish_execution_state("stopped", group_name=group_name, message="MoveIt goal canceled.")
                return

            if status == GoalStatus.STATUS_SUCCEEDED and result.error_code.val == MoveItErrorCodes.SUCCESS:
                if plan_only:
                    LOGGER.info("MoveGroup planning succeeded for %s in %.3fs", group_name, result.planning_time)
                    await self._publish_planning_state(
                        "planned",
                        group_name=group_name,
                        message=f"Plan ready in {result.planning_time:.3f}s.",
                    )
                else:
                    with self._state_lock:
                        self._pending_goals[group_name] = None
                    LOGGER.info("MoveGroup execution succeeded for %s", group_name)
                    await self._publish_execution_state("succeeded", group_name=group_name, message="Target reached.")
                return

            if plan_only:
                await self._publish_planning_state("failed", group_name=group_name, message=error_message)
            else:
                await self._publish_execution_state("failed", group_name=group_name, message=error_message)
        except Exception as exc:  # noqa: BLE001
            LOGGER.exception("Failed while waiting for MoveIt result")
            if plan_only:
                await self._publish_planning_state("failed", group_name=group_name, message=str(exc))
            else:
                await self._publish_execution_state("failed", group_name=group_name, message=str(exc))

    async def _cancel_active_goal(self) -> None:
        goal_handle = self._active_goal_handle
        if goal_handle is None:
            return
        if goal_handle.status not in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING):
            self._active_goal_handle = None
            self._active_group_name = None
            return

        try:
            cancel_future = goal_handle.cancel_goal_async()
            await self._await_rclpy_future(cancel_future)
        except Exception:  # noqa: BLE001
            LOGGER.exception("Failed to cancel active MoveIt goal")
        finally:
            self._active_goal_handle = None
            self._active_group_name = None

    async def _wait_for_action_server(self, timeout_s: float) -> bool:
        action_client = self._action_client
        node = self._node
        if action_client is None or node is None or self._stop_event.is_set():
            return False
        try:
            return bool(await asyncio.to_thread(action_client.wait_for_server, timeout_s))
        except Exception as exc:  # noqa: BLE001
            LOGGER.warning("Failed to check if action server is available: %s", exc)
            return False

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

    def _map_ws_joint_goal(self, group_name: str, joint_names: list[Any], positions_rad: list[Any]) -> list[float]:
        if group_name == GROUP_ARM:
            index_by_name = {str(name): idx for idx, name in enumerate(joint_names)}
            return [float(positions_rad[index_by_name[ws_name]]) for ws_name in WS_ARM_JOINT_NAMES]

        if group_name == GROUP_GRIPPER:
            index_by_name = {str(name): idx for idx, name in enumerate(joint_names)}
            return [float(positions_rad[index_by_name[name]]) for name in ROS_GRIPPER_JOINT_NAMES]

        raise ValueError(f"Unsupported group_name: {group_name!r}")

    def _ros_joint_names_for_group(self, group_name: str) -> list[str]:
        if group_name == GROUP_ARM:
            return list(ROS_ARM_JOINT_NAMES)
        if group_name == GROUP_GRIPPER:
            return list(ROS_GRIPPER_JOINT_NAMES)
        raise ValueError(f"Unsupported group_name: {group_name!r}")

    def _get_pending_goal(self, group_name: str) -> list[float] | None:
        with self._state_lock:
            pending = self._pending_goals.get(group_name)
            return list(pending) if pending is not None else None

    async def _publish_joint_state_arm(self) -> None:
        with self._state_lock:
            positions = [self._arm_positions_by_ros_name[name] for name in ROS_ARM_JOINT_NAMES]
            velocities = [self._arm_velocities_by_ros_name[name] for name in ROS_ARM_JOINT_NAMES]
        await self._emit(
            make_message(
                "joint_state",
                {
                    "group_name": GROUP_ARM,
                    "name": list(WS_ARM_JOINT_NAMES),
                    "position_rad": positions,
                    "velocity_rad_s": velocities,
                },
            )
        )

    async def _publish_joint_state_gripper(self) -> None:
        with self._state_lock:
            positions = [self._gripper_positions_by_ros_name[name] for name in ROS_GRIPPER_JOINT_NAMES]
            velocities = [self._gripper_velocities_by_ros_name[name] for name in ROS_GRIPPER_JOINT_NAMES]
        await self._emit(
            make_message(
                "joint_state",
                {
                    "group_name": GROUP_GRIPPER,
                    "name": list(ROS_GRIPPER_JOINT_NAMES),
                    "position_rad": positions,
                    "velocity_rad_s": velocities,
                },
            )
        )

    async def _publish_planning_state(self, status: str, *, group_name: str, message: str) -> None:
        await self._emit(
            make_message(
                "planning_state",
                {
                    "command_id": "moveit",
                    "group_name": group_name,
                    "status": status,
                    "message": message,
                },
            )
        )

    async def _publish_execution_state(self, status: str, *, group_name: str, message: str) -> None:
        await self._emit(
            make_message(
                "execution_state",
                {
                    "command_id": "moveit",
                    "group_name": group_name,
                    "status": status,
                    "message": message,
                },
            )
        )

    async def _emit(self, message: dict[str, Any]) -> None:
        if self._event_sink is None:
            return
        await self._event_sink(message)

    def _emit_from_thread(self, message: dict[str, Any]) -> None:
        if self._event_sink is None or self._loop is None:
            return

        async def emit_message() -> None:
            await self._event_sink(message)

        asyncio.run_coroutine_threadsafe(emit_message(), self._loop)

    async def _await_rclpy_future(self, future: Any) -> Any:
        wrapped: asyncio.Future[Any] = asyncio.get_running_loop().create_future()

        def _complete(done_future: Any) -> None:
            try:
                result = done_future.result()
                asyncio.get_running_loop()
            except RuntimeError:
                pass
            if wrapped.cancelled():
                return
            try:
                result = done_future.result()
            except Exception as exc:  # noqa: BLE001
                self._loop.call_soon_threadsafe(wrapped.set_exception, exc)
            else:
                self._loop.call_soon_threadsafe(wrapped.set_result, result)

        future.add_done_callback(_complete)
        return await wrapped


def _round_list(values: list[float], digits: int = 3) -> list[float]:
    return [round(value, digits) for value in values]


def _moveit_error_name(code: int) -> str:
    for name, value in MoveItErrorCodes.__dict__.items():
        if name.isupper() and isinstance(value, int) and value == code:
            return name
    return f"MoveIt error {code}"
