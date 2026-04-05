from __future__ import annotations

import asyncio
import contextlib
import logging
import threading
from collections.abc import Callable
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from .adapter_base import EventSink, RobotAdapter
from .config import GatewayConfig
from .protocol import (
    GROUP_ARM,
    GROUP_GRIPPER,
    as_dict,
    as_group_name,
    make_fault_state,
    make_message,
)


LOGGER = logging.getLogger("silverhand_arm_ws_gateway.ros")

ROS_ARM_JOINT_NAMES = (
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
)

WS_ARM_JOINT_NAMES = (
    "arm_joint_1",
    "arm_joint_2",
    "arm_joint_3",
    "arm_joint_4",
    "arm_joint_5",
    "arm_joint_6",
)

WS_TO_ROS_ARM_JOINT = dict(zip(WS_ARM_JOINT_NAMES, ROS_ARM_JOINT_NAMES, strict=True))
ROS_TO_WS_ARM_JOINT = dict(zip(ROS_ARM_JOINT_NAMES, WS_ARM_JOINT_NAMES, strict=True))

ARM_ACTION_NAME = "/arm_controller/follow_joint_trajectory"
JOINT_STATE_TOPIC = "/joint_states"


class RosRobotAdapter(RobotAdapter):
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
        self._pending_arm_goal: list[float] | None = None
        self._active_goal_handle: ClientGoalHandle | None = None
        self._estop_active = False

    async def start(self, event_sink: EventSink) -> None:
        self._event_sink = event_sink
        self._loop = asyncio.get_running_loop()

        if not rclpy.ok():
            rclpy.init(args=None)
            self._rclpy_owned = True

        self._node = rclpy.create_node("silverhand_arm_ws_gateway")
        self._node.create_subscription(JointState, JOINT_STATE_TOPIC, self._on_joint_state, 10)
        self._action_client = ActionClient(self._node, FollowJointTrajectory, ARM_ACTION_NAME)

        self._spin_thread = threading.Thread(target=self._spin_worker, name="silverhand_ws_gateway_ros_spin", daemon=True)
        self._spin_thread.start()

        await self._publish_planning_state("idle", group_name=GROUP_ARM, message="Idle.")
        await self._publish_execution_state("idle", group_name=GROUP_ARM, message="Idle.")
        await self._publish_joint_state_arm()

        action_ready = await asyncio.to_thread(self._action_client.wait_for_server, 2.0)
        if action_ready:
            await self._emit(
                make_fault_state(
                    "ros_adapter_ready",
                    "ROS adapter connected to arm_controller.",
                    severity="info",
                    active=False,
                )
            )
        else:
            await self._emit(
                make_fault_state(
                    "arm_controller_unavailable",
                    "Action server /arm_controller/follow_joint_trajectory is unavailable.",
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

        if group_name == GROUP_GRIPPER:
            await self._emit(
                make_fault_state(
                    "gripper_unmapped",
                    "Gripper is not mapped into ros2_control yet.",
                    severity="warning",
                    active=False,
                )
            )
            await self._publish_execution_state("failed", group_name=GROUP_GRIPPER, message="Gripper is not mapped yet.")
            return

        joint_names = goal.get("joint_names")
        positions_rad = goal.get("positions_rad")
        if not isinstance(joint_names, list) or not isinstance(positions_rad, list):
            raise ValueError("goal.joint_names and goal.positions_rad must be arrays.")
        if len(joint_names) != len(positions_rad):
            raise ValueError("goal.joint_names and goal.positions_rad must have the same length.")

        mapped_positions = self._map_ws_joint_goal_to_ros(joint_names, positions_rad)
        with self._state_lock:
            self._pending_arm_goal = mapped_positions

        LOGGER.info("Accepted ROS joint goal arm: target=%s", _round_list(mapped_positions))
        await self._publish_planning_state("goal_set", group_name=GROUP_ARM, message="Goal accepted.")

    async def _handle_plan(self, payload: dict[str, Any]) -> None:
        options = self._extract_options_payload(payload)
        group_name = as_group_name(options)
        if group_name == GROUP_GRIPPER:
            await self._publish_planning_state("planned", group_name=GROUP_GRIPPER, message="Gripper planning is a no-op.")
            return

        await self._publish_planning_state("planning", group_name=GROUP_ARM, message="Planning started.")
        await asyncio.sleep(0.05)
        await self._publish_planning_state("planned", group_name=GROUP_ARM, message="Plan ready.")

    async def _handle_execute(self, payload: dict[str, Any]) -> None:
        group_name = as_group_name(payload)
        if self._estop_active:
            await self._publish_execution_state("estop_active", group_name=group_name, message="Emergency stop active.")
            return

        if group_name == GROUP_GRIPPER:
            await self._publish_execution_state("failed", group_name=GROUP_GRIPPER, message="Gripper is not mapped yet.")
            return

        with self._state_lock:
            pending_goal = list(self._pending_arm_goal) if self._pending_arm_goal is not None else None
        if pending_goal is None:
            await self._publish_execution_state("succeeded", group_name=GROUP_ARM, message="No-op execute.")
            return

        if self._action_client is None:
            await self._publish_execution_state("failed", group_name=GROUP_ARM, message="Action client is not ready.")
            return

        action_ready = await asyncio.to_thread(self._action_client.wait_for_server, 1.0)
        if not action_ready:
            await self._publish_execution_state("failed", group_name=GROUP_ARM, message="arm_controller action server unavailable.")
            return

        await self._cancel_active_goal()

        goal_message = FollowJointTrajectory.Goal()
        goal_message.trajectory.joint_names = list(ROS_ARM_JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = list(pending_goal)
        point.velocities = [0.0 for _ in pending_goal]
        duration_s = self._estimate_execution_duration_s(pending_goal)
        point.time_from_start.sec = int(duration_s)
        point.time_from_start.nanosec = int((duration_s - int(duration_s)) * 1_000_000_000)
        goal_message.trajectory.points = [point]

        LOGGER.info("Sending FollowJointTrajectory goal: target=%s duration=%.2fs", _round_list(pending_goal), duration_s)
        await self._publish_execution_state("executing", group_name=GROUP_ARM, message="Trajectory sent.")

        send_goal_future = self._action_client.send_goal_async(goal_message)
        goal_handle = await self._await_rclpy_future(send_goal_future)
        if goal_handle is None or not goal_handle.accepted:
            await self._publish_execution_state("failed", group_name=GROUP_ARM, message="Trajectory goal rejected.")
            return

        self._active_goal_handle = goal_handle
        asyncio.create_task(self._wait_for_goal_result(goal_handle))

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
                "Pose goals are not mapped in ros_adapter yet. Send joint goals instead.",
                severity="warning",
                active=False,
            )
        )
        await self._publish_execution_state("failed", group_name=GROUP_ARM, message="Pose goals are not mapped yet.")

    def _spin_worker(self) -> None:
        assert self._node is not None
        while rclpy.ok() and not self._stop_event.is_set():
            rclpy.spin_once(self._node, timeout_sec=0.1)

    def _on_joint_state(self, message: JointState) -> None:
        names = list(message.name)
        if not names:
            return

        mapped_positions: dict[str, float] = {}
        mapped_velocities: dict[str, float] = {}
        for index, name in enumerate(names):
            if name not in ROS_TO_WS_ARM_JOINT:
                continue
            ws_name = ROS_TO_WS_ARM_JOINT[name]
            mapped_positions[ws_name] = float(message.position[index]) if index < len(message.position) else 0.0
            mapped_velocities[ws_name] = float(message.velocity[index]) if index < len(message.velocity) else 0.0

        if not mapped_positions:
            return

        with self._state_lock:
            for ros_name, ws_name in ROS_TO_WS_ARM_JOINT.items():
                self._arm_positions_by_ros_name[ros_name] = mapped_positions.get(ws_name, self._arm_positions_by_ros_name[ros_name])
                self._arm_velocities_by_ros_name[ros_name] = mapped_velocities.get(ws_name, self._arm_velocities_by_ros_name[ros_name])

        self._emit_from_thread(
            make_message(
                "joint_state",
                {
                    "group_name": GROUP_ARM,
                    "name": list(WS_ARM_JOINT_NAMES),
                    "position_rad": [mapped_positions.get(name, 0.0) for name in WS_ARM_JOINT_NAMES],
                    "velocity_rad_s": [mapped_velocities.get(name, 0.0) for name in WS_ARM_JOINT_NAMES],
                },
            )
        )

    async def _wait_for_goal_result(self, goal_handle: ClientGoalHandle) -> None:
        try:
            result_future = goal_handle.get_result_async()
            wrapped = await self._await_rclpy_future(result_future)
            result = wrapped.result
            status = wrapped.status
            self._active_goal_handle = None

            if status == GoalStatus.STATUS_SUCCEEDED and result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                with self._state_lock:
                    self._pending_arm_goal = None
                await self._publish_execution_state("succeeded", group_name=GROUP_ARM, message="Target reached.")
                return

            if status == GoalStatus.STATUS_CANCELED:
                await self._publish_execution_state("stopped", group_name=GROUP_ARM, message="Goal canceled.")
                return

            await self._publish_execution_state(
                "failed",
                group_name=GROUP_ARM,
                message=result.error_string or f"Action failed with status {status} and error_code {result.error_code}.",
            )
        except Exception as exc:  # noqa: BLE001
            LOGGER.exception("Failed while waiting for action result")
            await self._publish_execution_state("failed", group_name=GROUP_ARM, message=str(exc))

    async def _cancel_active_goal(self) -> None:
        goal_handle = self._active_goal_handle
        if goal_handle is None:
            return
        if goal_handle.status not in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING):
            self._active_goal_handle = None
            return

        try:
            cancel_future = goal_handle.cancel_goal_async()
            await self._await_rclpy_future(cancel_future)
        except Exception:  # noqa: BLE001
            LOGGER.exception("Failed to cancel active goal")
        finally:
            self._active_goal_handle = None

    def _estimate_execution_duration_s(self, target_positions: list[float]) -> float:
        with self._state_lock:
            current_positions = [self._arm_positions_by_ros_name[name] for name in ROS_ARM_JOINT_NAMES]
        max_delta = max((abs(target - current) for target, current in zip(target_positions, current_positions, strict=True)), default=0.0)
        return max(1.0, min(8.0, (max_delta / 0.8) + 0.5))

    def _map_ws_joint_goal_to_ros(self, joint_names: list[Any], positions_rad: list[Any]) -> list[float]:
        mapped_by_ros_name: dict[str, float] = {}
        for index, raw_name in enumerate(joint_names):
            if not isinstance(raw_name, str):
                raise ValueError("joint_names entries must be strings.")
            position = float(positions_rad[index])
            ros_name = WS_TO_ROS_ARM_JOINT.get(raw_name, raw_name)
            if ros_name not in ROS_ARM_JOINT_NAMES:
                raise ValueError(f"Unsupported arm joint name: {raw_name!r}")
            mapped_by_ros_name[ros_name] = position

        missing = [name for name in ROS_ARM_JOINT_NAMES if name not in mapped_by_ros_name]
        if missing:
            raise ValueError(f"Joint goal is missing arm joints: {missing}")
        return [mapped_by_ros_name[name] for name in ROS_ARM_JOINT_NAMES]

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

    async def _publish_planning_state(self, status: str, *, group_name: str, message: str) -> None:
        LOGGER.info("Planning state %s -> %s (%s)", group_name, status, message)
        await self._emit(
            make_message(
                "planning_state",
                {
                    "command_id": "ros",
                    "group_name": group_name,
                    "status": status,
                    "message": message,
                },
            )
        )

    async def _publish_execution_state(self, status: str, *, group_name: str, message: str) -> None:
        LOGGER.info("Execution state %s -> %s (%s)", group_name, status, message)
        await self._emit(
            make_message(
                "execution_state",
                {
                    "command_id": "ros",
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
        future = asyncio.run_coroutine_threadsafe(self._event_sink(message), self._loop)
        future.add_done_callback(_log_threadsafe_future)

    async def _await_rclpy_future(self, future: Any) -> Any:
        loop = asyncio.get_running_loop()
        wrapped: asyncio.Future[Any] = loop.create_future()

        def _done_callback(done_future: Any) -> None:
            try:
                result = done_future.result()
            except Exception as exc:  # noqa: BLE001
                loop.call_soon_threadsafe(wrapped.set_exception, exc)
            else:
                loop.call_soon_threadsafe(wrapped.set_result, result)

        future.add_done_callback(_done_callback)
        return await wrapped


def _round_list(values: list[float], digits: int = 3) -> list[float]:
    return [round(value, digits) for value in values]


def _log_threadsafe_future(future: "concurrent.futures.Future[Any]") -> None:
    with contextlib.suppress(Exception):
        future.result()
