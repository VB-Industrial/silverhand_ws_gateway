from __future__ import annotations

import asyncio
import contextlib
import logging
import math
import threading
import time
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.client import Client
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_srvs.srv import SetBool

from ...core.adapter_base import EventSink, RobotAdapter
from ...core.config import GatewayConfig
from ...core.protocol import as_dict, make_fault_state, make_message
from .protocol import GROUP_ROVER, as_drive_mode, make_rover_state, normalize_input_source


LOGGER = logging.getLogger("silverhand_ws_gateway.rover.ros")


class RoverRosAdapter(RobotAdapter):
    server_name = "silverhand_rover_ws_gateway"
    groups = (GROUP_ROVER,)

    def __init__(self, config: GatewayConfig) -> None:
        self._config = config
        self._event_sink: EventSink | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._node: Node | None = None
        self._cmd_vel_publisher: Any = None
        self._headlights_client: Client | None = None
        self._spin_thread: threading.Thread | None = None
        self._status_task: asyncio.Task[None] | None = None
        self._stop_event = threading.Event()
        self._rclpy_owned = False
        self._drive_mode = "manual"
        self._ready = True
        self._input_source = "keyboard_mouse"
        self._estop_active = False
        self._headlights_enabled = False
        self._last_command_monotonic = time.monotonic()
        self._odometer_km = 0.0
        self._last_xy: tuple[float, float] | None = None

    async def start(self, event_sink: EventSink) -> None:
        self._event_sink = event_sink
        self._loop = asyncio.get_running_loop()

        if not rclpy.ok():
            rclpy.init(args=None)
            self._rclpy_owned = True

        self._node = rclpy.create_node("silverhand_rover_ws_gateway")
        self._cmd_vel_publisher = self._node.create_publisher(Twist, self._config.rover_cmd_vel_topic, 10)
        self._node.create_subscription(Odometry, self._config.rover_odom_topic, self._on_odometry, 10)
        self._node.create_subscription(BatteryState, self._config.rover_battery_topic, self._on_battery_state, 10)
        self._headlights_client = self._node.create_client(SetBool, self._config.rover_headlights_service)

        self._spin_thread = threading.Thread(target=self._spin_worker, name="silverhand_ws_gateway_rover_spin", daemon=True)
        self._spin_thread.start()
        self._status_task = asyncio.create_task(self._status_loop())

        await self._publish_rover_state()

        headlights_ready = await self._wait_for_service(self._headlights_client, timeout_s=1.0)
        if headlights_ready:
            await self._emit(
                make_fault_state(
                    "rover_adapter_ready",
                    "Rover ROS adapter connected.",
                    severity="info",
                    active=False,
                )
            )
        else:
            await self._emit(
                make_fault_state(
                    "headlights_service_unavailable",
                    f"Headlights service {self._config.rover_headlights_service} is unavailable.",
                    severity="warning",
                    active=False,
                )
            )

    async def stop(self) -> None:
        self._stop_event.set()
        if self._status_task is not None:
            self._status_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._status_task
            self._status_task = None

        if self._spin_thread is not None:
            self._spin_thread.join(timeout=1.0)
            self._spin_thread = None

        if self._node is not None:
            self._node.destroy_node()
            self._node = None

        self._cmd_vel_publisher = None
        self._headlights_client = None

        if self._rclpy_owned and rclpy.ok():
            rclpy.shutdown()
            self._rclpy_owned = False

    async def handle_message(self, message: dict[str, Any]) -> None:
        message_type = message.get("type")
        payload = as_dict(message.get("payload", {}), field_name="payload")

        if message_type == "cmd_vel":
            await self._handle_cmd_vel(payload)
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
        if message_type == "set_drive_mode":
            await self._handle_set_drive_mode(payload)
            return
        if message_type == "set_headlights":
            await self._handle_set_headlights(payload)
            return

    async def _handle_cmd_vel(self, payload: dict[str, Any]) -> None:
        if self._estop_active:
            await self._emit(make_fault_state("rover_estop_active", "Ignoring cmd_vel because E-STOP is active.", severity="warning", active=True))
            await self._publish_rover_state()
            return

        twist = Twist()
        twist.linear.x = float(payload.get("linear_m_s", 0.0))
        twist.angular.z = float(payload.get("angular_rad_s", 0.0))
        self._cmd_vel_publisher.publish(twist)

        self._input_source = normalize_input_source(payload, default=self._input_source)
        self._last_command_monotonic = time.monotonic()
        await self._publish_rover_state()

    async def _handle_stop(self) -> None:
        self._publish_zero_twist()
        self._last_command_monotonic = time.monotonic()
        await self._publish_rover_state()

    async def _handle_estop(self) -> None:
        self._estop_active = True
        self._publish_zero_twist()
        await self._emit(make_fault_state("rover_estop_active", "Emergency stop active.", severity="fatal", active=True))
        await self._publish_rover_state()

    async def _handle_reset_estop(self) -> None:
        self._estop_active = False
        self._last_command_monotonic = time.monotonic()
        await self._emit(make_fault_state("rover_estop_active", "Emergency stop reset.", severity="info", active=False))
        await self._publish_rover_state()

    async def _handle_set_drive_mode(self, payload: dict[str, Any]) -> None:
        self._drive_mode = as_drive_mode(payload)
        await self._publish_rover_state()

    async def _handle_set_headlights(self, payload: dict[str, Any]) -> None:
        if self._headlights_client is None:
            await self._emit(make_fault_state("headlights_client_missing", "Headlights client is not ready.", severity="error", active=True))
            await self._publish_rover_state()
            return

        enabled = bool(payload.get("enabled", False))
        request = SetBool.Request()
        request.data = enabled

        if not await self._wait_for_service(self._headlights_client, timeout_s=1.0):
            await self._emit(
                make_fault_state(
                    "headlights_service_unavailable",
                    f"Headlights service {self._config.rover_headlights_service} is unavailable.",
                    severity="warning",
                    active=True,
                )
            )
            await self._publish_rover_state()
            return

        response = await self._await_rclpy_future(self._headlights_client.call_async(request))
        if not response.success:
            await self._emit(make_fault_state("headlights_call_failed", response.message or "Headlights service call failed.", severity="error", active=True))
            await self._publish_rover_state()
            return

        self._headlights_enabled = enabled
        await self._emit(make_fault_state("headlights_changed", response.message or ("Headlights enabled" if enabled else "Headlights disabled"), severity="info", active=False))
        await self._publish_rover_state()

    async def _status_loop(self) -> None:
        while True:
            await asyncio.sleep(0.2)
            await self._publish_rover_state()

    def _spin_worker(self) -> None:
        assert self._node is not None
        while rclpy.ok() and not self._stop_event.is_set():
            rclpy.spin_once(self._node, timeout_sec=0.1)

    def _on_odometry(self, message: Odometry) -> None:
        orientation = message.pose.pose.orientation
        heading_deg = _yaw_from_quaternion_deg(orientation.x, orientation.y, orientation.z, orientation.w)
        x_m = float(message.pose.pose.position.x)
        y_m = float(message.pose.pose.position.y)

        if self._last_xy is not None:
            dx = x_m - self._last_xy[0]
            dy = y_m - self._last_xy[1]
            self._odometer_km += math.hypot(dx, dy) / 1000.0
        self._last_xy = (x_m, y_m)

        self._emit_from_thread(
            make_message(
                "odometry",
                {
                    "linear_m_s": float(message.twist.twist.linear.x),
                    "angular_rad_s": float(message.twist.twist.angular.z),
                    "heading_deg": heading_deg,
                    "odometer_km": self._odometer_km,
                    "x_m": x_m,
                    "y_m": y_m,
                },
            )
        )

    def _on_battery_state(self, message: BatteryState) -> None:
        percent = float(message.percentage * 100.0) if math.isfinite(message.percentage) and message.percentage >= 0.0 else 0.0
        self._emit_from_thread(
            make_message(
                "battery_state",
                {
                    "percent": percent,
                    "voltage_v": float(message.voltage),
                    "current_a": float(message.current),
                },
            )
        )

    def _publish_zero_twist(self) -> None:
        if self._cmd_vel_publisher is None:
            return
        twist = Twist()
        self._cmd_vel_publisher.publish(twist)

    async def _publish_rover_state(self) -> None:
        await self._emit(
            make_rover_state(
                mode=self._drive_mode,
                ready=self._ready,
                control_active=not self._estop_active,
                headlights_enabled=self._headlights_enabled,
                input_source=self._input_source,
                signal_quality="strong",
                command_age_ms=(time.monotonic() - self._last_command_monotonic) * 1000.0,
            )
        )

    async def _wait_for_service(self, client: Client | None, *, timeout_s: float) -> bool:
        if client is None:
            return False
        try:
            return bool(await asyncio.to_thread(client.wait_for_service, timeout_s))
        except Exception as exc:  # noqa: BLE001
            LOGGER.warning("Failed while waiting for service: %s", exc)
            return False

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


def _yaw_from_quaternion_deg(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)
    heading_deg = math.degrees(yaw_rad) % 360.0
    if heading_deg < 0.0:
        heading_deg += 360.0
    return heading_deg


def _log_threadsafe_future(future: "concurrent.futures.Future[Any]") -> None:
    with contextlib.suppress(Exception):
        future.result()
