from __future__ import annotations

import argparse
import asyncio
import logging

from .adapter_factory import resolve_adapter_factory
from .core.config import GatewayConfig
from .core.server import run_gateway


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="SilverHand robot-side websocket gateway")
    parser.add_argument("--domain", choices=("arm", "rover"), default="arm")
    parser.add_argument("--mode", choices=("mock", "ros", "moveit"), default="mock")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--move-group-action", default="/move_action")
    parser.add_argument("--heartbeat-interval", type=float, default=3.0)
    parser.add_argument("--mock-step", type=float, default=0.05)
    parser.add_argument("--mock-steps-per-execute", type=int, default=20)
    parser.add_argument("--mock-max-joint-velocity", type=float, default=1.2)
    parser.add_argument("--rover-cmd-vel-topic", default="/rover_base_controller/cmd_vel_unstamped")
    parser.add_argument("--rover-odom-topic", default="/rover_base_controller/odom")
    parser.add_argument("--rover-battery-topic", default="/battery_state")
    parser.add_argument("--rover-headlights-service", default="/power_board/set_headlights")
    parser.add_argument("--log-level", default="INFO")
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    logging.basicConfig(level=getattr(logging, str(args.log_level).upper(), logging.INFO))

    config = GatewayConfig(
        domain=args.domain,
        mode=args.mode,
        host=args.host,
        port=args.port,
        move_group_action_name=args.move_group_action,
        heartbeat_interval_s=args.heartbeat_interval,
        mock_step_s=args.mock_step,
        mock_steps_per_execute=args.mock_steps_per_execute,
        mock_max_joint_velocity_rad_s=args.mock_max_joint_velocity,
        rover_cmd_vel_topic=args.rover_cmd_vel_topic,
        rover_odom_topic=args.rover_odom_topic,
        rover_battery_topic=args.rover_battery_topic,
        rover_headlights_service=args.rover_headlights_service,
    )

    try:
        adapter_factory = resolve_adapter_factory(config)
        asyncio.run(run_gateway(config, adapter_factory))
    except KeyboardInterrupt:
        logging.getLogger("silverhand_ws_gateway").info("Gateway stopped by user")
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc


if __name__ == "__main__":
    main()
