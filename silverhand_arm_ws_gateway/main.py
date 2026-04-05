from __future__ import annotations

import argparse
import asyncio
import logging

from .config import GatewayConfig
from .mock_adapter import MockRobotAdapter
from .moveit_adapter import MoveItRobotAdapter
from .ros_adapter import RosRobotAdapter
from .server import run_gateway


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="SilverHand robot-side websocket gateway")
    parser.add_argument("--mode", choices=("mock", "ros", "moveit"), default="mock")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--move-group-action", default="/move_action")
    parser.add_argument("--heartbeat-interval", type=float, default=3.0)
    parser.add_argument("--mock-step", type=float, default=0.05)
    parser.add_argument("--mock-steps-per-execute", type=int, default=20)
    parser.add_argument("--mock-max-joint-velocity", type=float, default=1.2)
    parser.add_argument("--log-level", default="INFO")
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    logging.basicConfig(level=getattr(logging, str(args.log_level).upper(), logging.INFO))

    config = GatewayConfig(
        mode=args.mode,
        host=args.host,
        port=args.port,
        move_group_action_name=args.move_group_action,
        heartbeat_interval_s=args.heartbeat_interval,
        mock_step_s=args.mock_step,
        mock_steps_per_execute=args.mock_steps_per_execute,
        mock_max_joint_velocity_rad_s=args.mock_max_joint_velocity,
    )

    if args.mode == "mock":
        adapter_factory = MockRobotAdapter
    elif args.mode == "ros":
        adapter_factory = RosRobotAdapter
    else:
        adapter_factory = MoveItRobotAdapter
    try:
        asyncio.run(run_gateway(config, adapter_factory))
    except KeyboardInterrupt:
        logging.getLogger("silverhand_arm_ws_gateway").info("Gateway stopped by user")


if __name__ == "__main__":
    main()
