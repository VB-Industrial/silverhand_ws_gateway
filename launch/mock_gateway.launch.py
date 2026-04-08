from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="8765"),
            Node(
                package="silverhand_arm_ws_gateway",
                executable="gateway",
                name="silverhand_arm_ws_gateway",
                output="screen",
                arguments=[
                    "--domain",
                    "arm",
                    "--mode",
                    "mock",
                    "--host",
                    LaunchConfiguration("host"),
                    "--port",
                    LaunchConfiguration("port"),
                ],
            ),
        ]
    )
