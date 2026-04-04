from setuptools import find_packages, setup


package_name = "silverhand_arm_ws_gateway"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/launch", ["launch/mock_gateway.launch.py", "launch/ros_gateway.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="r",
    maintainer_email="r@localhost",
    description="WebSocket gateway for SilverHand MoveIt and robot telemetry.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "gateway = silverhand_arm_ws_gateway.main:main",
        ],
    },
)
