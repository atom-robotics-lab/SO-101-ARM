from setuptools import find_packages, setup

package_name = "so101_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Aditya Dev SIngh",
    maintainer_email="adityadevsingh16@gmail.com",
    description="Bridges lerobot's FeetechMotorsBus to ROS2 topics for ros2_control.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "feetech_bridge_node = so101_bridge.feetech_bridge_node:main",
        ],
    },
)
