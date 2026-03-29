from setuptools import setup, find_packages
import os
from glob import glob

package_name = "ur5e_collision_avoidance"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.py")),
        (os.path.join("share", package_name, "config"),
         glob("config/*.yaml")),
        (os.path.join("share", package_name, "worlds"),
         glob("worlds/*.world")),
        (os.path.join("share", package_name, "urdf"),
         glob("urdf/*.xacro")),
        (os.path.join("share", package_name, "rviz"),
         glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="Dual UR5e collision avoidance in Gazebo + ROS 2",
    license="MIT",
    entry_points={
        "console_scripts": [
            "collision_monitor    = ur5e_collision_avoidance.scripts.collision_monitor:main",
            "robot1_controller   = ur5e_collision_avoidance.scripts.robot1_controller:main",
            "robot2_controller   = ur5e_collision_avoidance.scripts.robot2_controller:main",
            "scenario_orchestrator = ur5e_collision_avoidance.scripts.scenario_orchestrator:main",
        ],
    },
)