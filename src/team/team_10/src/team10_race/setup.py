from glob import glob
import os
from setuptools import find_packages, setup

package_name = "team10_race"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "models", "team10_car"), glob("models/team10_car/*")),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="Hanla Team",
    maintainer_email="student@example.com",
    description="LiDAR-only wall navigation and race adapter for URRC.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "navigation = team10_race.relaxed_navigation_node:main",
            "start_camera = team10_race.start_camera_node:main",
            "race_adapter = team10_race.race_adapter:main",
        ],
    },
)
