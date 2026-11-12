from setuptools import setup
from glob import glob
import os

setup(
    name="team_11",
    version="0.1.0",
    packages=["team_11"],
    py_modules=["routes", "strategy"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/team_11"]),
        ("share/team_11", ["package.xml", "README.md"]),
        ("share/team_11/models/team11_yellow_car", ["models/team11_yellow_car/model.sdf", "models/team11_yellow_car/model.config"]),
        ("share/team_11/tests", ["tests/model_test_world.sdf"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="URRC team 11",
    maintainer_email="nahee5315@gmail.com",
    description="Tactical route controller (Gazebo adapter not yet implemented)",
    license="MIT",
    entry_points={"console_scripts": [
        "team11_controller = team_11.ros_controller:main",
        "team11_stage5_adapter = team_11.stage5_adapter:main",
        "team11_stage5_manual = team_11.stage5_manual:main",
        "team11_context = team_11.context_node:main",
        "team11_signal_camera = team_11.signal_camera:main",
        "team11_stage7_drive = team_11.stage7_drive:main"
    ]},
)
