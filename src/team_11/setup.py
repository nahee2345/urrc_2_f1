from setuptools import setup
from glob import glob

setup(
    name="team_11",
    version="0.1.0",
    packages=["team_11"],
    py_modules=["routes", "strategy"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/team_11"]),
        ("share/team_11", ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="URRC team 11",
    maintainer_email="nahee5315@gmail.com",
    description="Tactical route controller (Gazebo adapter not yet implemented)",
    license="MIT",
    entry_points={"console_scripts": [
        "team11_controller = team_11.ros_controller:main"
    ]},
)
