from glob import glob
from setuptools import setup

name = "team11_race"
setup(
    name=name,version="0.1.0",packages=[name],
    data_files=[
        ("share/ament_index/resource_index/packages",["resource/"+name]),
        ("share/"+name,["package.xml","race_entry.json"]),
        ("share/"+name+"/launch",glob("launch/*.launch.py")),
        ("share/"+name+"/models/team11_car",glob("models/team11_car/*")),
        ("share/"+name+"/config",glob("config/*.yaml")),
    ],
    install_requires=["setuptools","numpy"],zip_safe=True,
    maintainer="TEAM11",maintainer_email="team11@example.invalid",
    description="URRC team11 portable official F1 entry",
    license="MIT",
    entry_points={"console_scripts":[
        "team11_drive = team11_race.controller:main",
        "team11_adapter = team11_race.adapter:main",
    ]},
)
