from glob import glob
from setuptools import find_packages, setup

package_name="team09_awd"
setup(name=package_name,version="27.0.0",packages=find_packages(exclude=["test"]),
      data_files=[("share/ament_index/resource_index/packages",["resource/"+package_name]),
                  ("share/"+package_name,["package.xml"]),
                  ("share/"+package_name+"/launch",glob("launch/*.py")),
                  ("share/"+package_name+"/config",glob("config/*")),
                  ("share/"+package_name+"/models/car",glob("models/car/*"))],
      install_requires=["setuptools"],zip_safe=False,
      maintainer="URRC team",maintainer_email="team@example.invalid",
      description="Sensor-only AWD racer for URRC v27",license="Apache-2.0",
      entry_points={"console_scripts":["racer = team09_awd.node:main","actuator = team09_awd.actuator_node:main"]})
