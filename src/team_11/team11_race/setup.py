from setuptools import setup
from glob import glob
from pathlib import Path

name = 'team11_race'
resource_files = [('share/' + name + '/maps/' + folder.name,
                   [str(f) for f in sorted(folder.glob('*')) if f.is_file()])
                  for folder in sorted(Path('maps').iterdir()) if folder.is_dir()]
setup(
    name=name, version='0.1.0', packages=[name],
    data_files=[
      ('share/ament_index/resource_index/packages', ['resource/' + name]),
      ('share/' + name, ['package.xml', 'race_entry.json']),
      ('share/' + name + '/launch', glob('launch/*.launch.py')),
      ('share/' + name + '/models/team11_car', glob('models/team11_car/*')),
      *resource_files,
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='team11', maintainer_email='team11@example.invalid',
    description='TEAM11 self-contained six-track competition path follower',
    license='MIT',
    entry_points={'console_scripts': [
      'team11_control = team11_race.control:main',
      'team11_adapter = team11_race.adapter:main'
    ]},
)
