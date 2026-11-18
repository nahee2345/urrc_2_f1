from setuptools import setup
from glob import glob
from pathlib import Path

name = 'team08_race'
resource_files = [('share/' + name + '/maps/' + folder.name,
                   [str(f) for f in sorted(folder.glob('*')) if f.is_file()])
                  for folder in sorted(Path('maps').iterdir()) if folder.is_dir()]
setup(
    name=name, version='0.1.0', packages=[name],
    data_files=[
      ('share/ament_index/resource_index/packages', ['resource/' + name]),
      ('share/' + name, ['package.xml', 'race_entry.json']),
      ('share/' + name + '/launch', glob('launch/*.launch.py')),
      ('share/' + name + '/models/team08_car', glob('models/team08_car/*')),
      *resource_files,
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='team08', maintainer_email='team08@example.invalid',
    description='Legacy metadata only: actual build uses CMakeLists.txt / ament_cmake',
    license='MIT',
    entry_points={'console_scripts': [
      'team08_control = team08_race.control:main',
      'team08_adapter = team08_race.adapter:main'
    ]},
)
