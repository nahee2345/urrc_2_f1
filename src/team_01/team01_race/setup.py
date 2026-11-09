from setuptools import setup
from glob import glob

name = 'team01_race'
setup(
    name=name, version='0.1.0', packages=[name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml', 'race_entry.json']),
        ('share/' + name + '/launch', glob('launch/*.launch.py')),
        ('share/' + name + '/config', glob('config/*')),
        ('share/' + name + '/models/team01_car', glob('models/team01_car/*')),
    ],
    install_requires=['setuptools', 'numpy'], zip_safe=True,
    maintainer='team_01', maintainer_email='team01@example.invalid',
    description='URRC Team 01 depth-aware racing prototype', license='MIT',
    entry_points={'console_scripts': [
        'team01_control = team01_race.node:main',
        'team01_adapter = team01_race.adapter:main',
        'team01_rgbd_relay = team01_race.slam_rgbd:main',
        'team01_scan_frame = team01_race.slam_scan_tf:main',
        'team01_scan_fusion = team01_race.slam_scan_fusion:main',
    ]},
)
