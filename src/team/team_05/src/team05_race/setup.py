from setuptools import setup
from glob import glob

setup(
    name='team05_race',
    version='0.3.0',
    packages=['team05_race'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/team05_race']),
        ('share/team05_race', ['package.xml']),
        ('share/team05_race/launch', glob('launch/*.py')),
        ('share/team05_race/models/car', glob('models/car/*.sdf')),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'controller=team05_race.controller:main'
        ]
    }
)
