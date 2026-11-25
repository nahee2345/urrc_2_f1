from setuptools import setup
from glob import glob
from os.path import join

setup(
    name='team07_race', version='0.1.0', packages=['team07_race'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/team07_race']),
        ('share/team07_race', ['package.xml']),
        (join('share','team07_race','launch'), glob('launch/*.launch.py')),
        (join('share','team07_race','models','car'), ['models/car/model.sdf']),
    ],
    install_requires=['setuptools'], zip_safe=False,
    maintainer='team07', maintainer_email='team07@example.invalid',
    description='Competition wrapper for unmodified TEAM07 hybrid_racer',
    license='MIT',
    entry_points={'console_scripts': ['hybrid_racer = team07_race.hybrid_racer:main']},
)
