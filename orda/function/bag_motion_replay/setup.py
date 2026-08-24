import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'bag_motion_replay'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xytron',
    maintainer_email='ktypet13@hanyang.ac.kr',
    description='Byte-exact, deadline-scheduled replay of recorded drive commands',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'replay_node = bag_motion_replay.replay_node:main',
            'verify_node = bag_motion_replay.verify_node:main',
            'bag_motion_cue = bag_motion_replay.cue_tool:main',
            'bag_motion_selftest = bag_motion_replay.selftest:main',
        ],
    },
)
