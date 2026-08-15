from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'manual_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py') + ['launch/manual_drive.py'],
        ),
        (os.path.join('share', package_name), ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xytron',
    maintainer_email='ktypet13@hanyang.ac.kr',
    description='Deadman-gated Xbox manual drive for Xycar',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystic = manual_drive.joystic:main'
        ],
    },
)
