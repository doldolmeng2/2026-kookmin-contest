from setuptools import find_packages, setup

package_name = 'bag_visualizer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xytron',
    maintainer_email='ynkg1026@gmail.com',
    description='rosbag2 이미지 토픽을 영상처럼 재생/드래그 탐색하는 뷰어',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bag_visualizer = bag_visualizer.player:main',
        ],
    },
)
