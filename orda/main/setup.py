from setuptools import find_packages, setup

package_name = 'main'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
       'launch/module_drive.py',     # 지금 만든 파일
       'launch/module_drive_bag_test.py',
    ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='doldolmeng2',
    maintainer_email='ktypet13@hanyang.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_node = main.main:main',
        ],
    },
)
