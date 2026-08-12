from setuptools import find_packages, setup


package_name = 'segmentation_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/color_filters.yaml']),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML'],
    zip_safe=True,
    maintainer='xycar team',
    maintainer_email='maintainer@example.com',
    description='ROS 2 bag color-filter tuning and PIDNet dataset annotation tools',
    license='MIT',
    entry_points={
        'console_scripts': [
            'color_filter_tuner = segmentation_tools.color_filter_tuner:main',
            'extract_dataset = segmentation_tools.extract_dataset:main',
            'label_editor = segmentation_tools.label_editor:main',
            'merge_datasets = segmentation_tools.merge_datasets:main',
        ],
    },
)
