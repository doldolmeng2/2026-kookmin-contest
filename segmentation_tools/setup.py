from setuptools import find_packages, setup


package_name = 'segmentation_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'PIDNET_SOURCE.md']),
        ('share/' + package_name + '/model', ['model/pidnet_s_best.pt']),
        ('share/' + package_name + '/third_party', ['third_party/PIDNet_LICENSE']),
        ('share/' + package_name + '/config', ['config/color_filters.yaml']),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML', 'torch'],
    zip_safe=True,
    maintainer='xycar team',
    maintainer_email='maintainer@example.com',
    description='ROS 2 bag color-filter tuning and PIDNet dataset annotation tools',
    license='MIT',
    entry_points={
        'console_scripts': [
            'apply_roi_mask = segmentation_tools.apply_roi_mask:main',
            'color_filter_tuner = segmentation_tools.color_filter_tuner:main',
            'extract_dataset = segmentation_tools.extract_dataset:main',
            'label_editor = segmentation_tools.label_editor:main',
            'merge_datasets = segmentation_tools.merge_datasets:main',
            'split_dataset = segmentation_tools.split_dataset:main',
            'train_pidnet = segmentation_tools.train_pidnet:main',
            'pidnet_inference = segmentation_tools.infer_pidnet:main',
        ],
    },
)
