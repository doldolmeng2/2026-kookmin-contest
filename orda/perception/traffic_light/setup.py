from setuptools import setup
import glob
package_name = 'traffic_light'
setup(
    name=package_name, version='1.0.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/'+package_name]),
        ('share/'+package_name, ['package.xml']),
        ('share/'+package_name+'/model', glob.glob('model/*.onnx')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='orda', maintainer_email='hyerica.orda@gmail.com',
    description='4gu traffic light detection', license='MIT',
    entry_points={'console_scripts': ['traffic_node = traffic_light.traffic_node:main']},
)
