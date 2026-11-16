from setuptools import setup
import os, glob

package_name = 'detectron2_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shengjian Chen, Steven Macenski',
    maintainer_email='csj15thu@gmail.com, stevenmacenski@gmail.com',
    description='This detector uses detectron2 to get object mask and then uses pointcloud2 data to estimate 3D position.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detectron2_node = detectron2_detector.detectron2_node:main'
        ],
    },
)
