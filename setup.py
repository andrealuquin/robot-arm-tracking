from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_tracking_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'urdf'),
            glob('urdf/*'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrea',
    maintainer_email='andrealuquin15@gmail.com',
    description='ROS 2 project for 3D robot arm tracking with Jacobian-based control',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller_node = robot_tracking_project.controller_node:main',
            'target_publisher = robot_tracking_project.target_publisher:main',
        ],
    },
)
