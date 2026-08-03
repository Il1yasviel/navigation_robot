from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'navigation_robot_remote_control'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='air',
    maintainer_email='air@localhost.local',
    description='Safe wireless keyboard teleoperation for navigation_robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_teleop = navigation_robot_remote_control.keyboard_teleop:main',
            'local_map_publisher = navigation_robot_remote_control.local_map_publisher:main',
        ],
    },
)
