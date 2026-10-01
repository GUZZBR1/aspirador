from setuptools import setup
from glob import glob
import os
package_name = 'robot_tests'
setup(name=package_name, version='0.1.0', packages=[package_name], data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'scenarios'), glob('scenarios/*.json')),
], install_requires=['setuptools'], zip_safe=True, maintainer='Aspirador MVP', maintainer_email='dev@example.com',
description='Automated simulation scenarios and telemetry validation', license='Apache-2.0', entry_points={'console_scripts': ['mobility_runner = robot_tests.runner:main', 'battery_runner = robot_tests.battery_runner:main', 'physics_runner = robot_tests.physics_runner:main']})
