from setuptools import setup
from glob import glob
import os

package_name = 'robot_autonomy'
setup(name=package_name, version='0.1.0', packages=[package_name], data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
], install_requires=['setuptools'], zip_safe=True, maintainer='Aspirador MVP',
    maintainer_email='dev@example.com', description='Robot autonomy nodes', license='Apache-2.0',
    entry_points={'console_scripts': ['autonomy_manager = robot_autonomy.manager:main']})
