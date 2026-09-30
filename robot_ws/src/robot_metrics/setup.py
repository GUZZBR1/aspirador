from setuptools import setup
package_name = 'robot_metrics'
setup(name=package_name, version='0.1.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]), ('share/' + package_name, ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True, maintainer='Aspirador MVP', maintainer_email='dev@example.com',
      description='Structured simulation metrics', license='Apache-2.0',
      entry_points={'console_scripts': ['metrics_recorder = robot_metrics.recorder:main']})
