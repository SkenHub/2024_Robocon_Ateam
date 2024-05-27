from setuptools import find_packages, setup

package_name = 'robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='altair',
    maintainer_email='Altairu@github.comu',
    description='ROS2 package for robot control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = robot_controller.controller_node:main',
            'cmd_vel_to_serial_node = robot_controller.cmd_vel_to_serial_node:main',
            'serial_to_position_node = robot_controller.serial_to_position_node:main'
        ],
    },
)
