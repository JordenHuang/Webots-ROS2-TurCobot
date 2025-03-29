import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_package'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/waffle.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/TurtleBot3Waffle.urdf']))
data_files.append(('share/' + package_name + '/resource/protos', ['resource/protos/TurtleBot3Waffle.proto']))
data_files.append(('share/' + package_name + '/srv', glob('srv/*.srv')))
data_files.append(('share/' + package_name, ['package.xml']))

# Include all mesh files
mesh_files = glob('resource/protos/turtleBot3Waffle_mesh/*')
data_files.append(('share/' + package_name + '/resource/protos/turtleBot3Waffle_mesh', mesh_files))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jordenhuang',
    maintainer_email='jorden90573@gmail.com',
    description='Webots and ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_robot_driver = my_package.my_robot_driver:main',
            'terminal_keyboard_client = my_package.terminal_keyboard_client:main'
        ],
    },
)
