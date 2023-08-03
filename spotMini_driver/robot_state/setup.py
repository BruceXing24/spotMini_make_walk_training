from setuptools import setup
from glob import glob
import os
package_name = 'robot_state'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name),glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='woofh',
    maintainer_email='woofh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_data = robot_state.sensor_data:main',
            'motors_start = robot_state.motors_start:main',
            'imu_data = robot_state.jy901b:main',
            'PPO_controller = robot_state.PPO_controller:main' 
        ],
    },
)
