from setuptools import setup
from glob import glob
import os
package_name = 'motor_controller'

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
    maintainer='roslab',
    maintainer_email='yingguang.xing@alumni.fh-aachen.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_controller = motor_controller.pose_controller:main',
            'sensor_data = motor_controller.sensor_data:main',
            'motors_start = motor_controller.motors_start:main'
        ],
    },
)
