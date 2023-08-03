from setuptools import setup
import os
from glob import  glob
package_name = 'spot_controller'

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
    maintainer='xing',
    maintainer_email='yingguang.xing@alumni.fh-aachen.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spot_showpb = spot_controller.show_robot:main',
            'spot_show3d = spot_controller.show_robot3D:main',
            'spot_controller = spot_controller.spot_controller:main'
        ],
    },
)
