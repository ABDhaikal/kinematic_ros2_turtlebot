import os
from glob import glob

from setuptools import setup

package_name = 'diffbot_fk'


def share(sub, pattern):
    return (os.path.join('share', package_name, sub), glob(os.path.join(sub, pattern)))


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        share('launch', '*.launch.py'),
        share('urdf', '*.xacro'),
        share('worlds', '*.world'),
        share('config', '*.yaml'),
        share('rviz', '*.rviz'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='haikal',
    maintainer_email='haikalabdurrahman95@gmail.com',
    description='Differential drive robot (TurtleBot3 Burger) with forward kinematics in Gazebo.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fk_odometry = diffbot_fk.fk_odometry_node:main',
            'keypad_teleop = diffbot_fk.keypad_teleop_node:main',
            'pattern_driver = diffbot_fk.pattern_driver_node:main',
        ],
    },
)
