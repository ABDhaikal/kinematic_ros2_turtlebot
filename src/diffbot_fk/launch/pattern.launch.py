"""Start the pattern driver with config/patterns.yaml (Gazebo must already be running).

Examples:
    ros2 launch diffbot_fk pattern.launch.py
    ros2 launch diffbot_fk pattern.launch.py side_length:=1.5 loops:=2 clockwise:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('diffbot_fk')
    params = os.path.join(pkg, 'config', 'patterns.yaml')

    overrides = {
        'side_length': ParameterValue(LaunchConfiguration('side_length'), value_type=float),
        'loops': ParameterValue(LaunchConfiguration('loops'), value_type=int),
        'clockwise': ParameterValue(LaunchConfiguration('clockwise'), value_type=bool),
        'use_sim_time': True,
    }

    return LaunchDescription([
        DeclareLaunchArgument('side_length', default_value='1.0', description='Square side [m]'),
        DeclareLaunchArgument('loops', default_value='1', description='Number of laps'),
        DeclareLaunchArgument('clockwise', default_value='false', description='Turn direction'),
        Node(
            package='diffbot_fk', executable='pattern_driver',
            parameters=[params, overrides],
            output='screen'),
    ])
