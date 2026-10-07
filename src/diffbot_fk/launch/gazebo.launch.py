"""Gazebo + TurtleBot3 Burger robot (diffbot) + forward kinematics odometry node + RViz.

Examples:
    ros2 launch diffbot_fk gazebo.launch.py
    ros2 launch diffbot_fk gazebo.launch.py rviz:=false gui:=false
Then in another terminal:
    ros2 run diffbot_fk keypad_teleop --ros-args --params-file <share>/config/robot_params.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('diffbot_fk')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    x, y, yaw = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('yaw')

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(pkg, 'urdf', 'diffbot.urdf.xacro')]), value_type=str)
    params = os.path.join(pkg, 'config', 'robot_params.yaml')

    # So Gazebo can find the package://turtlebot3_description/... meshes
    tb3_share_parent = os.path.dirname(get_package_share_directory('turtlebot3_description'))

    return LaunchDescription([
        AppendEnvironmentVariable('GAZEBO_MODEL_PATH', tb3_share_parent),
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'empty_grid.world')),
        DeclareLaunchArgument('gui', default_value='true', description='Show the Gazebo window'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz2'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world, 'verbose': 'false'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(gui)),

        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            output='screen'),

        Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'diffbot',
                       '-x', x, '-y', y, '-z', '0.01', '-Y', yaw],
            output='screen'),

        Node(
            package='diffbot_fk', executable='fk_odometry',
            parameters=[params, {'use_sim_time': True}],
            output='screen'),

        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', os.path.join(pkg, 'rviz', 'diffbot.rviz')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(rviz)),
    ])
