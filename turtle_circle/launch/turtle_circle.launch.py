#!/usr/bin/env python3
"""Launch turtlesim + the turtle_circle node (and optionally a rosbag recording).

Examples
--------
Default run (period 8 s, amplitude 2 m/s)::

    ros2 launch turtle_circle turtle_circle.launch.py

Custom parameters::

    ros2 launch turtle_circle turtle_circle.launch.py period:=4.0 amplitude:=1.5

Also record a bag (to ./turtle_bag)::

    ros2 launch turtle_circle turtle_circle.launch.py record:=true bag_dir:=turtle_bag
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    period = LaunchConfiguration('period')
    amplitude = LaunchConfiguration('amplitude')
    publish_rate = LaunchConfiguration('publish_rate')
    record = LaunchConfiguration('record')
    bag_dir = LaunchConfiguration('bag_dir')

    declared = [
        DeclareLaunchArgument(
            'period', default_value='8.0',
            description='Seconds for one full circle (controls angular speed).'),
        DeclareLaunchArgument(
            'amplitude', default_value='2.0',
            description='Linear speed of the turtle in m/s (controls circle radius).'),
        DeclareLaunchArgument(
            'publish_rate', default_value='20.0',
            description='cmd_vel publish rate in Hz.'),
        DeclareLaunchArgument(
            'record', default_value='false',
            description='If true, also start `ros2 bag record`.'),
        DeclareLaunchArgument(
            'bag_dir', default_value='turtle_bag',
            description='Output directory of the rosbag (when record:=true).'),
    ]

    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen',
    )

    circle_node = Node(
        package='turtle_circle',
        executable='circle_node',
        name='turtle_circle',
        output='screen',
        parameters=[{
            'period': ParameterValue(period, value_type=float),
            'amplitude': ParameterValue(amplitude, value_type=float),
            'publish_rate': ParameterValue(publish_rate, value_type=float),
        }],
    )

    record_bag = Node(
        package='turtle_circle',
        executable='record_bag',   # wrapper that runs `ros2 bag record`
        name='record_bag',
        output='screen',
        condition=IfCondition(record),
        parameters=[{'bag_dir': ParameterValue(bag_dir, value_type=str)}],
    )

    return LaunchDescription(declared + [turtlesim, circle_node, record_bag])
