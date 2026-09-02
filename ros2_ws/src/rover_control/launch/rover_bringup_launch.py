#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

from launch.substitutions import EnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import os

def generate_launch_description():

    ####################################################################
    microros = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'serial',
            '--dev', '/dev/ttyUSB0',
            '--baudrate', '115200',
            '-v4'
        ],
        output='screen'
    )

    ####################################################################
    livox_driver = ExecuteProcess(
        cmd=[
            'ros2', 'launch',
            'livox_ros_driver2', 'msg_MID360_launch.py',
        ],
        output='screen'
    )

    ####################################################################
    fastlio2_slam = ExecuteProcess(
        cmd=[
            'ros2', 'launch',
            'fast_lio', 'mapping.launch.py',
            'config_file:=~/fastlio_ws/src/FAST_LIO_ROS2/config/mid360.yaml',
            'rviz:=false'
        ],
        output='screen'
    )

    ####################################################################
    fastlio2_pose = ExecuteProcess(
        cmd=[
            'ros2', 'run',
            'fastlio2_pose', 'fastlio2_pose_node'
        ],
        output='screen'
    )

    ####################################################################
    rover = Node(
        package='rover_control',
        executable='rover_controller',
        output='screen'
    )

    ####################################################################
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            os.path.expanduser(
                '~/ros2_ws/src/rover_control/rviz/rviz_config.rviz'
            )
        ],
        output='screen'
    )

    return LaunchDescription([
        microros,
        livox_driver,

        TimerAction(period=5.0, actions=[fastlio2_slam]),
        TimerAction(period=10.0, actions=[fastlio2_pose]),

        TimerAction(period=12.0, actions=[rviz]),
        TimerAction(period=14.0, actions=[rover]),
    ])
