#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='Pose',
            executable='spot_pose_controller',
            name = 'rpy_controller',
            output = 'screen'

        ),
        Node(
            package='Pose',
            executable='spot_show',
            name = 'spot_show',
            output = 'screen'
        ),
    ]
    )
