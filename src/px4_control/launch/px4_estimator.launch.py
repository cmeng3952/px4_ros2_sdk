#!/usr/bin/env python

"""
Launch file for PX4 Estimator node.
Subscribes to multiple PX4 topics and displays status information in terminal.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    px4_estimator_node = Node(
        package='px4_control',
        executable='px4_estimator',
        name='px4_estimator',
        output='screen',
    )

    return LaunchDescription([
        px4_estimator_node
    ])

