#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    package_dir = get_package_share_directory('px4_control')
    
    return LaunchDescription([
        # Mavlink Mission Node (通过MAVLink上传任务)
        Node(
            package='px4_control',
            executable='mavlink_mission.py',
            name='mavlink_mission',
            output='screen',
            parameters=[{
                'mavlink_connection': 'udp:127.0.0.1:14540'
            }]
        ),
        
        # PX4 Mission Node (执行任务并监测状态)
        Node(
            package='px4_control',
            executable='px4_mission',
            name='px4_mission',
            output='screen',
            parameters=[]
        ),
    ])

