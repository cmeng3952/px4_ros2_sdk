#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='px4_control',
			executable='px4_control',
			name='px4_control',
			output='screen',
			parameters=[]
		)
	])

