#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
	return LaunchDescription([
		# Joystick device arguments
		DeclareLaunchArgument(
			'device',
			default_value='/dev/input/js0',
			description='Joystick device path (e.g., /dev/input/js0)'
		),
		DeclareLaunchArgument(
			'publish_rate',
			default_value='50.0',
			description='Joystick publishing rate in Hz'
		),
		
		# Axis mapping arguments
		DeclareLaunchArgument(
			'roll_axis',
			default_value='0',
			description='Joystick axis index for roll control'
		),
		DeclareLaunchArgument(
			'pitch_axis',
			default_value='1',
			description='Joystick axis index for pitch control'
		),
		DeclareLaunchArgument(
			'throttle_axis',
			default_value='2',
			description='Joystick axis index for throttle control'
		),
		DeclareLaunchArgument(
			'yaw_axis',
			default_value='3',
			description='Joystick axis index for yaw control'
		),
		DeclareLaunchArgument(
			'roll_inverted',
			default_value='false',
			description='Invert roll axis'
		),
		DeclareLaunchArgument(
			'pitch_inverted',
			default_value='false',
			description='Invert pitch axis'
		),
		DeclareLaunchArgument(
			'throttle_inverted',
			default_value='false',
			description='Invert throttle axis'
		),
		DeclareLaunchArgument(
			'yaw_inverted',
			default_value='false',
			description='Invert yaw axis'
		),
		DeclareLaunchArgument(
			'dead_zone',
			default_value='0.05',
			description='Dead zone threshold (0.0-1.0)'
		),
		
		# Joystick bridge node (reads USB joystick and publishes /joy)
		Node(
			package='px4_control',
			executable='joystick_bridge',
			name='joystick_bridge',
			output='screen',
			parameters=[{
				'device': LaunchConfiguration('device'),
				'publish_rate': LaunchConfiguration('publish_rate'),
			}]
		),
		
		# PX4 Joystick converter node (converts /joy to /fmu/in/manual_control_input)
		Node(
			package='px4_control',
			executable='px4_joystick',
			name='px4_joystick',
			output='screen',
			parameters=[{
				'roll_axis': LaunchConfiguration('roll_axis'),
				'pitch_axis': LaunchConfiguration('pitch_axis'),
				'throttle_axis': LaunchConfiguration('throttle_axis'),
				'yaw_axis': LaunchConfiguration('yaw_axis'),
				'roll_inverted': LaunchConfiguration('roll_inverted'),
				'pitch_inverted': LaunchConfiguration('pitch_inverted'),
				'throttle_inverted': LaunchConfiguration('throttle_inverted'),
				'yaw_inverted': LaunchConfiguration('yaw_inverted'),
				'dead_zone': LaunchConfiguration('dead_zone'),
			}]
		)
	])


