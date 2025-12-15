#!/usr/bin/env python

"""
Launch file for MQTT Mission node.
Subscribes to MQTT broker for mission waypoints and publishes to ROS2.
Also publishes mission status to MQTT.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Declare launch arguments (can override YAML config)
    uav_name_arg = DeclareLaunchArgument(
        'uav_name',
        default_value='',
        description='UAV name/ID (overrides YAML config if provided)'
    )
    
    mqtt_broker_arg = DeclareLaunchArgument(
        'mqtt_broker',
        default_value='',
        description='MQTT broker address (overrides YAML config if provided)'
    )
    
    mqtt_username_arg = DeclareLaunchArgument(
        'mqtt_username',
        default_value='',
        description='MQTT username (overrides YAML config if provided)'
    )
    
    mqtt_password_arg = DeclareLaunchArgument(
        'mqtt_password',
        default_value='',
        description='MQTT password (overrides YAML config if provided)'
    )

    return LaunchDescription([
        uav_name_arg,
        mqtt_broker_arg,
        mqtt_username_arg,
        mqtt_password_arg,
        OpaqueFunction(function=launch_setup)
    ])

def launch_setup(context):
    # Try to use source directory config first (for development, no rebuild needed)
    # This allows modifying config without rebuilding the package
    
    # Get the package share directory (installed location)
    pkg_share = FindPackageShare(package='px4_mqtt').find('px4_mqtt')
    install_config_file = os.path.join(pkg_share, 'config', 'mqtt_mission.yaml')
    
    # Try to find source directory config by replacing install path with src path
    source_config_file = None
    if 'install' in pkg_share:
        # Replace install/px4_mqtt/share/px4_mqtt with src/px4_mqtt
        # Example: /path/to/px4_ros2_ws/install/px4_mqtt/share/px4_mqtt
        #       -> /path/to/px4_ros2_ws/src/px4_mqtt
        src_dir = pkg_share.replace('/install/px4_mqtt/share/px4_mqtt', '/src/px4_mqtt')
        source_config_file = os.path.join(src_dir, 'config', 'mqtt_mission.yaml')
    
    # Use source config if it exists, otherwise use installed config
    if source_config_file and os.path.exists(source_config_file):
        config_file = source_config_file
    else:
        config_file = install_config_file

    # Get launch argument values
    uav_name = LaunchConfiguration('uav_name').perform(context)
    mqtt_broker = LaunchConfiguration('mqtt_broker').perform(context)
    mqtt_username = LaunchConfiguration('mqtt_username').perform(context)
    mqtt_password = LaunchConfiguration('mqtt_password').perform(context)

    # Build parameters list - start with YAML config
    parameters = [config_file]
    
    # Only add overrides if launch arguments are provided (non-empty)
    override_params = {}
    if uav_name and uav_name.strip():
        override_params['uav_name'] = uav_name
    if mqtt_broker and mqtt_broker.strip():
        override_params['mqtt_broker'] = mqtt_broker
    if mqtt_username and mqtt_username.strip():
        override_params['mqtt_username'] = mqtt_username
    if mqtt_password and mqtt_password.strip():
        override_params['mqtt_password'] = mqtt_password
    
    if override_params:
        parameters.append(override_params)

    mqtt_mission_node = Node(
        package='px4_mqtt',
        executable='mqtt_mission',
        name='mqtt_mission',
        output='screen',
        parameters=parameters
    )

    return [mqtt_mission_node]

