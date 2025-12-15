#!/usr/bin/env python3
"""
命令行工具：发送固定格式航点任务
Usage: 
    python3 px4_mission_cli.py --waypoints "frame,cmd,lat,lon,alt,p1,p2,p3,p4|frame,cmd,lat,lon,alt,p1,p2,p3,p4|..."
    或
    python3 px4_mission_cli.py --file mission.json
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
import json
import sys
import argparse

class PX4MissionCLI(Node):
    def __init__(self):
        super().__init__('px4_mission_cli')
        self.waypoints_pub = self.create_publisher(
            Float32MultiArray, '/mission/waypoints', 10)
        self.trigger_pub = self.create_publisher(
            Bool, '/mission/trigger', 10)
        
        # Wait for publishers to be ready
        import time
        time.sleep(0.5)
    
    def send_waypoints(self, waypoints):
        """发送航点到mavlink_mission节点
        
        Args:
            waypoints: 航点列表，每个航点是9个参数的列表
        """
        msg = Float32MultiArray()
        for wp in waypoints:
            if len(wp) != 9:
                self.get_logger().error(f"Invalid waypoint format: {wp}")
                return False
            msg.data.extend(wp)
        
        self.waypoints_pub.publish(msg)
        self.get_logger().info(f"Sent {len(waypoints)} waypoints")
        return True
    
    def trigger_mission(self):
        """发送任务触发信号到px4_mission节点"""
        msg = Bool()
        msg.data = True
        self.trigger_pub.publish(msg)

def parse_waypoints_string(waypoints_str):
    """解析命令行字符串格式的航点
    
    格式: "frame,cmd,lat,lon,alt,p1,p2,p3,p4|frame,cmd,lat,lon,alt,p1,p2,p3,p4|..."
    """
    waypoints = []
    wp_strings = waypoints_str.split('|')
    
    for wp_str in wp_strings:
        wp_str = wp_str.strip()
        if not wp_str:
            continue
        
        try:
            values = [float(x.strip()) for x in wp_str.split(',')]
            if len(values) != 9:
                print(f"Error: Waypoint must have 9 values, got {len(values)}: {wp_str}")
                return None
            waypoints.append(values)
        except ValueError as e:
            print(f"Error parsing waypoint '{wp_str}': {e}")
            return None
    
    return waypoints

def load_waypoints_from_json(filename):
    """从JSON文件加载航点"""
    with open(filename, 'r') as f:
        data = json.load(f)
    
    waypoints = []
    if 'waypoints' in data:
        for wp in data['waypoints']:
            waypoint = [
                wp.get('frame', 3),
                wp.get('command', 16),
                wp.get('lat', 0.0),
                wp.get('lon', 0.0),
                wp.get('alt', 0.0),
                wp.get('param1', 0.0),
                wp.get('param2', 1.0),
                wp.get('param3', 0.0),
                wp.get('param4', 0.0)
            ]
            waypoints.append(waypoint)
    return waypoints


def main():
    parser = argparse.ArgumentParser(
        description='Send waypoint mission to PX4',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Use JSON file
  python3 px4_mission_cli.py --file examples/example_mission.json
  
  # Use command line waypoints
  python3 px4_mission_cli.py --waypoints "3,16,47.397742,8.545594,5.0,0.0,1.0,0.0,0.0|3,16,47.397842,8.545594,5.0,0.0,1.0,0.0,0.0"
  
Waypoint format: frame,command,lat,lon,alt,param1,param2,param3,param4

Frame types:
  - 0: FRAME_GLOBAL (absolute altitude)
  - 1: FRAME_LOCAL_NED (local NED frame)
  - 2: FRAME_MISSION (mission frame)
  - 3: FRAME_GLOBAL_REL_ALT (relative altitude, recommended)

Common commands:
  - 16: NAV_WAYPOINT - Standard waypoint
  - 17: NAV_LOITER_UNLIMITED - Loiter indefinitely
  - 19: NAV_LOITER_TIME - Loiter for specified time
  - 20: NAV_RETURN_TO_LAUNCH - Return to launch point
  - 21: NAV_LAND - Land at waypoint
  - 22: NAV_TAKEOFF - Takeoff to altitude
  - 82: NAV_SPLINE_WAYPOINT - Spline waypoint (smooth path)
  - 178: DO_CHANGE_SPEED - Change speed
  - 201: DO_SET_ROI - Set region of interest

Command-specific parameters (for NAV_WAYPOINT):
  - param1: hold time in seconds
  - param2: acceptance radius in meters
  - param3: pass radius in meters (0 = pass through)
  - param4: yaw angle in radians (0 = no change)

For other commands, refer to MAVLink mission command documentation.
        """
    )
    
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--waypoints', type=str, help='Waypoints string format: "wp1|wp2|..."')
    group.add_argument('--file', type=str, help='JSON file with waypoints')
    
    parser.add_argument('--no-trigger', action='store_true', 
                       help='Only upload mission, do not trigger execution')
    
    args = parser.parse_args()
    
    rclpy.init()
    cli = PX4MissionCLI()
    
    try:
        # Load waypoints
        if args.file:
            waypoints = load_waypoints_from_json(args.file)
            cli.get_logger().info(f"Loaded waypoints from {args.file}")
        else:
            waypoints = parse_waypoints_string(args.waypoints)
            if waypoints is None:
                return 1
        
        if not waypoints:
            cli.get_logger().error("No waypoints to send")
            return 1
        
        # Send waypoints
        if not cli.send_waypoints(waypoints):
            cli.get_logger().error("Failed to send waypoints")
            return 1
        
        import time
        time.sleep(1.0)  # Wait for upload
        
        # Trigger mission if requested
        if not args.no_trigger:
            cli.trigger_mission()
            cli.get_logger().info("Mission uploaded and triggered")
        
    except KeyboardInterrupt:
        cli.get_logger().info("Interrupted by user")
    except Exception as e:
        cli.get_logger().error(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        rclpy.shutdown()
    
    return 0

if __name__ == '__main__':
    sys.exit(main())

