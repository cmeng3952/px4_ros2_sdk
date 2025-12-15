#!/usr/bin/env python3
"""
航点任务上传器：接收航点任务并通过MAVLink协议上传到PX4
订阅: /mission/waypoints (std_msgs/Float32MultiArray)
功能: 通过MAVLink Mission协议将航点上传到PX4的dataman存储
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, UInt16
from pymavlink import mavutil
import threading
import time

class MavlinkMission(Node):
    def __init__(self):
        super().__init__('mavlink_mission')
        
        # MAVLink connection parameters
        self.declare_parameter('mavlink_connection', 'udp:127.0.0.1:14540')
        self.mavlink_connection = self.get_parameter('mavlink_connection').get_parameter_value().string_value
        
        # Create subscriber
        self.waypoints_sub = self.create_subscription(
            Float32MultiArray,
            '/mission/waypoints',
            self.waypoints_callback,
            10
        )
        
        # Create publishers
        self.mission_count_pub = self.create_publisher(
            UInt16,
            '/mission/count',
            10
        )
        self.current_waypoint_pub = self.create_publisher(
            UInt16,
            '/mission/current_waypoint',
            10
        )
        
        # MAVLink connection
        self.master = None
        self.upload_lock = threading.Lock()
        self.is_connected = False
        self.mission_count_ = 0
        self.monitoring_active = False
        
        # Start thread to monitor current waypoint
        threading.Thread(target=self.monitor_current_waypoint, daemon=True).start()
        
        # Connect to MAVLink
        self.connect_mavlink()
        
        self.get_logger().info(f"Mavlink Mission started, connection: {self.mavlink_connection}")
    
    def connect_mavlink(self):
        """连接到MAVLink"""
        try:
            self.master = mavutil.mavlink_connection(self.mavlink_connection)
            
            def wait_heartbeat():
                try:
                    if self.master is None:
                        self.get_logger().error("MAVLink master is None, cannot wait for heartbeat")
                        self.is_connected = False
                        return
                    
                    self.master.wait_heartbeat()
                    self.is_connected = True
                    self.get_logger().info("MAVLink heartbeat received, connected")
                except Exception as e:
                    self.get_logger().error(f"Heartbeat failed: {e}")
                    self.is_connected = False
            
            threading.Thread(target=wait_heartbeat, daemon=True).start()
        except Exception as e:
            self.get_logger().error(f"MAVLink connection failed: {e}")
            self.is_connected = False
            self.master = None
    
    def waypoints_callback(self, msg):
        """接收航点数据并上传到PX4"""
        if len(msg.data) % 9 != 0:
            self.get_logger().error(
                f"Invalid waypoint format! Expected multiple of 9 parameters, got {len(msg.data)}"
            )
            return
        
        # Parse waypoints
        num_waypoints = len(msg.data) // 9
        waypoints = []
        for i in range(num_waypoints):
            wp = [
                msg.data[i * 9 + 0],  # frame
                msg.data[i * 9 + 1],  # command
                msg.data[i * 9 + 2],  # lat
                msg.data[i * 9 + 3],  # lon
                msg.data[i * 9 + 4],  # alt
                msg.data[i * 9 + 5],  # param1
                msg.data[i * 9 + 6],  # param2
                msg.data[i * 9 + 7],  # param3
                msg.data[i * 9 + 8],  # param4
            ]
            waypoints.append(wp)
        
        self.mission_count_ = len(waypoints)
        self.get_logger().info(f"Received {len(waypoints)} waypoints, uploading...")
        threading.Thread(target=self.upload_mission_thread, args=(waypoints,), daemon=True).start()
    
    def upload_mission_thread(self, waypoints):
        """在单独线程中上传任务"""
        with self.upload_lock:
            if not self.is_connected:
                self.connect_mavlink()
                if not self.is_connected:
                    self.get_logger().error("MAVLink not connected")
                    return
            
            try:
                if not self.is_connected:
                    if self.master is None:
                        self.get_logger().error("MAVLink master is None, cannot reconnect")
                        return
                    self.master.wait_heartbeat(timeout=5)
                    self.is_connected = True
                    self.get_logger().info("MAVLink reconnected, heartbeat received")
                
                # Clear existing mission
                self.master.mav.mission_clear_all_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                )
                
                # Wait for acknowledgment (timeout 3s)
                timeout = time.time() + 3.0
                while time.time() < timeout:
                    msg = self.master.recv_match(type='MISSION_ACK', blocking=False, timeout=0.1)
                    if msg and msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                        break
                    elif msg and msg.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                        self.get_logger().error(f"Mission clear failed: {msg.type}")
                        return
                
                # Send mission count
                self.master.mav.mission_count_send(
                    self.master.target_system,
                    self.master.target_component,
                    len(waypoints),
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                )
                
                # Wait for mission request
                timeout = time.time() + 3.0
                while time.time() < timeout:
                    msg = self.master.recv_match(type='MISSION_REQUEST', blocking=False, timeout=0.1)
                    if msg and msg.mission_type == mavutil.mavlink.MAV_MISSION_TYPE_MISSION:
                        break
                else:
                    self.get_logger().error("Mission request timeout")
                    return
                
                # Send waypoints
                for i, wp in enumerate(waypoints):
                    self.master.mav.mission_item_send(
                        self.master.target_system,
                        self.master.target_component,
                        i, int(wp[0]), int(wp[1]), 0, 1,
                        wp[5], wp[6], wp[7], wp[8],
                        wp[2], wp[3], wp[4],
                        mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                    )
                    
                    # Wait for next request or final acknowledgment
                    timeout = time.time() + 3.0
                    while time.time() < timeout:
                        msg = self.master.recv_match(
                            type=['MISSION_REQUEST', 'MISSION_ACK'],
                            blocking=False, timeout=0.1
                        )
                        if msg:
                            if msg.get_type() == 'MISSION_REQUEST' and msg.seq == i + 1:
                                break
                            elif msg.get_type() == 'MISSION_ACK':
                                if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                                    self.get_logger().info("Mission uploaded successfully")
                                    # Publish mission count
                                    count_msg = UInt16()
                                    count_msg.data = len(waypoints)
                                    self.mission_count_pub.publish(count_msg)
                                    return
                                else:
                                    self.get_logger().error(f"Upload failed: {msg.type}")
                                    return
                
                # Wait for final acknowledgment
                timeout = time.time() + 3.0
                while time.time() < timeout:
                    msg = self.master.recv_match(type='MISSION_ACK', blocking=False, timeout=0.1)
                    if msg:
                        if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                            self.get_logger().info("Mission uploaded successfully")
                            # Publish mission count
                            count_msg = UInt16()
                            count_msg.data = len(waypoints)
                            self.mission_count_pub.publish(count_msg)
                            return
                        else:
                            self.get_logger().error(f"Upload failed: {msg.type}")
                            return
                
            except Exception as e:
                self.get_logger().error(f"Upload error: {e}")
    
    def monitor_current_waypoint(self):
        """监控当前航点（通过MAVLink MISSION_CURRENT和MISSION_ITEM_REACHED消息）"""
        current_seq = 0
        self.monitoring_active = True
        while self.monitoring_active:
            try:
                if self.is_connected and self.master is not None:
                    try:
                        # Listen for MISSION_CURRENT (current waypoint being executed)
                        msg_current = self.master.recv_match(type='MISSION_CURRENT', blocking=False, timeout=0.1)
                        if msg_current:
                            current_seq = msg_current.seq
                            current_wp_msg = UInt16()
                            current_wp_msg.data = current_seq
                            self.current_waypoint_pub.publish(current_wp_msg)
                        
                        # Listen for MISSION_ITEM_REACHED (waypoint reached)
                        msg_reached = self.master.recv_match(type='MISSION_ITEM_REACHED', blocking=False, timeout=0.1)
                        if msg_reached:
                            # When a waypoint is reached, the next waypoint becomes current
                            # So if waypoint N is reached, current becomes N+1
                            reached_seq = msg_reached.seq
                            current_seq = reached_seq + 1
                            current_wp_msg = UInt16()
                            current_wp_msg.data = current_seq
                            self.current_waypoint_pub.publish(current_wp_msg)
                            # Waypoint reached, no need to log
                    except Exception as e:
                        # Connection may have been lost
                        # Connection error, silently handle
                        self.is_connected = False
                else:
                    # If not connected and master is None, try to reconnect (but not too frequently)
                    if not self.is_connected and self.master is None:
                        self.connect_mavlink()
                
                time.sleep(0.1)  # Check more frequently for better responsiveness
            except Exception as e:
                # Silently handle errors (connection may not be ready)
                # Monitor thread error, silently handle
                if self.monitoring_active:
                    time.sleep(1.0)

def main(args=None):
    rclpy.init(args=args)
    node = MavlinkMission()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down mavlink_mission node...")
    except Exception as e:
        node.get_logger().error(f"Error in mavlink_mission: {e}")
    finally:
        # Stop monitoring thread
        node.monitoring_active = False
        
        # Destroy node
        try:
            node.destroy_node()
        except Exception as e:
            pass  # Node may already be destroyed
        
        # Shutdown rclpy only if context is still valid
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            pass  # Context may already be shut down

if __name__ == '__main__':
    main()

