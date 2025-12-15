#!/usr/bin/env python3
"""
PX4 Control Command Line Interface
Provides arm/disarm, arm+takeoff, and offboard control (position/velocity)
"""

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint
import sys
import time
import math

class PX4ControlCLI(Node):
    def __init__(self):
        super().__init__('px4_control_cli')
        
        # Create publishers
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        
        # Wait for publishers to be ready
        time.sleep(0.5)
    
    def get_timestamp(self):
        return int(self.get_clock().now().nanoseconds / 1000)
    
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0, 
                                 param5=0.0, param6=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_timestamp()
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.confirmation = 0
        self.vehicle_command_pub.publish(msg)
    
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
    
    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
    
    def set_mode_auto_takeoff(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 2.0)
    
    def set_mode_land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 6.0)
    
    def set_mode_hold(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 3.0)
    
    def arm_and_takeoff(self):
        self.arm()
        time.sleep(3)
        self.set_mode_auto_takeoff()
    
    def set_mode_offboard(self):
        current_pos = [0.0, 0.0, 0.0]
        current_yaw = 0.0
        
        try:
            from px4_msgs.msg import VehicleLocalPosition
            pos_sub = self.create_subscription(
                VehicleLocalPosition, '/fmu/out/vehicle_local_position', 
                lambda msg: None, 10)
            time.sleep(0.1)
        except:
            pass
        
        for i in range(20):
            self.publish_offboard_control_mode(position=True, velocity=False)
            traj_msg = TrajectorySetpoint()
            traj_msg.timestamp = self.get_timestamp()
            traj_msg.position = current_pos
            traj_msg.velocity = [float('nan'), float('nan'), float('nan')]
            traj_msg.acceleration = [float('nan'), float('nan'), float('nan')]
            traj_msg.yaw = current_yaw
            self.trajectory_setpoint_pub.publish(traj_msg)
            time.sleep(0.05)
        
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0, 0.0)
    
    def publish_offboard_control_mode(self, position=True, velocity=False):
        msg = OffboardControlMode()
        msg.timestamp = self.get_timestamp()
        msg.position = position
        msg.velocity = velocity
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_pub.publish(msg)
    
    def publish_position_setpoint(self, x, y, z, yaw=0.0):
        # Publish to node's command topic so it can continuously publish
        # The node will receive this and update its internal setpoint, then continuously publish
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_timestamp()
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float(yaw)
        msg.yawspeed = 0.0
        
        command_pub = self.create_publisher(TrajectorySetpoint, '/px4_control/trajectory_setpoint_command', 10)
        time.sleep(0.1)
        command_pub.publish(msg)
    
    def publish_velocity_setpoint(self, vx, vy, vz, yaw=0.0):
        # Publish to node's command topic so it can continuously publish
        # The node will receive this and update its internal setpoint, then continuously publish
        msg = TrajectorySetpoint()
        msg.timestamp = self.get_timestamp()
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float(yaw)
        msg.yawspeed = 0.0
        
        command_pub = self.create_publisher(TrajectorySetpoint, '/px4_control/trajectory_setpoint_command', 10)
        time.sleep(0.1)
        command_pub.publish(msg)

def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python3 px4_control_cli.py <command> [args...]")
        print("\nCommands:")
        print("  arm                    # Arm (unlock) the vehicle")
        print("  disarm                 # Disarm (lock) the vehicle")
        print("  arm_and_takeoff        # Arm then switch to AUTO.TAKEOFF mode")
        print("  set_mode_offboard      # Switch to OFFBOARD mode")
        print("  set_mode_land          # Switch to AUTO.LAND mode")
        print("  set_mode_hold          # Switch to AUTO.LOITER (HOLD) mode")
        print("  position <x> <y> <z> [yaw]  # Position setpoint (NED frame)")
        print("  velocity <vx> <vy> <vz> [yaw]  # Velocity setpoint (NED frame)")
        print("\nExamples:")
        print("  python3 px4_control_cli.py arm")
        print("  python3 px4_control_cli.py arm_and_takeoff")
        print("  python3 px4_control_cli.py set_mode_offboard")
        print("  python3 px4_control_cli.py set_mode_land          # Land the vehicle")
        print("  python3 px4_control_cli.py set_mode_hold         # Hold current position")
        print("  python3 px4_control_cli.py position 0 0 -5 0    # Hover at 5m altitude")
        print("  python3 px4_control_cli.py velocity 1 0 0 0     # Move forward at 1 m/s")
        return
    
    rclpy.init()
    cli = PX4ControlCLI()
    
    command = sys.argv[1]
    
    try:
        if command == 'arm':
            cli.arm()
        elif command == 'disarm':
            cli.disarm()
        elif command == 'arm_and_takeoff':
            cli.arm_and_takeoff()
        elif command == 'set_mode_offboard':
            cli.set_mode_offboard()
        elif command == 'set_mode_land':
            cli.set_mode_land()
        elif command == 'set_mode_hold':
            cli.set_mode_hold()
        elif command == 'position':
            if len(sys.argv) < 5:
                print("Usage: position <x> <y> <z> [yaw]")
                return
            x, y, z = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
            yaw = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
            
            cli.publish_position_setpoint(x, y, z, yaw)
        elif command == 'velocity':
            if len(sys.argv) < 5:
                print("Usage: velocity <vx> <vy> <vz> [yaw]")
                return
            vx, vy, vz = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
            yaw = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
            cli.publish_velocity_setpoint(vx, vy, vz, yaw)
        else:
            print(f"Unknown command: {command}")
            print("Available commands: arm, disarm, arm_and_takeoff, set_mode_offboard, set_mode_land, set_mode_hold, position, velocity")
            return
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    
    # Give time for message to be published
    time.sleep(0.1)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

