# PX4 Control Package

This package contains ROS2 nodes for monitoring and controlling PX4 autopilot.

## Nodes

### px4_estimator

A comprehensive status monitoring node that subscribes to multiple PX4 topics and displays the information in a formatted terminal output.

#### Subscribed Topics

**Sensor Related:**
- `/fmu/out/sensor_combined` - IMU data (gyroscope and accelerometer)
- `/fmu/out/vehicle_gps_position` - GPS position data
- `/fmu/out/timesync_status` - Time synchronization status

**Vehicle State:**
- `/fmu/out/vehicle_attitude` - Vehicle attitude (quaternion)
- `/fmu/out/vehicle_local_position` - Local position (NED frame)
- `/fmu/out/vehicle_global_position` - Global position (lat/lon/alt)
- `/fmu/out/vehicle_odometry` - Odometry data

**Control Related:**
- `/fmu/out/vehicle_control_mode` - Current control mode
- `/fmu/out/manual_control_setpoint` - Manual control inputs
- `/fmu/out/position_setpoint_triplet` - Position setpoints

**Health Status:**
- `/fmu/out/vehicle_status` - Vehicle status (armed, nav_state, etc.)
- `/fmu/out/estimator_status_flags` - EKF estimator status
- `/fmu/out/failsafe_flags` - Failsafe flags
- `/fmu/out/battery_status` - Battery status
- `/fmu/out/vehicle_command_ack` - Command acknowledgments

#### Usage

**Method 1: Using launch file**
```bash
cd ~/px4_sitl/px4_ros2_ws
source install/setup.bash
ros2 launch px4_control px4_estimator.launch.py
```

**Method 2: Direct execution**
```bash
cd ~/px4_sitl/px4_ros2_ws
source install/setup.bash
ros2 run px4_control px4_estimator
```

The node will display a real-time status dashboard that updates every 500ms, showing:
- Sensor data (IMU, GPS, time sync)
- Vehicle state (attitude, position, velocity)
- Control information (control modes, manual inputs, setpoints)
- Health status (vehicle status, estimator flags, failsafe, battery)

#### DDS QoS Configuration

The node uses optimized DDS QoS profiles according to [PX4 uXRCE-DDS documentation](https://docs.px4.io/v1.15/en/middleware/uxrce_dds.html):

- **Sensor Data QoS** (sensor_combined, vehicle_gps_position):
  - Reliability: BEST_EFFORT (loss-tolerant, high-frequency data)
  - History: KEEP_LAST with depth 5
  - Suitable for high-frequency sensor streams where occasional loss is acceptable

- **Status/Control QoS** (vehicle_status, vehicle_control_mode, failsafe_flags, etc.):
  - Reliability: BEST_EFFORT (matching PX4 publisher QoS)
  - Durability: TRANSIENT_LOCAL (messages persist for late-joining subscribers)
  - History: KEEP_LAST with depth 10
  - Matches PX4's default QoS settings to ensure proper topic matching

- **Default QoS** (vehicle_attitude, vehicle_local_position, etc.):
  - Reliability: BEST_EFFORT
  - History: KEEP_LAST with depth 10
  - Used for high-frequency state updates

This configuration ensures optimal data delivery according to message importance and frequency.

Press Ctrl+C to exit.

### px4_control

A control node that provides an interface for controlling PX4 vehicles, including:
- **Arming/Disarming**: Unlock and lock the vehicle motors
- **Mode Switching**: Switch between different flight modes (Offboard, Auto Takeoff, Auto Land, Auto Loiter/Hold)
- **Offboard Control**: Position and velocity control in Offboard mode

#### Published Topics

- `/fmu/in/vehicle_command` - Vehicle commands (ARM, DISARM, mode switching, takeoff, land)
- `/fmu/in/offboard_control_mode` - Offboard control mode (must be published before entering Offboard mode)
- `/fmu/in/trajectory_setpoint` - Trajectory setpoints for position/velocity control in Offboard mode

#### Subscribed Topics

- `/fmu/out/vehicle_status` - Vehicle status (armed state, navigation state)
- `/fmu/out/vehicle_command_ack` - Command acknowledgments
- `/fmu/out/vehicle_local_position` - Local position for maintaining altitude during velocity control

#### Public API

**Arming/Disarming:**
```cpp
void arm();           // Arm (unlock) the vehicle
void disarm();         // Disarm (lock) the vehicle
```

**Mode Switching:**
```cpp
void set_mode_offboard();     // Switch to Offboard mode
void set_mode_land();         // Switch to AUTO.LAND mode
void set_mode_hold();         // Switch to AUTO.LOITER (HOLD) mode
```

**Takeoff:**
```cpp
void arm_and_takeoff();  // Arm then switch to AUTO.TAKEOFF mode
```

**Offboard Control:**
```cpp
// Position control (NED frame: z is negative for altitude)
void publish_position_setpoint(float x, float y, float z, float yaw = 0.0f);

// Velocity control (NED frame)
void publish_velocity_setpoint(float vx, float vy, float vz, float yaw = 0.0f);

// Advanced: Custom offboard control mode and trajectory setpoint
void publish_offboard_control_mode(bool position, bool velocity, bool acceleration, 
                                    bool attitude, bool body_rate);
void publish_trajectory_setpoint(float x, float y, float z, 
                                 float vx, float vy, float vz,
                                 float yaw, float yawspeed);
```

#### Usage Example

**Method 1: Using launch file**
```bash
cd ~/px4_sitl/px4_ros2_ws
source install/setup.bash
ros2 launch px4_control px4_control.launch.py
```

**Method 2: Direct execution**
```bash
cd ~/px4_sitl/px4_ros2_ws
source install/setup.bash
ros2 run px4_control px4_control
```

**Method 3: Using CLI script**
```bash
# Arm and takeoff
python3 src/px4_control/scripts/px4_control_cli.py arm_and_takeoff

# Switch to offboard mode
python3 src/px4_control/scripts/px4_control_cli.py set_mode_offboard

# Send position setpoint (hover at 5m altitude)
python3 src/px4_control/scripts/px4_control_cli.py position 0 0 -5 0

# Send velocity setpoint (move forward at 1 m/s)
python3 src/px4_control/scripts/px4_control_cli.py velocity 1 0 0 0

# Switch to land mode
python3 src/px4_control/scripts/px4_control_cli.py set_mode_land

# Switch to hold mode
python3 src/px4_control/scripts/px4_control_cli.py set_mode_hold
```

**Method 4: Using in your own code**
```cpp
#include "px4_control/px4_control.hpp"

auto control_node = std::make_shared<PX4ControlNode>();

// Switch to Offboard mode
control_node->set_mode_offboard();

// Arm the vehicle
control_node->arm();

// Publish position setpoint (hover at 5m altitude)
control_node->publish_position_setpoint(0.0f, 0.0f, -5.0f, 0.0f);

// Publish velocity setpoint (move forward at 1 m/s)
control_node->publish_velocity_setpoint(1.0f, 0.0f, 0.0f, 0.0f);
```

#### Important Notes

1. **Offboard Mode Requirements**: 
   - Before entering Offboard mode, you must publish `/fmu/in/offboard_control_mode` at least at 2Hz
   - The node automatically maintains this publication when `offboard_mode_active_` is true
   - After switching to Offboard mode, wait a few seconds before sending setpoints

2. **Coordinate Frame**: 
   - All positions and velocities use NED (North-East-Down) frame
   - Z-axis is negative for altitude (e.g., -5.0 means 5 meters above ground)
   - Yaw angle is in radians, range [-PI, PI]

3. **Arming Requirements**:
   - Vehicle must be in a safe state (not in failsafe)
   - Pre-flight checks must pass
   - For Offboard mode, vehicle must be armed before entering

4. **Mode Switching**:
   - Mode changes may take a few seconds to take effect
   - Check `/fmu/out/vehicle_status` to verify mode changes

### joystick_bridge

A node that reads input from USB joystick/gamepad devices and publishes to ROS2 `/joy` topic.

#### Published Topics

- `/joy` (sensor_msgs/msg/Joy) - Joystick data with axes and buttons

#### Usage

**Using launch file (recommended):**
```bash
cd ~/px4_sitl/px4_ros2_ws
source install/setup.bash
ros2 launch px4_control px4_joystick.launch.py
```

**Direct execution:**
```bash
ros2 run px4_control joystick_bridge
```

**Parameters:**
- `device`: Joystick device path (default: `/dev/input/js0`)
- `publish_rate`: Publishing rate in Hz (default: `50.0`)

### px4_joystick

A converter node that subscribes to `/joy` topic and converts joystick input to PX4 `manual_control_input` messages for controlling the vehicle.

#### Subscribed Topics

- `/joy` (sensor_msgs/msg/Joy) - Joystick input data

#### Published Topics

- `/fmu/in/manual_control_input` (px4_msgs/msg/ManualControlSetpoint) - Manual control commands sent to PX4

#### Usage

**Using launch file (recommended):**
```bash
cd ~/px4_sitl/px4_ros2_ws
source install/setup.bash
ros2 launch px4_control px4_joystick.launch.py
```

**Direct execution:**
```bash
ros2 run px4_control px4_joystick
```

**Parameters:**
- `roll_axis`: Joystick axis index for roll control (default: `0`)
- `pitch_axis`: Joystick axis index for pitch control (default: `1`)
- `throttle_axis`: Joystick axis index for throttle control (default: `2`)
- `yaw_axis`: Joystick axis index for yaw control (default: `3`)
- `roll_inverted`: Invert roll axis (default: `false`)
- `pitch_inverted`: Invert pitch axis (default: `false`)
- `throttle_inverted`: Invert throttle axis (default: `false`)
- `yaw_inverted`: Invert yaw axis (default: `false`)
- `dead_zone`: Dead zone threshold (default: `0.05`)

#### Joystick Control Setup

**1. Check joystick device:**
```bash
ls -la /dev/input/js*
```

**2. Set device permissions (if needed):**
```bash
sudo chmod 666 /dev/input/js0
# Or add user to input group:
sudo usermod -a -G input $USER
```

**3. Configure PX4:**
- Set `COM_RC_IN_MODE = 1` (Joystick) in QGroundControl or via MAVLink
- Ensure PX4 is in a manual control mode (Manual/Stabilized/Position/Altitude)

**Important Note about QGroundControl Joystick:**
QGroundControl has its own joystick driver that directly reads from `/dev/input/js*` devices, independent of ROS2. This means:
- QGC's joystick calibration page shows QGC's own joystick readings, not ROS2 data
- Both QGC and ROS2 can access the joystick simultaneously
- To use only ROS2 joystick control:
  1. In QGC, go to **Vehicle Setup > Joystick**
  2. Disconnect/disable the joystick in QGC settings
  3. Or simply don't configure joystick in QGC - PX4 will use ROS2 input when `COM_RC_IN_MODE = 1`
- The ROS2 joystick data is sent via `/fmu/in/manual_control_input` topic with `data_source = SOURCE_MAVLINK_0`
- PX4's `ManualControlSelector` will prioritize the ROS2 input when properly configured

**4. Verify data flow:**
```bash
# Check joystick data
ros2 topic echo /joy

# Check PX4 input
ros2 topic echo /fmu/in/manual_control_input

# Check PX4 output
ros2 topic echo /fmu/out/manual_control_setpoint
```

**5. Customize axis mapping:**
```bash
ros2 launch px4_control px4_joystick.launch.py \
    roll_axis:=0 \
    pitch_axis:=1 \
    throttle_axis:=2 \
    yaw_axis:=3 \
    roll_inverted:=false \
    dead_zone:=0.05
```

#### Troubleshooting

- **No joystick detected**: Check `lsusb` and `/dev/input/js*`
- **Permission denied**: Run `sudo chmod 666 /dev/input/js0` or add user to input group
- **PX4 not responding**: 
  - Verify `COM_RC_IN_MODE = 1`
  - Check that `/fmu/in/manual_control_input` has data
  - Ensure PX4 is armed and in correct flight mode
- **Wrong axis direction**: Use `*_inverted` parameters to reverse axes
- **Wrong axis mapping**: Check `/joy` topic to see actual axis indices and adjust mapping

## One-Click Startup Script

A convenient script to launch all required components for PX4 SITL simulation with ROS2 control.

### Usage

```bash
cd ~/px4_sitl/px4_ros2_ws
./src/px4_control/sh/px4_control.sh
```

This script will automatically launch:
1. **Gazebo Simulator** - Starts the Gazebo simulation environment
2. **PX4 SITL** - Compiles and launches PX4 Software-In-The-Loop
3. **MicroXRCEAgent** - DDS agent for communication between PX4 and ROS2
4. **px4_estimator** - Status monitoring node
5. **px4_control** - Control node for vehicle commands
6. **px4_joystick** - Joystick bridge and converter nodes

Each component runs in a separate terminal tab, with appropriate delays to ensure proper startup order.

**Note**: Make sure you have sourced the ROS2 workspace before running the script:
```bash
source install/setup.bash
```

### Mission Planning System

A waypoint mission planning system that allows you to upload and execute waypoint missions on PX4.

#### Architecture Overview

The mission planning system uses a three-component architecture:

```
┌─────────────────────┐      ┌──────────────────┐
│  px4_mission_cli.py  │      │   mqtt_mission   │  MQTT bridge
│   (Python Script)   │      │   (C++ Node)     │
└──────────┬──────────┘      └────────┬─────────┘
           │                          │
           ├─→ /mission/waypoints ───┼──→ (Float32MultiArray)
           │         ↓                │
           │  ┌──────────────────────┐ │
           │  │  mavlink_mission.py  │ │  MAVLink uploader
           │  │    (ROS2 Node)       │ │
           │  └──────────┬───────────┘ │
           │             │             │
           │             └─→ MAVLink Protocol → PX4 dataman
           │
           ├─→ /mission/trigger (Bool) ───┐
           │                                │
           └─→ /px4_mission/trigger ────────┼──→ (Bool)
                                            ↓
                                    ┌──────────────────┐
                                    │   px4_mission    │  Mission executor & monitor
                                    │   (C++ Node)     │
                                    └────────┬─────────┘
                                             │
                                             ├─→ /fmu/in/vehicle_command → PX4 execution
                                             │
                                             └─→ /px4_mission/state (JSON) → mqtt_mission → MQTT
```

#### Component Details

**1. px4_mission_cli.py** (CLI Tool)
- **Location**: `scripts/px4_mission_cli.py`
- **Purpose**: Command-line interface for sending waypoint missions
- **Input**: Command-line arguments (waypoints string or JSON file)
- **Output**: 
  - Publishes waypoints to `/mission/waypoints` topic
  - Publishes trigger signal to `/mission/trigger` topic
- **Features**:
  - Parse waypoints from command line string or JSON file
  - Validate waypoint format (9 parameters per waypoint)
  - Optional mission trigger (use `--no-trigger` to upload only)

**2. mavlink_mission.py** (MAVLink Uploader)
- **Location**: `scripts/mavlink_mission.py`
- **Purpose**: Receive waypoints via ROS2 and upload to PX4 via MAVLink Mission Protocol
- **Subscribed Topics**:
  - `/mission/waypoints` (std_msgs/Float32MultiArray): Waypoint data
- **MAVLink Connection**: 
  - Default: `udp:127.0.0.1:14540` (configurable via ROS2 parameter)
  - Uses `pymavlink` library for MAVLink communication
- **Protocol Flow**:
  1. Clear existing mission (`MISSION_CLEAR_ALL`)
  2. Send mission count (`MISSION_COUNT`)
  3. Receive mission request (`MISSION_REQUEST`)
  4. Send waypoints one by one (`MISSION_ITEM`)
  5. Receive final acknowledgment (`MISSION_ACK`)
- **Thread Safety**: Uses separate thread for upload to avoid blocking ROS2 callbacks

**3. px4_mission** (Mission Executor)
- **Location**: 
  - Header: `include/px4_mission/px4_mission.hpp`
  - Implementation: `utils/px4_mission.cpp`
  - Node wrapper: `utils/px4_mission_node.cpp`
- **Purpose**: Execute missions and monitor mission status
- **Subscribed Topics**:
  - `/px4_mission/trigger` (std_msgs/Bool): Mission trigger signal (from mqtt_mission or other nodes)
  - `/mission/trigger` (std_msgs/Bool): Legacy mission trigger signal (backward compatibility)
  - `/mission/count` (std_msgs/UInt16): Total waypoint count from mavlink_mission
  - `/mission/current_waypoint` (std_msgs/UInt16): Current waypoint index from mavlink_mission
  - `/fmu/out/vehicle_status` (px4_msgs/VehicleStatus): Vehicle state
  - `/fmu/out/vehicle_global_position` (px4_msgs/VehicleGlobalPosition): GPS position
  - `/fmu/out/vehicle_command_ack` (px4_msgs/VehicleCommandAck): Command acknowledgments
- **Published Topics**:
  - `/fmu/in/vehicle_command` (px4_msgs/VehicleCommand): Vehicle commands
  - `/px4_mission/state` (std_msgs/String): Mission state in JSON format (for mqtt_mission bridge)
- **State Machine**:
  ```
  IDLE → ARMING → MISSION_ACTIVE → MISSION_COMPLETE → IDLE
  ```
- **Execution Flow**:
  1. **IDLE**: Wait for mission trigger (status not displayed)
  2. **ARMING**: 
     - Wait for mission availability (checks mission_count or 3s timeout for MAVLink upload)
     - Set mode to AUTO_MISSION (nav_state=3)
     - Arm vehicle
     - Start mission execution
  3. **MISSION_ACTIVE**: 
     - Monitor mission progress via `/mission/count` and `/mission/current_waypoint` topics
     - Display status: State, Nav mode, Armed status, Waypoint progress
     - Detect completion (current_waypoint >= total_waypoints or nav_state changes to 4=HOLD, 5=RTL, or 6=LAND)
  4. **MISSION_COMPLETE**: 
     - Mission completed, return to IDLE
     - No automatic RTL (user can set RTL waypoint as last waypoint in mission)
  5. **IDLE**: Reset and wait for next mission

#### File Structure

```
px4_control/
├── scripts/
│   ├── px4_mission_cli.py      # CLI tool for sending missions
│   └── mavlink_mission.py       # MAVLink mission uploader
├── include/
│   └── px4_mission/
│       └── px4_mission.hpp      # Mission executor header
├── utils/
│   ├── px4_mission.cpp          # Mission executor implementation
│   └── px4_mission_node.cpp     # ROS2 node wrapper
└── launch/
    └── px4_mission.launch.py    # Launch file for mission system
```

#### Installation

Install required dependencies:
```bash
pip3 install pymavlink
```

#### Usage

**Step 1: Start the mission system**
```bash
cd ~/px4_sitl/px4_ros2_ws
source install/setup.bash
ros2 launch px4_control px4_mission.launch.py
```

This launches:
- `mavlink_mission.py`: Waits for waypoints on `/mission/waypoints`, uploads via MAVLink, publishes `/mission/count` and `/mission/current_waypoint`
- `px4_mission`: Waits for trigger on `/px4_mission/trigger` (or `/mission/trigger` for backward compatibility), publishes `/px4_mission/state`

**Step 2: Send a mission**

```bash
# Using JSON file
ros2 run px4_control px4_mission_cli.py --file examples/example_mission.json

# Using command line waypoints (format: "wp1|wp2|wp3|...")
ros2 run px4_control px4_mission_cli.py --waypoints \
  "3,16,47.397742,8.545594,5.0,0.0,1.0,0.0,0.0|3,16,47.397842,8.545594,5.0,0.0,1.0,0.0,0.0"

# Upload only (don't trigger execution)
ros2 run px4_control px4_mission_cli.py --file examples/example_mission.json --no-trigger
```

**Alternative: Using MQTT (requires `mqtt_mission` node)**

```bash
# Start mqtt_mission node
ros2 launch px4_mqtt mqtt_mission.launch.py

# Send mission via MQTT (using mosquitto_pub)
mosquitto_pub -h <broker> -p <port> -u <username> -P <password> \
  -t "uavmission/waypoints/uav1" \
  -m '{"waypoints": [[3,16,47.397742,8.545594,5.0,0.0,1.0,0.0,0.0]], "auto_trigger": true}'
```

**Step 3: Monitor mission execution**

The `px4_mission` node will:
- Display status updates every 2 seconds (only when mission is active)
- Show current state, navigation mode, armed status, waypoint progress
- Status display is suppressed in IDLE state to reduce clutter
- Mission completion returns to IDLE (no automatic RTL - set RTL waypoint as last waypoint if needed)

#### Waypoint Format

Each waypoint consists of 9 parameters:
```
[frame, command, lat, lon, alt, param1, param2, param3, param4]
```

- **frame**: Mission frame (3 = FRAME_GLOBAL_REL_ALT)
- **command**: MAVLink command (16 = NAV_WAYPOINT)
- **lat, lon**: Latitude and longitude in degrees
- **alt**: Altitude in meters (relative to home)
- **param1**: Hold time in seconds
- **param2**: Acceptance radius in meters
- **param3**: Pass radius in meters (0 = pass through)
- **param4**: Yaw angle in radians (0 = no change)

#### JSON Mission File Format

```json
{
  "waypoints": [
    {
      "frame": 3,
      "command": 16,
      "lat": 47.397742,
      "lon": 8.545594,
      "alt": 5.0,
      "param1": 0.0,
      "param2": 1.0,
      "param3": 0.0,
      "param4": 0.0
    }
  ]
}
```

#### Execution Flow Details

**Mission Upload Flow:**
1. User runs `px4_mission_cli.py` with waypoints
2. CLI parses and validates waypoints
3. CLI publishes waypoints to `/mission/waypoints`
4. `mavlink_mission.py` receives waypoints
5. MAVLink uploader connects to PX4 via UDP
6. Clears existing mission
7. Uploads waypoints via MAVLink Mission Protocol
8. Stores mission in PX4 dataman

**Mission Execution Flow:**
1. CLI publishes trigger (`true`) to `/mission/trigger`
2. `px4_mission` receives trigger, transitions to ARMING state
3. Waits for mission availability (checks `/mission/count` topic or 3s timeout for MAVLink upload)
4. Sets mode to AUTO_MISSION (nav_state=3)
5. Arms vehicle if not already armed
6. Sends `VEHICLE_CMD_MISSION_START`
7. Transitions to MISSION_ACTIVE state
8. Monitors mission progress via `/mission/count` and `/mission/current_waypoint` topics from mavlink_mission
9. Detects completion when:
   - `current_waypoint >= total_waypoints`, OR
   - `nav_state` changes from 3 (AUTO_MISSION) to 4 (HOLD), 5 (RTL), or 6 (LAND)
10. Transitions to MISSION_COMPLETE state
11. Returns to IDLE state (no automatic RTL - user can set RTL waypoint as last waypoint)
12. Publishes mission state to `/px4_mission/state` topic (JSON format) for MQTT bridge

#### ROS2 Topics

**Published Topics:**
- `/mission/waypoints` (std_msgs/Float32MultiArray): Waypoint data from CLI or mqtt_mission
- `/mission/trigger` (std_msgs/Bool): Mission trigger signal from CLI (legacy, backward compatibility)
- `/px4_mission/trigger` (std_msgs/Bool): Mission trigger signal from mqtt_mission or other nodes
- `/mission/count` (std_msgs/UInt16): Total waypoint count from mavlink_mission
- `/mission/current_waypoint` (std_msgs/UInt16): Current waypoint index from mavlink_mission
- `/px4_mission/state` (std_msgs/String): Mission state in JSON format (for mqtt_mission bridge)
- `/fmu/in/vehicle_command` (px4_msgs/VehicleCommand): Vehicle commands from px4_mission

**Subscribed Topics:**
- `/mission/waypoints` (std_msgs/Float32MultiArray): Waypoint data (subscribed by mavlink_mission)
- `/px4_mission/trigger` (std_msgs/Bool): Mission trigger (subscribed by px4_mission, primary)
- `/mission/trigger` (std_msgs/Bool): Mission trigger (subscribed by px4_mission, legacy for backward compatibility)
- `/mission/count` (std_msgs/UInt16): Total waypoint count (subscribed by px4_mission)
- `/mission/current_waypoint` (std_msgs/UInt16): Current waypoint index (subscribed by px4_mission)
- `/px4_mission/state` (std_msgs/String): Mission state (subscribed by mqtt_mission)
- `/fmu/out/vehicle_status` (px4_msgs/VehicleStatus): Vehicle state (subscribed by px4_mission)
- `/fmu/out/vehicle_global_position` (px4_msgs/VehicleGlobalPosition): GPS position (subscribed by px4_mission)
- `/fmu/out/vehicle_command_ack` (px4_msgs/VehicleCommandAck): Command acknowledgments (subscribed by px4_mission)

#### Important Notes

- **MAVLink Protocol**: Waypoints are uploaded via MAVLink Mission Protocol to PX4's dataman storage
- **ROS2 Commands**: Mission execution is controlled via ROS2 VehicleCommand interface
- **Automatic Mode Switching**: System automatically switches to AUTO_MISSION mode and arms vehicle
- **No Automatic RTL**: System does not automatically trigger RTL after mission completion. User can set RTL waypoint as the last waypoint in the mission if needed.
- **MQTT Integration**: The system integrates with `mqtt_mission` node from `px4_mqtt` package:
  - `mqtt_mission` publishes waypoints to `/mission/waypoints` and trigger to `/px4_mission/trigger`
  - `px4_mission` publishes state to `/px4_mission/state` for `mqtt_mission` to forward to MQTT
- **Backward Compatibility**: `px4_mission` subscribes to both `/px4_mission/trigger` (primary) and `/mission/trigger` (legacy) for backward compatibility
- **QoS Configuration**: 
  - `/mission/waypoints` uses Reliable QoS (matching mavlink_mission subscription)
  - `/px4_mission/trigger` uses BestEffort QoS (matching px4_mission subscription)
  - `/px4_mission/state` uses default QoS (BestEffort)
- **GPS Requirement**: GPS must be available and home position must be set before sending missions
- **Status Display**: Status information is displayed at 2Hz when mission is active (ARMING, MISSION_ACTIVE, MISSION_COMPLETE). IDLE state is suppressed to reduce output clutter.
- **MAVLink Connection**: Default is `udp:127.0.0.1:14540` (configurable via ROS2 parameter `mavlink_connection`)
- **Mission Progress Tracking**: Uses `/mission/count` and `/mission/current_waypoint` topics from `mavlink_mission` instead of `mission_result` topic (which may not be available in all PX4 configurations)

#### Troubleshooting

- **No waypoints received**: 
  - Check that `mavlink_mission.py` is running: `ros2 node list | grep mavlink_mission`
  - Verify waypoints are published: `ros2 topic echo /mission/waypoints`
  
- **Mission not uploading**: 
  - Check MAVLink connection: Verify PX4 is running and listening on `udp:127.0.0.1:14540`
  - Check logs: `ros2 topic echo /rosout | grep mavlink_mission`
  
- **Mission not starting**: 
  - Ensure GPS is available: Check `/fmu/out/vehicle_global_position`
  - Verify mission was uploaded: Check `/mission/count` topic (should show total waypoints)
  - Check MAVLink connection: Verify `mavlink_mission.py` is connected and uploaded successfully
  - Ensure vehicle is not in failsafe mode
  - Verify trigger was sent: Check `/px4_mission/trigger` or `/mission/trigger` topic
  
- **Status not updating**: 
  - Verify `px4_mission` node is running: `ros2 node list | grep px4_mission`
  - Check PX4 topics are available: `ros2 topic list | grep fmu`

## File Structure

```
px4_control/
├── include/
│   ├── px4_control/
│   │   ├── px4_estimator.hpp         # Estimator header file
│   │   └── px4_control.hpp           # Control node header file
│   └── px4_mission/
│       └── px4_mission.hpp            # PX4 mission header file
│   └── px4_joystick/
│       ├── joystick_bridge.hpp       # Joystick bridge header file
│       └── px4_joystick.hpp          # PX4 joystick converter header file
├── src/
│   ├── px4_estimator.cpp             # Estimator implementation
│   ├── px4_estimator_node.cpp       # Estimator main entry point
│   ├── px4_control.cpp               # Control node implementation
│   └── px4_control_node.cpp         # Control node main entry point
├── utils/
│   ├── joystick_bridge.cpp           # Joystick bridge implementation
│   └── px4_joystick.cpp              # PX4 joystick converter implementation
│   ├── px4_mission.cpp               # PX4 mission implementation
│   └── px4_mission_node.cpp          # PX4 mission main entry point
├── launch/
│   ├── px4_estimator.launch.py      # Estimator launch file
│   ├── px4_control.launch.py        # Control node launch file
│   ├── px4_joystick.launch.py      # Complete joystick control launch file
│   └── px4_mission.launch.py        # PX4 mission planning system launch file
├── scripts/
│   ├── px4_control_cli.py           # Command-line interface script
│   ├── px4_mission_cli.py           # Mission command-line tool
│   └── mavlink_mission.py            # MAVLink mission uploader node
├── examples/
│   └── example_mission.json         # Example waypoint mission file
├── sh/
│   └── px4_control.sh                # One-click startup script
├── CMakeLists.txt
└── package.xml
```
