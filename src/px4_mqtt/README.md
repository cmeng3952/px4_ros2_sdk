# PX4 MQTT Bridge

ROS2包，用于将PX4飞控的状态数据桥接到MQTT服务器，以及通过MQTT接收控制命令。

## 概述

本包包含三个节点：

1. **`mqtt_estimator`**: 订阅`px4_estimator`节点发布的状态话题，将PX4飞控的状态数据（位置、姿态、电池、GPS等）转换为JSON格式并发布到MQTT服务器。

2. **`mqtt_control`**: 订阅MQTT服务器上的控制命令主题，接收JSON格式的控制命令并转发到PX4飞控。

3. **`mqtt_mission`**: 订阅MQTT服务器上的航线任务主题，接收JSON格式的航线任务并转发到ROS2任务系统；订阅`px4_mission`节点发布的任务状态并转发到MQTT服务器。

## 架构

### 状态数据流（mqtt_estimator）

```
PX4话题 → px4_estimator → /px4_estimator/state → mqtt_estimator → MQTT服务器
```

### 控制命令流（mqtt_control）

```
MQTT服务器 → mqtt_control → px4_control → PX4飞控
```

### 航线任务流（mqtt_mission）

```
MQTT服务器 → mqtt_mission → /mission/waypoints → mavlink_mission → MAVLink协议 → PX4 dataman存储
MQTT服务器 → mqtt_mission → /px4_mission/trigger → px4_mission → PX4飞控执行
PX4飞控 → MAVLink消息 → mavlink_mission → /mission/count, /mission/current_waypoint → px4_mission
px4_mission → /px4_mission/state → mqtt_mission → MQTT服务器
```

`mqtt_control`节点发布命令到`px4_control`节点的话题，由`px4_control`节点转发给PX4飞控，避免直接与PX4通信。

`mqtt_estimator`节点订阅`px4_estimator`发布的状态话题，避免重复订阅PX4话题。

`mqtt_mission`节点与`px4_control`包中的`px4_mission`节点集成，架构与`mqtt_estimator`和`mqtt_control`一致：
- 通过MQTT接收航线任务，发布到`/mission/waypoints`（供`mavlink_mission`通过MAVLink Mission协议上传到PX4的`dataman`存储）和`/px4_mission/trigger`（供`px4_mission`执行任务）
- 订阅`px4_mission`发布的状态话题`/px4_mission/state`，转发到MQTT服务器
- 任务进度由`mavlink_mission`节点监听MAVLink的`MISSION_CURRENT`和`MISSION_ITEM_REACHED`消息获取，并发布到`/mission/count`和`/mission/current_waypoint`话题供`px4_mission`使用

## 依赖

### 系统依赖

```bash
sudo apt-get update
sudo apt-get install libpaho-mqttcpp-dev nlohmann-json3-dev
```

### ROS2依赖

- `rclcpp`
- `std_msgs`
- `px4_msgs`
- `px4_control` (需要`px4_estimator`节点运行，用于`mqtt_estimator`)

## 编译

```bash
cd ~/px4_sitl/px4_ros2_ws
colcon build --packages-select px4_mqtt
source install/setup.bash
```

## 配置

### YAML配置文件

配置文件位于 `config/mqtt_estimator.yaml`，包含以下参数：

**重要提示：** 
- Launch文件会**优先读取源目录**（`src/px4_mqtt/config/`）中的配置文件
- 修改配置文件后**无需重新编译**，直接运行launch文件即可生效
- 如果源目录配置文件不存在，会自动使用安装目录（`install/px4_mqtt/share/px4_mqtt/config/`）中的配置文件

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `uav_name` | 无人机名称/ID | `uav1` |
| `mqtt_broker` | MQTT服务器地址（格式：`tcp://host:port`） | `tcp://localhost:1883` |
| `mqtt_username` | MQTT用户名 | 空 |
| `mqtt_password` | MQTT密码 | 空 |
| `mqtt_state_topic` | MQTT状态主题前缀 | `uavcontrol/state/` |

**注意：** `mqtt://` 前缀会自动转换为 `tcp://`

### 配置文件示例

编辑 `config/mqtt_estimator.yaml`：

```yaml
mqtt_estimator:
  ros__parameters:
    uav_name: "uav1"
    mqtt_broker: "tcp://118.195.156.74:5704"
    mqtt_username: "uav1"
    mqtt_password: "Aa123456."
    mqtt_state_topic: "uavcontrol/state/"
```

## 使用方法

### 方法1：使用YAML配置文件（推荐）

1. 编辑 `src/px4_mqtt/config/mqtt_estimator.yaml` 设置MQTT服务器参数
2. **直接启动节点**（无需重新编译）：

```bash
ros2 launch px4_mqtt mqtt_estimator.launch.py
```

**优势：** 修改配置文件后无需重新编译安装，直接运行launch文件即可生效（类似ROS1的行为）

### 方法2：使用launch参数覆盖配置

```bash
ros2 launch px4_mqtt mqtt_estimator.launch.py \
    uav_name:=uav1 \
    mqtt_broker:=tcp://118.195.156.74:5704 \
    mqtt_username:=uav1 \
    mqtt_password:=Aa123456.
```

### 方法3：直接运行节点

```bash
ros2 run px4_mqtt mqtt_estimator \
    --ros-args \
    -p uav_name:=uav1 \
    -p mqtt_broker:=tcp://118.195.156.74:5704 \
    -p mqtt_username:=uav1 \
    -p mqtt_password:=Aa123456. \
    -p mqtt_state_topic:=uavcontrol/state/
```

## 前提条件

在启动`mqtt_estimator`之前，**必须先启动`px4_estimator`节点**：

```bash
# 终端1：启动px4_estimator
ros2 launch px4_control px4_estimator.launch.py

# 终端2：启动mqtt_estimator
ros2 launch px4_mqtt mqtt_estimator.launch.py
```

## ROS2话题

### 订阅的话题

- `/px4_estimator/state` (`std_msgs/msg/String`)
  - 订阅`px4_estimator`节点发布的JSON格式状态数据
  - 发布频率：1Hz

## MQTT主题

状态数据发布到主题：`{mqtt_state_topic}{uav_name}`

**示例：**
- 如果 `mqtt_state_topic` = `uavcontrol/state/`
- 如果 `uav_name` = `uav1`
- 则完整主题 = `uavcontrol/state/uav1`

## JSON消息格式

发布的JSON消息包含以下字段：

```json
{
  "uav_id": "uav1",
  "uav_name": "uav1",
  "armed": false,
  "arming_state": 1,
  "nav_state": 4,
  "battery_v": 12.6,
  "battery_b": 85.5,
  "battery_current": 2.3,
  "position": [1.2, 3.4, 5.6],
  "velocity": [0.1, 0.2, 0.0],
  "latitude": 39.1234567,
  "longitude": 116.1234567,
  "altitude": 100.5,
  "attitude_rpy": [1.2, 3.4, 45.6],
  "attitude": [0.021, 0.059, 0.796],
  "gps_status": 3,
  "gps_satellites": 12,
  "gps_speed": 0.0,
  "control_mode": {
    "manual": false,
    "position": true,
    "offboard": false,
    "attitude": false
  },
  "estimator": {
    "gps_ok": true,
    "tilt_align": true,
    "yaw_align": true,
    "baro_hgt": true,
    "in_air": false
  },
  "failsafe": {
    "global_pos_invalid": false,
    "local_pos_invalid": false,
    "attitude_invalid": false,
    "battery_unhealthy": false,
    "battery_warning": 0,
    "manual_ctrl_lost": false,
    "gcs_connection_lost": false
  },
  "imu": {
    "gyro": [0.001, 0.002, 0.003],
    "accel": [0.1, 0.2, 9.8]
  },
  "timestamp": 1234567890123456789
}
```

### 字段说明

- `uav_id`, `uav_name`: 无人机标识
- `armed`: 是否解锁（true/false）
- `arming_state`: 解锁状态（0=初始化, 1=未解锁, 2=已解锁）
- `nav_state`: 导航状态
- `battery_v`: 电池电压（V）
- `battery_b`: 电池电量百分比（%）
- `battery_current`: 电池电流（A）
- `position`: 本地位置 [x, y, z]（米）
- `velocity`: 速度 [vx, vy, vz]（米/秒）
- `latitude`, `longitude`, `altitude`: GPS位置（度，米）
- `attitude_rpy`: 姿态欧拉角（度）[roll, pitch, yaw]
- `attitude`: 姿态欧拉角（弧度）[roll, pitch, yaw]
- `gps_status`: GPS定位类型（0=无GPS, 1=无定位, 2=2D定位, 3=3D定位）
- `gps_satellites`: GPS卫星数量
- `gps_speed`: GPS速度（米/秒）
- `control_mode`: 控制模式标志
- `estimator`: 估计器状态标志
- `failsafe`: 故障保护标志
- `imu`: IMU数据（陀螺仪和加速度计）
- `timestamp`: 时间戳（纳秒）

## 故障排除

### MQTT连接失败

1. **检查配置文件**（优先检查源目录）
   ```bash
   # 检查源目录配置文件（实际使用的）
   cat ~/px4_sitl/px4_ros2_ws/src/px4_mqtt/config/mqtt_estimator.yaml
   
   # 检查安装目录配置文件（备用）
   cat ~/px4_sitl/px4_ros2_ws/install/px4_mqtt/share/px4_mqtt/config/mqtt_estimator.yaml
   ```

2. **检查MQTT服务器**
   - 确认服务器地址和端口正确
   - 确认用户名和密码正确
   - 测试网络连接：`ping <mqtt_server_ip>`

3. **查看节点日志**
   ```bash
   ros2 launch px4_mqtt mqtt_estimator.launch.py
   ```

### 没有数据发布到MQTT

1. **检查`px4_estimator`是否运行**
   ```bash
   ros2 node list | grep px4_estimator
   ```

2. **检查状态话题是否有数据**
   ```bash
   ros2 topic echo /px4_estimator/state
   ```

3. **检查话题是否存在**
   ```bash
   ros2 topic list | grep px4_estimator
   ```

4. **检查MQTT连接状态**
   - 查看节点日志中的连接信息
   - 确认MQTT服务器是否收到消息

### 编译错误

1. **检查依赖是否安装**
   ```bash
   dpkg -l | grep -E "paho-mqtt|nlohmann"
   ```

2. **重新安装依赖**
   ```bash
   sudo apt-get install libpaho-mqttcpp-dev nlohmann-json3-dev
   ```

3. **清理并重新编译**
   ```bash
   cd ~/px4_sitl/px4_ros2_ws
   colcon build --packages-select px4_mqtt --cmake-clean-cache
   source install/setup.bash
   ```

---

# MQTT Control 节点

## 概述

`mqtt_control`节点订阅MQTT服务器上的控制命令主题，接收JSON格式的控制命令并转发到PX4飞控。

## 配置

### YAML配置文件

配置文件位于 `config/mqtt_control.yaml`，包含以下参数：

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `uav_name` | 无人机名称/ID | `uav1` |
| `mqtt_broker` | MQTT服务器地址（格式：`tcp://host:port`） | `tcp://localhost:1883` |
| `mqtt_username` | MQTT用户名 | 空 |
| `mqtt_password` | MQTT密码 | 空 |
| `mqtt_command_topic` | MQTT命令主题前缀 | `uavcontrol/command/` |

**注意：** `mqtt://` 前缀会自动转换为 `tcp://`

### 配置文件示例

编辑 `config/mqtt_control.yaml`：

```yaml
mqtt_control:
  ros__parameters:
    uav_name: "uav1"
    mqtt_broker: "tcp://118.195.156.74:5704"
    mqtt_username: "uav1"
    mqtt_password: "Aa123456."
    mqtt_command_topic: "uavcontrol/command/"
```

## 使用方法

### 方法1：使用YAML配置文件（推荐）

1. 编辑 `src/px4_mqtt/config/mqtt_control.yaml` 设置MQTT服务器参数
2. **直接启动节点**（无需重新编译）：

```bash
ros2 launch px4_mqtt mqtt_control.launch.py
```

### 方法2：使用launch参数覆盖配置

```bash
ros2 launch px4_mqtt mqtt_control.launch.py \
    uav_name:=uav1 \
    mqtt_broker:=tcp://118.195.156.74:5704 \
    mqtt_username:=uav1 \
    mqtt_password:=Aa123456.
```

## MQTT主题

控制命令订阅主题：`{mqtt_command_topic}{uav_name}`

**示例：**
- 如果 `mqtt_command_topic` = `uavcontrol/command/`
- 如果 `uav_name` = `uav1`
- 则完整主题 = `uavcontrol/command/uav1`

## 支持的命令

### 基本命令

#### 1. 解锁 (arm)

```json
{
  "command": "arm"
}
```

#### 2. 上锁 (disarm)

```json
{
  "command": "disarm"
}
```

#### 3. 解锁并起飞 (arm_and_takeoff)

```json
{
  "command": "arm_and_takeoff"
}
```

### 模式切换命令

#### 4. 切换到OFFBOARD模式 (set_mode_offboard)

```json
{
  "command": "set_mode_offboard"
}
```

#### 5. 切换到LAND模式 (set_mode_land)

```json
{
  "command": "set_mode_land"
}
```

#### 6. 切换到HOLD模式 (set_mode_hold)

```json
{
  "command": "set_mode_hold"
}
```

### 位置控制命令

#### 7. 位置设定点 (position)

```json
{
  "command": "position",
  "x": 0.0,
  "y": 0.0,
  "z": -5.0,
  "yaw": 0.0
}
```

- `x`, `y`, `z`: 位置坐标（NED坐标系，单位：米）
- `yaw`: 偏航角（可选，默认：0.0，单位：弧度）

**示例：** 悬停在5米高度

```json
{
  "command": "position",
  "x": 0.0,
  "y": 0.0,
  "z": -5.0,
  "yaw": 0.0
}
```

### 速度控制命令

#### 8. 速度设定点 (velocity)

```json
{
  "command": "velocity",
  "vx": 1.0,
  "vy": 0.0,
  "vz": 0.0,
  "yaw": 0.0
}
```

- `vx`, `vy`, `vz`: 速度（NED坐标系，单位：米/秒）
- `yaw`: 偏航角（可选，默认：0.0，单位：弧度）

**示例：** 以1 m/s速度向前移动

```json
{
  "command": "velocity",
  "vx": 1.0,
  "vy": 0.0,
  "vz": 0.0,
  "yaw": 0.0
}
```

## ROS2话题

### 发布的话题

- `/px4_control/vehicle_command` (`px4_msgs/msg/VehicleCommand`)
  - 发布车辆命令到`px4_control`节点（解锁、上锁、模式切换等）
  - `px4_control`节点会转发给PX4飞控

- `/px4_control/trajectory_setpoint_command` (`px4_msgs/msg/TrajectorySetpoint`)
  - 发布轨迹设定点命令到`px4_control`节点（位置/速度控制）
  - `px4_control`节点会转发给PX4飞控并持续发布以维持offboard模式

## 使用示例

### 使用mosquitto_pub发送命令

```bash
# 解锁
mosquitto_pub -h 118.195.156.74 -p 5704 -u uav1 -P Aa123456. \
  -t "uavcontrol/command/uav1" \
  -m '{"command": "arm"}'

# 切换到OFFBOARD模式
mosquitto_pub -h 118.195.156.74 -p 5704 -u uav1 -P Aa123456. \
  -t "uavcontrol/command/uav1" \
  -m '{"command": "set_mode_offboard"}'

# 位置控制：悬停在5米高度
mosquitto_pub -h 118.195.156.74 -p 5704 -u uav1 -P Aa123456. \
  -t "uavcontrol/command/uav1" \
  -m '{"command": "position", "x": 0.0, "y": 0.0, "z": -5.0, "yaw": 0.0}'

# 速度控制：向前移动1 m/s
mosquitto_pub -h 118.195.156.74 -p 5704 -u uav1 -P Aa123456. \
  -t "uavcontrol/command/uav1" \
  -m '{"command": "velocity", "vx": 1.0, "vy": 0.0, "vz": 0.0, "yaw": 0.0}'

# 降落
mosquitto_pub -h 118.195.156.74 -p 5704 -u uav1 -P Aa123456. \
  -t "uavcontrol/command/uav1" \
  -m '{"command": "set_mode_land"}'

# 上锁
mosquitto_pub -h 118.195.156.74 -p 5704 -u uav1 -P Aa123456. \
  -t "uavcontrol/command/uav1" \
  -m '{"command": "disarm"}'
```

### Python示例

```python
import paho.mqtt.client as mqtt
import json

# MQTT配置
broker = "118.195.156.74"
port = 5704
username = "uav1"
password = "Aa123456."
topic = "uavcontrol/command/uav1"

# 创建MQTT客户端
client = mqtt.Client()
client.username_pw_set(username, password)
client.connect(broker, port, 60)

# 发送解锁命令
cmd = {"command": "arm"}
client.publish(topic, json.dumps(cmd))

# 发送位置控制命令
cmd = {
    "command": "position",
    "x": 0.0,
    "y": 0.0,
    "z": -5.0,
    "yaw": 0.0
}
client.publish(topic, json.dumps(cmd))

client.disconnect()
```

## 前提条件

在启动`mqtt_control`之前，**必须先启动`px4_control`节点**（`mqtt_control`通过`px4_control`转发命令）：

```bash
# 终端1：启动px4_control（必需）
ros2 launch px4_control px4_control.launch.py

# 终端2：启动mqtt_control
ros2 launch px4_mqtt mqtt_control.launch.py
```

**注意：** `mqtt_control`节点不直接与PX4飞控通信，所有命令都通过`px4_control`节点转发。

## 注意事项

1. **坐标系**: 所有位置和速度使用NED（North-East-Down）坐标系
   - `z`为负值表示高度（例如：`z: -5.0` 表示5米高度）

---

# MQTT Mission 节点

## 概述

`mqtt_mission`节点订阅MQTT服务器上的航线任务主题，接收JSON格式的航线任务并转发到ROS2任务系统；同时订阅`px4_mission`节点发布的任务状态并转发到MQTT服务器。

**架构说明：**
- 与`mqtt_estimator`和`mqtt_control`的架构一致
- 通过`px4_mission`节点与PX4通信，避免直接与PX4交互
- 航点数据发布到`/mission/waypoints`供`mavlink_mission`上传到PX4
- 任务触发信号发布到`/px4_mission/trigger`供`px4_mission`执行
- 任务状态从`/px4_mission/state`订阅并转发到MQTT

## 配置

### YAML配置文件

配置文件位于 `config/mqtt_mission.yaml`，包含以下参数：

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `uav_name` | 无人机名称/ID | `uav1` |
| `mqtt_broker` | MQTT服务器地址（格式：`tcp://host:port`） | `tcp://localhost:1883` |
| `mqtt_username` | MQTT用户名 | 空 |
| `mqtt_password` | MQTT密码 | 空 |
| `mqtt_mission_topic` | MQTT任务主题前缀 | `uavmission/waypoints/` |
| `mqtt_status_topic` | MQTT状态主题前缀 | `uavmission/status/` |

**注意：** `mqtt://` 前缀会自动转换为 `tcp://`

### 配置文件示例

编辑 `config/mqtt_mission.yaml`：

```yaml
mqtt_mission:
  ros__parameters:
    uav_name: "uav1"
    mqtt_broker: "tcp://118.195.156.74:5704"
    mqtt_username: "uav1"
    mqtt_password: "Aa123456."
    mqtt_mission_topic: "uavmission/waypoints/"
    mqtt_status_topic: "uavmission/status/"
```

## 使用方法

### 方法1：使用YAML配置文件（推荐）

1. 编辑 `src/px4_mqtt/config/mqtt_mission.yaml` 设置MQTT服务器参数
2. **直接启动节点**（无需重新编译）：

```bash
ros2 launch px4_mqtt mqtt_mission.launch.py
```

### 方法2：使用launch参数覆盖配置

```bash
ros2 launch px4_mqtt mqtt_mission.launch.py \
    uav_name:=uav1 \
    mqtt_broker:=tcp://118.195.156.74:5704 \
    mqtt_username:=uav1 \
    mqtt_password:=Aa123456.
```

## MQTT主题

- **任务订阅主题**：`{mqtt_mission_topic}{uav_name}`
- **状态发布主题**：`{mqtt_status_topic}{uav_name}`

**示例：**
- 如果 `mqtt_mission_topic` = `uavmission/waypoints/`
- 如果 `mqtt_status_topic` = `uavmission/status/`
- 如果 `uav_name` = `uav1`
- 则任务主题 = `uavmission/waypoints/uav1`
- 则状态主题 = `uavmission/status/uav1`

## 支持的MQTT消息格式

### 1. 发送航线任务

#### 格式1：数组格式（推荐）

```json
{
  "waypoints": [
    [3, 16, 47.397742, 8.545594, 5.0, 0.0, 1.0, 0.0, 0.0],
    [3, 16, 47.397842, 8.545594, 5.0, 0.0, 1.0, 0.0, 0.0],
    [3, 16, 47.397942, 8.545594, 5.0, 0.0, 1.0, 0.0, 0.0]
  ],
  "auto_trigger": true
}
```

#### 格式2：对象格式

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
    },
    {
      "frame": 3,
      "command": 16,
      "lat": 47.397842,
      "lon": 8.545594,
      "alt": 5.0,
      "param1": 0.0,
      "param2": 1.0,
      "param3": 0.0,
      "param4": 0.0
    }
  ],
  "auto_trigger": false
}
```

**航点参数说明：**
- `frame`: 坐标系类型（0=全局绝对高度, 1=本地NED, 2=任务坐标系, 3=全局相对高度，推荐）
- `command`: MAVLink命令类型（16=NAV_WAYPOINT标准航点）
- `lat`: 纬度（度）
- `lon`: 经度（度）
- `alt`: 高度（米）
- `param1`: 保持时间（秒）
- `param2`: 接受半径（米）
- `param3`: 通过半径（米，0=直接通过）
- `param4`: 偏航角（弧度，0=不改变）

**`auto_trigger`**: 是否自动触发任务执行（默认：`false`）

### 2. 触发任务执行

```json
{
  "trigger": true
}
```

或

```json
{
  "action": "start"
}
```

## 任务状态消息格式

节点会实时转发`px4_mission`发布的任务状态到MQTT（2Hz更新频率）：

```json
{
  "uav_name": "uav1",
  "state": "MISSION_ACTIVE",
  "mission_triggered": true,
  "armed": true,
  "arming_state": 2,
  "nav_state": 3,
  "total_waypoints": 3,
  "current_waypoint": 2,
  "last_reached_waypoint": 1,
  "mission_count_received": true,
  "position": {
    "lat": 47.397742,
    "lon": 8.545594,
    "alt": 5.0
  },
  "timestamp": 1234567890123456789
}
```

**状态字段说明：**
- `uav_name`: 无人机名称（由`mqtt_mission`添加）
- `state`: 任务执行状态（`IDLE`, `ARMING`, `MISSION_ACTIVE`, `MISSION_COMPLETE`, `ERROR`）
- `mission_triggered`: 任务是否已触发
- `armed`: 是否已解锁
- `arming_state`: 解锁状态（0=初始化, 1=未解锁, 2=已解锁）
- `nav_state`: 导航状态（3=AUTO_MISSION, 4=AUTO_LOITER/HOLD等）
- `total_waypoints`: 总航点数（从`mavlink_mission`的`/mission/count`话题获取）
- `current_waypoint`: 当前航点索引（从0开始，从`mavlink_mission`的`/mission/current_waypoint`话题获取）
- `last_reached_waypoint`: 最后到达的航点索引（从0开始，如果未到达任何航点则此字段不存在）
- `mission_count_received`: 是否已收到航点总数（从`/mission/count`话题）
- `position`: GPS位置信息（如果GPS可用则包含此字段，包含`lat`, `lon`, `alt`）
- `timestamp`: 时间戳（纳秒）

**注意：**
- 任务进度通过`mavlink_mission`节点监听MAVLink的`MISSION_CURRENT`和`MISSION_ITEM_REACHED`消息获取，并发布到`/mission/count`和`/mission/current_waypoint`话题
- `px4_mission`节点订阅这些话题来跟踪任务进度，不再依赖`mission_result` uORB话题（该话题在某些PX4配置中可能不可用）

## 完整启动流程

在启动`mqtt_mission`之前，**必须先启动任务系统**：

```bash
# 终端1：启动任务系统（必需）
# 这会启动 mavlink_mission.py 和 px4_mission 节点
ros2 launch px4_control px4_mission.launch.py

# 终端2：启动mqtt_mission
ros2 launch px4_mqtt mqtt_mission.launch.py
```

**启动顺序说明：**
1. 首先启动`px4_mission.launch.py`，这会启动：
   - `mavlink_mission.py`: 负责通过MAVLink Mission协议上传航点到PX4的`dataman`存储，监听MAVLink消息并发布任务进度到`/mission/count`和`/mission/current_waypoint`话题
   - `px4_mission`: 负责执行任务，订阅`/mission/count`和`/mission/current_waypoint`跟踪进度，并发布状态到`/px4_mission/state`
2. 然后启动`mqtt_mission.launch.py`，这会启动：
   - `mqtt_mission`: 订阅MQTT任务并发布到ROS2话题（`/mission/waypoints`和`/px4_mission/trigger`），订阅`/px4_mission/state`并转发到MQTT

**注意：** 
- `mqtt_mission`节点与`px4_control`包中的`px4_mission`节点集成，架构与`mqtt_estimator`和`mqtt_control`一致
- 航点数据通过`/mission/waypoints`话题发送给`mavlink_mission`节点，通过MAVLink协议上传到PX4的`dataman`存储
- 任务执行通过`/px4_mission/trigger`话题触发`px4_mission`节点
- 任务状态从`/px4_mission/state`话题订阅并转发到MQTT
- 任务进度通过`mavlink_mission`节点监听MAVLink消息获取，并发布到`/mission/count`和`/mission/current_waypoint`话题供`px4_mission`使用

## 使用示例

### 通过MQTT发送任务

使用MQTT客户端（如`mosquitto_pub`）发送任务：

```bash
mosquitto_pub -h 118.195.156.74 -p 5704 -u uav1 -P Aa123456. \
  -t "uavmission/waypoints/uav1" \
  -m '{
    "waypoints": [
      [3, 16, 47.397742, 8.545594, 5.0, 0.0, 1.0, 0.0, 0.0],
      [3, 16, 47.397842, 8.545594, 5.0, 0.0, 1.0, 0.0, 0.0]
    ],
    "auto_trigger": true
  }'
```

### 订阅任务状态

```bash
mosquitto_sub -h 118.195.156.74 -p 5704 -u uav1 -P Aa123456. \
  -t "uavmission/status/uav1"
```

## ROS2话题

**发布的话题：**
- `/mission/waypoints` (std_msgs/Float32MultiArray): 航点数据，发送给`mavlink_mission`节点上传到PX4
  - QoS: `Reliable`（匹配`mavlink_mission`的订阅QoS）
- `/px4_mission/trigger` (std_msgs/Bool): 任务触发信号，发送给`px4_mission`节点执行任务
  - QoS: `BestEffort`（匹配`px4_mission`的订阅QoS）

**订阅的话题：**
- `/px4_mission/state` (std_msgs/String): 任务状态（JSON格式），从`px4_mission`节点订阅并转发到MQTT
  - QoS: `BestEffort`（匹配`px4_mission`的发布QoS）
  - 更新频率：2Hz（由`px4_mission`节点控制）

## 注意事项

1. **任务系统依赖**: `mqtt_mission`需要`px4_control`包中的`px4_mission`节点和`mavlink_mission`节点运行
2. **架构一致性**: `mqtt_mission`的架构与`mqtt_estimator`和`mqtt_control`一致，都通过中间节点与PX4通信
3. **航点格式**: 每个航点必须包含9个参数：`[frame, command, lat, lon, alt, param1, param2, param3, param4]`
4. **坐标系**: 推荐使用`frame=3`（全局相对高度，FRAME_GLOBAL_REL_ALT）
5. **MAVLink协议**: 航点通过MAVLink Mission协议上传到PX4的`dataman`存储，确保任务持久化
6. **自动触发**: 如果`auto_trigger=true`，任务会在上传后1秒自动触发执行
7. **状态更新**: 任务状态实时从`px4_mission`节点获取并转发到MQTT，更新频率为2Hz
8. **任务进度跟踪**: 任务进度通过`mavlink_mission`节点监听MAVLink的`MISSION_CURRENT`和`MISSION_ITEM_REACHED`消息获取，不依赖`mission_result` uORB话题
9. **QoS配置**: 
   - `/mission/waypoints`使用`Reliable` QoS确保航点数据可靠传输
   - `/px4_mission/trigger`使用`BestEffort` QoS匹配`px4_mission`的订阅
   - `/px4_mission/state`使用`BestEffort` QoS匹配`px4_mission`的发布
10. **GPS要求**: 发送任务前必须确保GPS可用且已设置home位置
11. **错误处理**: 如果命令格式错误或缺少必需参数，节点会记录警告日志但不会崩溃

## 文件结构

```
px4_mqtt/
├── config/
│   ├── mqtt_estimator.yaml          # Estimator配置文件
│   └── mqtt_control.yaml             # Control配置文件
├── include/px4_mqtt/
│   ├── mqtt_estimator.hpp           # Estimator头文件
│   ├── mqtt_control.hpp             # Control头文件
│   └── mqtt_mission.hpp             # Mission头文件
├── src/
│   ├── mqtt_estimator.cpp           # Estimator源文件
│   ├── mqtt_control.cpp             # Control源文件
│   └── mqtt_mission.cpp             # Mission源文件
├── launch/
│   ├── mqtt_estimator.launch.py     # Estimator Launch文件
│   ├── mqtt_control.launch.py       # Control Launch文件
│   └── mqtt_mission.launch.py       # Mission Launch文件
├── config/
│   ├── mqtt_estimator.yaml          # Estimator配置文件
│   ├── mqtt_control.yaml            # Control配置文件
│   └── mqtt_mission.yaml            # Mission配置文件
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 示例：完整启动流程

```bash
# 1. 编译
cd ~/px4_sitl/px4_ros2_ws
colcon build --packages-select px4_mqtt px4_control
source install/setup.bash

# 2. 编辑配置文件（可选）
# 编辑 config/mqtt_estimator.yaml

# 3. 启动px4_estimator
ros2 launch px4_control px4_estimator.launch.py

# 4. 启动mqtt_estimator（新终端）
ros2 launch px4_mqtt mqtt_estimator.launch.py

# 5. 验证数据流
# 终端3：检查ROS2话题
ros2 topic echo /px4_estimator/state

# 终端4：使用MQTT客户端订阅（需要安装mosquitto-clients）
mosquitto_sub -h 118.195.156.74 -p 5704 -u uav1 -P Aa123456. -t "uavcontrol/state/uav1"
```

## 许可证

BSD 3-Clause License
