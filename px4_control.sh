#!/bin/bash

# 脚本名称: px4_control.sh
# 脚本描述: 启动PX4 SITL仿真和ROS2控制节点

# 启动PX4 SITL仿真器和Gazebo仿真器
cd ~/px4_sitl/PX4-Autopilot
make px4_sitl gz_px4vision
# 启动MicroXRCEAgent   MicroXRCEAgent udp4 -p 8888
cd ~/px4_sitl/px4_ros2_ws
source install/setup.bash

# 启动PX4 Estimator节点
cd ~/px4_sitl/px4_ros2_ws
source install/setup.bash
ros2 launch px4_control px4_estimator.launch.py

# 启动PX4 Control节点
cd ~/px4_sitl/px4_ros2_ws
source install/setup.bash
ros2 launch px4_control px4_control.launch.py
