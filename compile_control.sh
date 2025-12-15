#!/bin/bash

# 脚本名称: compile_control.sh
# 脚本描述: 编译PX4 ROS2控制模块

set -e  # 遇到错误立即退出

echo "=========================================="
echo "开始编译PX4 ROS2控制模块"
echo "=========================================="

# 进入工作空间目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo "警告: ROS2环境未设置，尝试source /opt/ros/*/setup.bash"
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    elif [ -f /opt/ros/foxy/setup.bash ]; then
        source /opt/ros/foxy/setup.bash
    else
        echo "错误: 未找到ROS2安装，请先安装ROS2或手动source环境"
        exit 1
    fi
fi

# 编译顺序：
# 1. microxrcedds_agent (基础依赖)
# 2. px4_msgs (消息定义)
# 3. px4_ros_com (ROS2通信库)
# 4. px4_control (控制模块)
# 5. px4_mqtt (MQTT桥接模块，可选)

echo ""
echo "[1/5] 编译 microxrcedds_agent..."
if ! colcon build --packages-select microxrcedds_agent --event-handlers console_direct+; then
    echo "错误: microxrcedds_agent 编译失败"
    exit 1
fi

echo ""
echo "[2/5] 编译 px4_msgs..."
if ! colcon build --packages-select px4_msgs --event-handlers console_direct+; then
    echo "错误: px4_msgs 编译失败"
    exit 1
fi

echo ""
echo "[3/5] 编译 px4_ros_com..."
if ! colcon build --packages-select px4_ros_com --event-handlers console_direct+; then
    echo "错误: px4_ros_com 编译失败"
    exit 1
fi

echo ""
echo "[4/5] 编译 px4_control..."
if ! colcon build --packages-select px4_control --event-handlers console_direct+; then
    echo "错误: px4_control 编译失败"
    exit 1
fi

echo ""
echo "[5/5] 编译 px4_mqtt..."
if ! colcon build --packages-select px4_mqtt --event-handlers console_direct+; then
    echo "警告: px4_mqtt 编译失败（可选包，继续）"
fi

echo ""
echo "=========================================="
echo "编译完成！"
echo "=========================================="

# 清理log文件（可选）
if [ -d "log" ]; then
    echo ""
    echo "清理log文件..."
    rm -rf log/*
    echo "log文件已清理"
fi

echo ""
echo "请运行以下命令设置环境："
echo "  source install/setup.bash"
echo ""

