#!/bin/bash

# Get the absolute path of the script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="$SCRIPT_DIR/../../../PX4-Autopilot"
ROS2_WS_DIR="$SCRIPT_DIR/../../.."

# gnome-terminal --window -e "bash -c \"cd $PX4_DIR && python3 ./simulation-gazebo; exec bash\"" \
# --tab -e "bash -c \"sleep 2; cd $PX4_DIR && PX4_GZ_STANDALONE=1 make px4_sitl gz_x500; exec bash\"" \
make px4_sitl gz_px4vision
# --tab -e "bash -c \"sleep 5; cd $ROS2_WS_DIR && MicroXRCEAgent udp4 -p 8888; exec bash\"" \
# --tab -e "bash -c \"sleep 6; cd $ROS2_WS_DIR && ros2 launch px4_control px4_estimator.launch.py; exec bash\"" \
# --tab -e "bash -c \"sleep 7; cd $ROS2_WS_DIR && ros2 launch px4_control px4_control.launch.py; exec bash\"" \
# --tab -e "bash -c \"sleep 8; cd $ROS2_WS_DIR && ros2 launch px4_control px4_joystick.launch.py; exec bash\""

