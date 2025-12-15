"mkdir -p ~/your_ws/px4_ros_uxrce_dds_ws/src
cd ~/your_ws/px4_ros_uxrce_dds_ws/src
git clone -b 2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
source /opt/ros/humble/setup.bash
colcon build
source /opt/ros/humble/setup.bash
source install/local_setup.bash
input termintor MicroXRCEAgent udp4 -p 8888(可修改)"
