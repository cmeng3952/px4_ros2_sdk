cmake_minimum_required(VERSION 3.5)
project(px4_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Find JSON library (nlohmann/json)
find_path(JSON_INCLUDE_DIR nlohmann/json.hpp
	PATHS
	/usr/include
	/usr/local/include
)
if(JSON_INCLUDE_DIR)
	message(STATUS "Found nlohmann/json at ${JSON_INCLUDE_DIR}")
	include_directories(${JSON_INCLUDE_DIR})
else()
	message(FATAL_ERROR "nlohmann/json not found. Please install: sudo apt-get install nlohmann-json3-dev")
endif()

#################
# Setup targets #
#################

include_directories(include)

# PX4 Estimator executable
add_executable(px4_estimator 
	src/px4_estimator.cpp
	src/px4_estimator_node.cpp
)
ament_target_dependencies(px4_estimator rclcpp px4_msgs std_msgs)
install(TARGETS px4_estimator DESTINATION lib/${PROJECT_NAME})

# PX4 Control Node executable
add_executable(px4_control 
	src/px4_control.cpp
	src/px4_control_node.cpp
)
ament_target_dependencies(px4_control rclcpp px4_msgs)
install(TARGETS px4_control DESTINATION lib/${PROJECT_NAME})

# Joystick Bridge executable
add_executable(joystick_bridge 
	utils/joystick_bridge.cpp
)
ament_target_dependencies(joystick_bridge rclcpp sensor_msgs)
install(TARGETS joystick_bridge DESTINATION lib/${PROJECT_NAME})

# PX4 Joystick converter executable
add_executable(px4_joystick 
	utils/px4_joystick.cpp
)
ament_target_dependencies(px4_joystick rclcpp px4_msgs sensor_msgs)
install(TARGETS px4_joystick DESTINATION lib/${PROJECT_NAME})

# PX4 Mission executable
add_executable(px4_mission 
	utils/px4_mission.cpp
	utils/px4_mission_node.cpp
)
ament_target_dependencies(px4_mission rclcpp px4_msgs std_msgs)
install(TARGETS px4_mission DESTINATION lib/${PROJECT_NAME})

############
# Install ##
############

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp px4_msgs sensor_msgs std_msgs)

# Install header files
install(DIRECTORY include/ DESTINATION include)

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Install scripts
install(PROGRAMS 
	scripts/px4_mission_cli.py
	scripts/mavlink_mission.py
	DESTINATION lib/${PROJECT_NAME})

ament_package()
