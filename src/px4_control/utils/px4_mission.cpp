/****************************************************************************
 *
 * PX4 Mission Implementation
 * 接收触发信号，通过ROS2执行航线任务，并监测任务状态
 *
 ****************************************************************************/

#include "px4_mission/px4_mission.hpp"
#include <sstream>
#include <iomanip>
#include <iostream>
#include <std_msgs/msg/u_int16.hpp>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

PX4Mission::PX4Mission() : Node("px4_mission"), last_nav_state_(255)
{
	// Configure QoS profiles
	rclcpp::QoS status_qos(10);
	status_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
	status_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
	status_qos.history(rclcpp::HistoryPolicy::KeepLast);

	rclcpp::QoS default_qos(10);
	default_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
	default_qos.history(rclcpp::HistoryPolicy::KeepLast);

	// Create publishers
	vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
		"/fmu/in/vehicle_command", 10);
	
	// Publisher for mission state (JSON format for MQTT bridge)
	mission_state_pub_ = this->create_publisher<std_msgs::msg::String>(
		"/px4_mission/state", 10);

	// Create subscribers
	// Subscribe to both topics for backward compatibility
	mission_trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
		"/px4_mission/trigger", default_qos,
		std::bind(&PX4Mission::mission_trigger_callback, this, std::placeholders::_1));
	
	// Also subscribe to legacy topic for backward compatibility
	legacy_trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
		"/mission/trigger", default_qos,
		std::bind(&PX4Mission::mission_trigger_callback, this, std::placeholders::_1));

	vehicle_command_ack_sub_ = this->create_subscription<px4_msgs::msg::VehicleCommandAck>(
		"/fmu/out/vehicle_command_ack", status_qos,
		std::bind(&PX4Mission::vehicle_command_ack_callback, this, std::placeholders::_1));

	vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
		"/fmu/out/vehicle_status", status_qos,
		std::bind(&PX4Mission::vehicle_status_callback, this, std::placeholders::_1));

	// Subscribe to mission count and current waypoint from mavlink_mission
	mission_count_sub_ = this->create_subscription<std_msgs::msg::UInt16>(
		"/mission/count", default_qos,
		std::bind(&PX4Mission::mission_count_callback, this, std::placeholders::_1));
	
	current_waypoint_sub_ = this->create_subscription<std_msgs::msg::UInt16>(
		"/mission/current_waypoint", default_qos,
		std::bind(&PX4Mission::current_waypoint_callback, this, std::placeholders::_1));

	vehicle_global_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
		"/fmu/out/vehicle_global_position", default_qos,
		std::bind(&PX4Mission::vehicle_global_position_callback, this, std::placeholders::_1));

	// Create control timer (10Hz)
	control_timer_ = this->create_wall_timer(100ms,
		std::bind(&PX4Mission::control_loop_callback, this));

	// Create status monitor timer (2Hz for smoother updates)
	status_timer_ = this->create_wall_timer(500ms,
		std::bind(&PX4Mission::status_monitor_callback, this));

	// Create timer to publish state data (for MQTT bridge, 2Hz)
	state_publish_timer_ = this->create_wall_timer(500ms,
		std::bind(&PX4Mission::publish_mission_state, this));

	RCLCPP_INFO(this->get_logger(), "PX4 Mission node started");
}

void PX4Mission::mission_trigger_callback(const std_msgs::msg::Bool::UniquePtr msg)
{
	mission_triggered_ = msg->data;
	if (mission_triggered_ && current_state_ == MissionExecState::IDLE) {
		mission_trigger_time_ = this->now();
		mission_trigger_time_set_ = true;
		last_nav_state_ = 255;
		current_state_ = MissionExecState::ARMING;
	} else if (!mission_triggered_) {
		current_state_ = MissionExecState::IDLE;
		mission_trigger_time_set_ = false;
		last_nav_state_ = 255;
	}
}

void PX4Mission::vehicle_command_ack_callback(const px4_msgs::msg::VehicleCommandAck::UniquePtr msg)
{
	if (msg->result != px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
		RCLCPP_WARN(this->get_logger(), "Command %u failed: %u", msg->command, msg->result);
	}
}

void PX4Mission::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::UniquePtr msg)
{
	if (last_nav_state_ == 255 || last_nav_state_ != msg->nav_state) {
		last_nav_state_ = msg->nav_state;
	}
	vehicle_status_ = *msg;
	vehicle_status_received_ = true;
	
	// Update current waypoint from vehicle_status if available
	// Note: PX4 may provide current mission item index in vehicle_status
	// This is a placeholder - actual implementation depends on PX4 message structure
}

void PX4Mission::mission_count_callback(const std_msgs::msg::UInt16::UniquePtr msg)
{
	total_waypoints_ = msg->data;
	mission_count_received_ = true;
	// Mission count received, no need to log
}

void PX4Mission::current_waypoint_callback(const std_msgs::msg::UInt16::UniquePtr msg)
{
	uint16_t new_waypoint = msg->data;
	
	// If current waypoint increased, update last_reached_waypoint
	if (new_waypoint > current_waypoint_) {
		if (current_waypoint_ < 65535) {
			last_reached_waypoint_ = current_waypoint_;  // Previous waypoint was reached
		}
	}
	
	current_waypoint_ = new_waypoint;
	
	// If current waypoint >= total waypoints, mission is complete
		if (mission_count_received_ && total_waypoints_ > 0 && current_waypoint_ >= total_waypoints_) {
			if (current_state_ == MissionExecState::MISSION_ACTIVE) {
				current_state_ = MissionExecState::MISSION_COMPLETE;
			}
		}
}

void PX4Mission::vehicle_global_position_callback(const px4_msgs::msg::VehicleGlobalPosition::UniquePtr msg)
{
	vehicle_global_position_ = *msg;
	vehicle_global_position_received_ = true;

	// Record home position on first GPS fix
	if (!home_received_ && msg->lat != 0.0 && msg->lon != 0.0) {
		home_lat_ = msg->lat;
		home_lon_ = msg->lon;
		home_alt_ = msg->alt;
		home_received_ = true;
	}
}

void PX4Mission::control_loop_callback()
{
	if (!mission_triggered_) {
		return;
	}

	switch (current_state_) {
		case MissionExecState::ARMING:
			handle_arming_state();
			break;
		case MissionExecState::MISSION_ACTIVE:
			handle_mission_active_state();
			break;
		case MissionExecState::MISSION_COMPLETE:
			handle_mission_complete_state();
			break;
		case MissionExecState::ERROR:
			handle_error_state();
			break;
		case MissionExecState::IDLE:
		default:
			break;
	}
}

void PX4Mission::status_monitor_callback()
{
	if (!vehicle_status_received_) {
		return;
	}

	// Display status for all states (including IDLE when mission is triggered or waiting)

	// Print header (similar to px4_estimator format)
	std::cout << "================================================================================\n";
	std::cout << "                            PX4 MISSION STATUS                                 \n";
	std::cout << "================================================================================\n\n";
	
	// Mission State Section
	std::cout << "[MISSION STATE]\n";
	std::cout << "--------------------------------------------------------------------------------\n";
	
	const char* state_names[] = {
		"IDLE", "ARMING", "MISSION_ACTIVE", "MISSION_COMPLETE", "ERROR"
	};
	
	std::cout << "State: " << state_names[static_cast<int>(current_state_)] << "\n";
	
	// Vehicle Status Section
	std::cout << "\n[VEHICLE STATUS]\n";
	std::cout << "--------------------------------------------------------------------------------\n";
	
	std::string arm_status;
	if (vehicle_status_.arming_state == 2) {
		arm_status = "ARMED";
	} else if (vehicle_status_.arming_state == 1) {
		arm_status = "DISARMED";
	} else {
		arm_status = "UNKNOWN";
	}
	
	const char* nav_state_names[] = {
		"MANUAL", "ALTCTL", "POSCTL", "AUTO_MISSION", "AUTO_LOITER", "AUTO_RTL", "AUTO_LAND", "AUTO_TAKEOFF"
	};
	const char* nav_state_name = "UNKNOWN";
	if (vehicle_status_.nav_state < 8) {
		nav_state_name = nav_state_names[vehicle_status_.nav_state];
	}
	
	std::cout << "Armed: " << arm_status << "\n";
	std::cout << "NavState: " << static_cast<int>(vehicle_status_.nav_state) << " (" << nav_state_name << ")\n";
	
	// Mission Progress Section
	std::cout << "\n[MISSION PROGRESS]\n";
	std::cout << "--------------------------------------------------------------------------------\n";
	if (mission_count_received_) {
		std::cout << "Total Waypoints: " << total_waypoints_ << "\n";
		
		// Display current waypoint (1-indexed for user display)
		if (current_waypoint_ < total_waypoints_) {
			std::cout << "Current Waypoint: " << (current_waypoint_ + 1) << " (index " << current_waypoint_ << ")\n";
		} else if (current_waypoint_ >= total_waypoints_) {
			std::cout << "Current Waypoint: Completed (all " << total_waypoints_ << " waypoints reached)\n";
		} else {
			std::cout << "Current Waypoint: " << (current_waypoint_ + 1) << "\n";
		}
		
		if (last_reached_waypoint_ != 65535) {
			std::cout << "Last Reached: " << (last_reached_waypoint_ + 1) << " (index " << last_reached_waypoint_ << ")\n";
		}
		
		if (vehicle_status_.nav_state == 3) {
			std::cout << "Status: Executing mission (AUTO_MISSION mode)\n";
		} else if (vehicle_status_.nav_state == 4) {
			std::cout << "Status: Mission completed (AUTO_LOITER/HOLD mode)\n";
		}
	} else {
		std::cout << "Total Waypoints: Waiting for mission count...\n";
		if (vehicle_status_.nav_state == 3) {
			std::cout << "Status: Executing mission (AUTO_MISSION mode)\n";
		} else if (vehicle_status_.nav_state == 4) {
			std::cout << "Status: Mission completed (AUTO_LOITER/HOLD mode)\n";
		}
	}
	
	// Position Section
	if (vehicle_global_position_received_) {
		std::cout << "\n[POSITION]\n";
		std::cout << "--------------------------------------------------------------------------------\n";
		std::cout << std::fixed << std::setprecision(7);
		std::cout << "Lat: " << vehicle_global_position_.lat 
		          << "  Lon: " << vehicle_global_position_.lon 
		          << "  Alt: " << std::setprecision(2) << vehicle_global_position_.alt << "m\n";
	}
	
	std::cout << "\n";
	std::cout.flush();
}


void PX4Mission::handle_arming_state()
{
	static rclcpp::Time last_arm_attempt = this->now();
	static rclcpp::Time last_mode_attempt = this->now();
	const double attempt_interval = 1.0;

	if (!vehicle_status_received_ || !home_received_) {
		return;
	}

	// Set mission trigger time if not set
	if (!mission_trigger_time_set_ && mission_triggered_) {
		mission_trigger_time_ = this->now();
		mission_trigger_time_set_ = true;
	}

	// Check mission availability: wait 3s for MAVLink upload
	bool mission_available = false;
	if (mission_trigger_time_set_ && (this->now() - mission_trigger_time_).seconds() > 3.0) {
		mission_available = true;
	}
	
	if (!mission_available) {
		return;
	}

	// Set mode to AUTO_MISSION first
	if (vehicle_status_.nav_state != 3) {
		if ((this->now() - last_mode_attempt).seconds() > attempt_interval) {
			set_mode_auto_mission();
			last_mode_attempt = this->now();
		}
		return;
	}

	// Arm vehicle if not armed
	if (vehicle_status_.arming_state != 2) {
		if ((this->now() - last_arm_attempt).seconds() > attempt_interval) {
			arm_vehicle();
			last_arm_attempt = this->now();
		}
		return;
	}

	// Start mission
		start_mission();
		current_state_ = MissionExecState::MISSION_ACTIVE;
}

void PX4Mission::handle_mission_active_state()
{
	if (!vehicle_status_received_) {
		return;
	}

	// Check mode change: nav_state 3=AUTO_MISSION, 4=AUTO_LOITER(HOLD), 5=AUTO_RTL, 6=AUTO_LAND
	if (vehicle_status_.nav_state != 3) {
		// nav_state 4 (HOLD) is normal completion state
		if (vehicle_status_.nav_state == 4 || vehicle_status_.nav_state == 5 || 
		    vehicle_status_.nav_state == 6 || vehicle_status_.arming_state != 2) {
			current_state_ = MissionExecState::MISSION_COMPLETE;
		}
	}
}

void PX4Mission::handle_mission_complete_state()
{
	// Mission completed, return to IDLE state (no automatic RTL)
	// User can set RTL waypoint as the last waypoint in the mission
	current_state_ = MissionExecState::IDLE;
	mission_triggered_ = false;
	last_reached_waypoint_ = 65535;
}


void PX4Mission::handle_error_state()
{
	current_state_ = MissionExecState::IDLE;
	mission_triggered_ = false;
}

uint64_t PX4Mission::get_timestamp()
{
	return this->get_clock()->now().nanoseconds() / 1000;
}

void PX4Mission::publish_vehicle_command(uint32_t command, float param1, float param2,
                                              float param3, float param4,
                                              double param5, double param6, float param7)
{
	px4_msgs::msg::VehicleCommand msg{};
	msg.timestamp = get_timestamp();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.param3 = param3;
	msg.param4 = param4;
	msg.param5 = param5;
	msg.param6 = param6;
	msg.param7 = param7;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.confirmation = 0;

	vehicle_command_pub_->publish(msg);
}

void PX4Mission::set_mode_auto_mission()
{
	// VEHICLE_CMD_DO_SET_MODE: param1=1, param2=4 (AUTO), param3=4 (AUTO_MISSION)
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
		1.0f, 4.0f, 4.0f);
}

void PX4Mission::arm_vehicle()
{
	if (vehicle_status_.arming_state != 2) {
		publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
	}
}

void PX4Mission::start_mission()
{
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_MISSION_START, 0.0f, 0.0f);
}

void PX4Mission::publish_mission_state()
{
	json j;
	
	// Mission state
	const char* state_names[] = {
		"IDLE", "ARMING", "MISSION_ACTIVE", "MISSION_COMPLETE", "ERROR"
	};
	j["state"] = state_names[static_cast<int>(current_state_)];
	j["mission_triggered"] = mission_triggered_;
	
	// Vehicle status
	if (vehicle_status_received_) {
		j["armed"] = (vehicle_status_.arming_state == 2);
		j["arming_state"] = static_cast<int>(vehicle_status_.arming_state);
		j["nav_state"] = static_cast<int>(vehicle_status_.nav_state);
	}
	
	// Mission progress
	j["total_waypoints"] = total_waypoints_;
	j["current_waypoint"] = current_waypoint_;
	if (last_reached_waypoint_ != 65535) {
		j["last_reached_waypoint"] = last_reached_waypoint_;
	}
	j["mission_count_received"] = mission_count_received_;
	
	// Position
	if (vehicle_global_position_received_) {
		j["position"] = {
			{"lat", vehicle_global_position_.lat},
			{"lon", vehicle_global_position_.lon},
			{"alt", vehicle_global_position_.alt}
		};
	}
	
	// Timestamp
	j["timestamp"] = this->now().nanoseconds();
	
	// Publish as string message
	auto msg = std_msgs::msg::String();
	msg.data = j.dump();
	mission_state_pub_->publish(msg);
}

