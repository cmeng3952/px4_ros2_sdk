/****************************************************************************
 *
 * PX4 Control Node Implementation
 * Provides arm/disarm, arm+takeoff, and offboard control (position/velocity)
 *
 ****************************************************************************/

#include "px4_control/px4_control.hpp"
#include <thread>
#include <chrono>
#include <cmath>

PX4ControlNode::PX4ControlNode() : Node("px4_control")
{
	// Configure QoS profiles according to PX4 DDS requirements
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
	
	offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
		"/fmu/in/offboard_control_mode", 10);
	
	trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
		"/fmu/in/trajectory_setpoint", 10);

	// Create subscribers
	vehicle_command_ack_sub_ = this->create_subscription<px4_msgs::msg::VehicleCommandAck>(
		"/fmu/out/vehicle_command_ack", status_qos,
		std::bind(&PX4ControlNode::vehicle_command_ack_callback, this, std::placeholders::_1));

	vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
		"/fmu/out/vehicle_local_position", default_qos,
		std::bind(&PX4ControlNode::vehicle_local_position_callback, this, std::placeholders::_1));

	vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
		"/fmu/out/vehicle_status", status_qos,
		std::bind(&PX4ControlNode::vehicle_status_callback, this, std::placeholders::_1));

	// Subscribe to trajectory setpoint commands (from CLI or other nodes)
	trajectory_setpoint_command_sub_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
		"/px4_control/trajectory_setpoint_command", default_qos,
		std::bind(&PX4ControlNode::trajectory_setpoint_command_callback, this, std::placeholders::_1));

	// Subscribe to vehicle command requests (from MQTT or other nodes)
	vehicle_command_sub_ = this->create_subscription<px4_msgs::msg::VehicleCommand>(
		"/px4_control/vehicle_command", default_qos,
		std::bind(&PX4ControlNode::vehicle_command_callback, this, std::placeholders::_1));

	// Create timer for offboard mode maintenance (must publish at least 2Hz)
	// Using 50ms = 20Hz to ensure we exceed the 2Hz requirement
	offboard_timer_ = this->create_wall_timer(50ms, std::bind(&PX4ControlNode::offboard_timer_callback, this));

	// Initialize setpoint variables
	current_pos_setpoint_[0] = 0.0f;
	current_pos_setpoint_[1] = 0.0f;
	current_pos_setpoint_[2] = 0.0f;
	current_vel_setpoint_[0] = 0.0f;
	current_vel_setpoint_[1] = 0.0f;
	current_vel_setpoint_[2] = 0.0f;
	current_yaw_setpoint_ = 0.0f;
	use_position_control_ = true;
	setpoint_initialized_ = false;

	RCLCPP_INFO(this->get_logger(), "PX4 Control Node started");
}

void PX4ControlNode::vehicle_command_ack_callback(const px4_msgs::msg::VehicleCommandAck::UniquePtr msg)
{
	// Silently handle command acknowledgments
	// Only log errors
	if (msg->command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM) {
		if (msg->result != px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
			RCLCPP_WARN(this->get_logger(), "Arm/Disarm command failed with result: %u", msg->result);
		}
	} else if (msg->command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE) {
		if (msg->result != px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
			RCLCPP_WARN(this->get_logger(), "Mode change command failed with result: %u", msg->result);
		}
	} else if (msg->command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND) {
		if (msg->result != px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
			RCLCPP_WARN(this->get_logger(), "Land command failed with result: %u", msg->result);
		}
	}
}

void PX4ControlNode::vehicle_command_callback(const px4_msgs::msg::VehicleCommand::UniquePtr msg)
{
	// This callback receives vehicle commands from MQTT or other nodes
	// Handle special commands that require additional processing
	
	// Check if this is a set_mode command for offboard mode
	// VEHICLE_CMD_DO_SET_MODE with param1=1.0, param2=6.0 (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED)
	if (msg->command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE &&
	    msg->param1 == 1.0f && msg->param2 == 6.0f && msg->param3 == 0.0f) {
		// This is a request to switch to OFFBOARD mode
		// Use the proper set_mode_offboard() function which handles initialization
		RCLCPP_INFO(this->get_logger(), "Received OFFBOARD mode command from external node, initializing...");
		set_mode_offboard();
		return;
	}
	
	// For all other commands, forward directly to PX4
	px4_msgs::msg::VehicleCommand forward_msg = *msg;
	forward_msg.timestamp = get_timestamp();
	
	RCLCPP_DEBUG(this->get_logger(), "Forwarding vehicle command %u to PX4", msg->command);
	vehicle_command_pub_->publish(forward_msg);
}

uint64_t PX4ControlNode::get_timestamp()
{
	return this->get_clock()->now().nanoseconds() / 1000;
}

void PX4ControlNode::publish_vehicle_command(uint32_t command, float param1, float param2,
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

void PX4ControlNode::arm()
{
	publish_vehicle_command(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
		1.0f
	);
}

void PX4ControlNode::disarm()
{
	publish_vehicle_command(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
		0.0f
	);
}

void PX4ControlNode::set_mode_auto_takeoff()
{
	publish_vehicle_command(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
		1.0f, 4.0f, 2.0f
	);
}

void PX4ControlNode::set_mode_land()
{
	publish_vehicle_command(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
		1.0f, 4.0f, 6.0f
	);
}

void PX4ControlNode::set_mode_hold()
{
	publish_vehicle_command(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
		1.0f, 4.0f, 3.0f
	);
}

void PX4ControlNode::arm_and_takeoff()
{
	RCLCPP_INFO(this->get_logger(), "Arming and switching to AUTO TAKEOFF mode...");
	// Step 1: Arm the vehicle
	arm();
	// Wait for arm to complete (PX4 needs time to process arm command)
	// Note: In practice, you should wait for vehicle_command_ack to confirm arm success
	// For now, we wait a short time
	std::this_thread::sleep_for(std::chrono::milliseconds(3000));
	// Step 2: Switch to AUTO TAKEOFF mode
	set_mode_auto_takeoff();
}

void PX4ControlNode::vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg)
{
	vehicle_local_position_ = *msg;
	vehicle_local_position_received_ = true;
	
	// If we're in offboard mode but setpoint not initialized, use current position
	if (offboard_mode_active_ && !setpoint_initialized_) {
		current_pos_setpoint_[0] = vehicle_local_position_.x;
		current_pos_setpoint_[1] = vehicle_local_position_.y;
		current_pos_setpoint_[2] = vehicle_local_position_.z;
		current_yaw_setpoint_ = vehicle_local_position_.heading;
		setpoint_initialized_ = true;
		use_position_control_ = true;
		RCLCPP_INFO(this->get_logger(), "Initialized offboard setpoint with current position: [%.2f, %.2f, %.2f]",
		            current_pos_setpoint_[0], current_pos_setpoint_[1], current_pos_setpoint_[2]);
	}
}

void PX4ControlNode::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::UniquePtr msg)
{
	vehicle_status_ = *msg;
	vehicle_status_received_ = true;
	
	// Check if vehicle is in OFFBOARD mode
	// PX4 vehicle_status.nav_state values:
	// 14 = NAVIGATION_STATE_OFFBOARD
	bool is_offboard = (vehicle_status_.nav_state == 14);
	
	if (is_offboard && !offboard_mode_detected_) {
		// Just entered offboard mode
		offboard_mode_detected_ = true;
		offboard_mode_active_ = true;  // Enable publishing
		
		// Initialize setpoint with current position if available and not already initialized
		if (vehicle_local_position_received_) {
			// Always update setpoint when entering offboard mode to maintain current position
			if (!setpoint_initialized_ || !offboard_mode_active_) {
				current_pos_setpoint_[0] = vehicle_local_position_.x;
				current_pos_setpoint_[1] = vehicle_local_position_.y;
				current_pos_setpoint_[2] = vehicle_local_position_.z;
				current_yaw_setpoint_ = vehicle_local_position_.heading;
				setpoint_initialized_ = true;
				use_position_control_ = true;
			}
		}
		
		if (!offboard_mode_active_) {
			offboard_mode_active_ = true;
		}
	} else if (!is_offboard && offboard_mode_detected_) {
		offboard_mode_detected_ = false;
		offboard_mode_active_ = false;
	}
}

void PX4ControlNode::trajectory_setpoint_command_callback(const px4_msgs::msg::TrajectorySetpoint::UniquePtr msg)
{
	// This callback receives setpoint commands from CLI or other nodes
	// Update internal setpoint state so timer can continuously publish it
	
	// Determine control type based on message content
	bool has_position = false;
	bool has_velocity = false;
	
	// Check if position is valid (not NaN) - all 3 components must be valid
	bool pos_valid[3] = {false, false, false};
	for (int i = 0; i < 3; ++i) {
		if (!std::isnan(msg->position[i])) {
			pos_valid[i] = true;
			current_pos_setpoint_[i] = msg->position[i];
		}
	}
	has_position = pos_valid[0] && pos_valid[1] && pos_valid[2];
	
	// Check if velocity is valid (not NaN) - at least one component must be valid
	for (int i = 0; i < 3; ++i) {
		if (!std::isnan(msg->velocity[i])) {
			has_velocity = true;
			current_vel_setpoint_[i] = msg->velocity[i];
		} else {
			current_vel_setpoint_[i] = 0.0f;
		}
	}
	
	// Update yaw
	if (!std::isnan(msg->yaw)) {
		current_yaw_setpoint_ = msg->yaw;
	}
	
	// Determine control mode
	if (has_position && !has_velocity) {
		// Position control only
		use_position_control_ = true;
		current_vel_setpoint_[0] = 0.0f;
		current_vel_setpoint_[1] = 0.0f;
		current_vel_setpoint_[2] = 0.0f;
	} else if (has_velocity) {
		use_position_control_ = false;
		if (vehicle_local_position_received_ && !has_position) {
			current_pos_setpoint_[0] = vehicle_local_position_.x;
			current_pos_setpoint_[1] = vehicle_local_position_.y;
			current_pos_setpoint_[2] = vehicle_local_position_.z;
		}
	} else {
		return;
	}
	
	if (!offboard_mode_active_) {
		offboard_mode_active_ = true;
	}
	
	setpoint_initialized_ = true;
	
	// Immediately publish once (timer will continue publishing at 20Hz)
	if (use_position_control_) {
		publish_offboard_control_mode(true, false, false, false, false);
		publish_trajectory_setpoint(
			current_pos_setpoint_[0],
			current_pos_setpoint_[1],
			current_pos_setpoint_[2],
			0.0f, 0.0f, 0.0f,
			current_yaw_setpoint_,
			0.0f
		);
	} else {
		float z = vehicle_local_position_received_ ? vehicle_local_position_.z : current_pos_setpoint_[2];
		publish_offboard_control_mode(false, true, false, false, false);
		publish_trajectory_setpoint(
			std::nanf(""), std::nanf(""), z,
			current_vel_setpoint_[0],
			current_vel_setpoint_[1],
			current_vel_setpoint_[2],
			current_yaw_setpoint_,
			0.0f
		);
	}
}

void PX4ControlNode::offboard_timer_callback()
{
	// If offboard mode is active (either manually activated or detected from vehicle_status),
	// maintain it by publishing control mode and setpoint
	// PX4 requires offboard_control_mode AND trajectory_setpoint to be published at least at 2Hz
	// We publish at 20Hz (50ms timer) to ensure we exceed the requirement
	if (offboard_mode_active_ || offboard_mode_detected_) {
		// Ensure offboard_mode_active_ is true if detected
		if (offboard_mode_detected_ && !offboard_mode_active_) {
			offboard_mode_active_ = true;
		}
		// Publish offboard control mode
		if (use_position_control_) {
			publish_offboard_control_mode(true, false, false, false, false);
		} else {
			publish_offboard_control_mode(false, true, false, false, false);
		}
		
		// Publish trajectory setpoint to maintain offboard mode
		// If setpoint is initialized, use it; otherwise use current position
		if (setpoint_initialized_) {
			if (use_position_control_) {
				publish_trajectory_setpoint(
					current_pos_setpoint_[0], 
					current_pos_setpoint_[1], 
					current_pos_setpoint_[2],
					0.0f, 0.0f, 0.0f,  // velocity = 0 for position control
					current_yaw_setpoint_, 
					0.0f
				);
			} else {
				// For velocity control, maintain current altitude
				float z = vehicle_local_position_received_ ? vehicle_local_position_.z : current_pos_setpoint_[2];
				publish_trajectory_setpoint(
					std::nanf(""), std::nanf(""), z,  // position = NaN for velocity control
					current_vel_setpoint_[0],
					current_vel_setpoint_[1],
					current_vel_setpoint_[2],
					current_yaw_setpoint_,
					0.0f
				);
			}
		} else if (vehicle_local_position_received_) {
			// Use current position as setpoint to maintain position
			// Update setpoint with current position
			current_pos_setpoint_[0] = vehicle_local_position_.x;
			current_pos_setpoint_[1] = vehicle_local_position_.y;
			current_pos_setpoint_[2] = vehicle_local_position_.z;
			current_yaw_setpoint_ = vehicle_local_position_.heading;
			setpoint_initialized_ = true;
			use_position_control_ = true;
			
			publish_offboard_control_mode(true, false, false, false, false);
			publish_trajectory_setpoint(
				vehicle_local_position_.x,
				vehicle_local_position_.y,
				vehicle_local_position_.z,
				0.0f, 0.0f, 0.0f,
				vehicle_local_position_.heading,
				0.0f
			);
		} else {
			// No position received yet, publish zero setpoint
			publish_offboard_control_mode(true, false, false, false, false);
			publish_trajectory_setpoint(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		}
		
		offboard_setpoint_counter_++;
	}
}

void PX4ControlNode::set_mode_offboard()
{
	RCLCPP_INFO(this->get_logger(), "Switching to OFFBOARD mode...");
	
	// Initialize setpoint with current position if available
	if (vehicle_local_position_received_) {
		current_pos_setpoint_[0] = vehicle_local_position_.x;
		current_pos_setpoint_[1] = vehicle_local_position_.y;
		current_pos_setpoint_[2] = vehicle_local_position_.z;
		current_yaw_setpoint_ = vehicle_local_position_.heading;
		setpoint_initialized_ = true;
		use_position_control_ = true;
	} else {
		current_pos_setpoint_[0] = 0.0f;
		current_pos_setpoint_[1] = 0.0f;
		current_pos_setpoint_[2] = 0.0f;
		current_yaw_setpoint_ = 0.0f;
		setpoint_initialized_ = false;
		use_position_control_ = true;
	}
	
	// Send initial offboard control mode messages (required by PX4)
	offboard_mode_active_ = true;
	offboard_setpoint_counter_ = 0;
	
	for (int i = 0; i < 20; ++i) {
		publish_offboard_control_mode(true, false, false, false, false);
		if (setpoint_initialized_) {
			publish_trajectory_setpoint(
				current_pos_setpoint_[0],
				current_pos_setpoint_[1],
				current_pos_setpoint_[2],
				0.0f, 0.0f, 0.0f,
				current_yaw_setpoint_,
				0.0f
			);
		} else {
			publish_trajectory_setpoint(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	
	publish_vehicle_command(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
		1.0f, 6.0f, 0.0f
	);
	
	offboard_mode_active_ = true;
	offboard_mode_detected_ = true;
}

void PX4ControlNode::publish_offboard_control_mode(bool position, bool velocity,
                                                    bool acceleration, bool attitude,
                                                    bool body_rate)
{
	px4_msgs::msg::OffboardControlMode msg{};
	msg.timestamp = get_timestamp();
	msg.position = position;
	msg.velocity = velocity;
	msg.acceleration = acceleration;
	msg.attitude = attitude;
	msg.body_rate = body_rate;
	msg.thrust_and_torque = false;
	msg.direct_actuator = false;

	offboard_control_mode_pub_->publish(msg);
}

void PX4ControlNode::publish_trajectory_setpoint(float x, float y, float z,
                                                 float vx, float vy, float vz,
                                                 float yaw, float yawspeed)
{
	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.timestamp = get_timestamp();
	
	// Position setpoint (NED frame: z is negative for altitude)
	msg.position[0] = x;
	msg.position[1] = y;
	msg.position[2] = z;
	
	// Velocity setpoint (NED frame)
	msg.velocity[0] = vx;
	msg.velocity[1] = vy;
	msg.velocity[2] = vz;
	
	// Acceleration setpoint (set to NaN to not control)
	msg.acceleration[0] = std::nanf("");
	msg.acceleration[1] = std::nanf("");
	msg.acceleration[2] = std::nanf("");
	
	// Yaw and yaw rate
	msg.yaw = yaw;
	msg.yawspeed = yawspeed;

	trajectory_setpoint_pub_->publish(msg);
}

void PX4ControlNode::publish_position_setpoint(float x, float y, float z, float yaw)
{
	if (!offboard_mode_active_) {
		offboard_mode_active_ = true;
	}
	
	// Update current setpoint
	current_pos_setpoint_[0] = x;
	current_pos_setpoint_[1] = y;
	current_pos_setpoint_[2] = z;
	current_yaw_setpoint_ = yaw;
	current_vel_setpoint_[0] = 0.0f;
	current_vel_setpoint_[1] = 0.0f;
	current_vel_setpoint_[2] = 0.0f;
	use_position_control_ = true;
	setpoint_initialized_ = true;
	
	// Publish offboard control mode with position enabled
	publish_offboard_control_mode(true, false, false, false, false);
	
	// Publish trajectory setpoint with position
	publish_trajectory_setpoint(x, y, z, 0.0f, 0.0f, 0.0f, yaw, 0.0f);
}

void PX4ControlNode::publish_velocity_setpoint(float vx, float vy, float vz, float yaw)
{
	if (!offboard_mode_active_) {
		offboard_mode_active_ = true;
	}
	
	// Update current setpoint
	current_vel_setpoint_[0] = vx;
	current_vel_setpoint_[1] = vy;
	current_vel_setpoint_[2] = vz;
	current_yaw_setpoint_ = yaw;
	use_position_control_ = false;
	setpoint_initialized_ = true;
	
	// Update position setpoint for maintaining altitude
	if (vehicle_local_position_received_) {
		current_pos_setpoint_[0] = vehicle_local_position_.x;
		current_pos_setpoint_[1] = vehicle_local_position_.y;
		current_pos_setpoint_[2] = vehicle_local_position_.z;
	}
	
	// Publish offboard control mode with velocity enabled
	publish_offboard_control_mode(false, true, false, false, false);
	
	// Get current position to maintain it (or use NaN to not control position)
	float x = std::nanf("");
	float y = std::nanf("");
	float z = vehicle_local_position_received_ ? vehicle_local_position_.z : current_pos_setpoint_[2];
	
	// Publish trajectory setpoint with velocity
	publish_trajectory_setpoint(x, y, z, vx, vy, vz, yaw, 0.0f);
}
