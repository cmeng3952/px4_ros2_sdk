/****************************************************************************
 *
 * PX4 Control Node
 * Provides control interface for PX4: arm/disarm, arm+takeoff, and offboard control
 *
 ****************************************************************************/

#ifndef PX4_CONTROL__PX4_CONTROL_HPP_
#define PX4_CONTROL__PX4_CONTROL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class PX4ControlNode : public rclcpp::Node
{
public:
	explicit PX4ControlNode();

	// Public control functions
	void arm();
	void disarm();
	void arm_and_takeoff();  // Arm then switch to AUTO.TAKEOFF
	
	// Mode switching functions
	void set_mode_offboard();
	void set_mode_land();     // Switch to AUTO.LAND mode
	void set_mode_hold();     // Switch to AUTO.LOITER (HOLD) mode
	
	// Offboard mode functions
	void publish_offboard_control_mode(bool position = true, bool velocity = false, 
	                                   bool acceleration = false, bool attitude = false, 
	                                   bool body_rate = false);
	void publish_trajectory_setpoint(float x = 0.0f, float y = 0.0f, float z = 0.0f, 
	                                 float vx = 0.0f, float vy = 0.0f, float vz = 0.0f,
	                                 float yaw = 0.0f, float yawspeed = 0.0f);
	void publish_position_setpoint(float x, float y, float z, float yaw = 0.0f);
	void publish_velocity_setpoint(float vx, float vy, float vz, float yaw = 0.0f);

private:
	// Publishers
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;

	// Subscribers
	rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicle_command_ack_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
	rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_command_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_sub_;

	// Timer for periodic offboard control mode publishing (must be at least 2Hz)
	rclcpp::TimerBase::SharedPtr offboard_timer_;

	// State variables
	px4_msgs::msg::VehicleLocalPosition vehicle_local_position_;
	px4_msgs::msg::VehicleStatus vehicle_status_;
	bool vehicle_local_position_received_ = false;
	bool vehicle_status_received_ = false;
	bool offboard_mode_active_ = false;
	bool offboard_mode_detected_ = false;  // Detected from vehicle_status
	uint64_t offboard_setpoint_counter_ = 0;
	
	// Current setpoint (used for maintaining offboard mode)
	float current_pos_setpoint_[3] = {0.0f, 0.0f, 0.0f};
	float current_vel_setpoint_[3] = {0.0f, 0.0f, 0.0f};
	float current_yaw_setpoint_ = 0.0f;
	bool use_position_control_ = true;  // true for position, false for velocity
	bool setpoint_initialized_ = false;

	// Callbacks
	void vehicle_command_ack_callback(const px4_msgs::msg::VehicleCommandAck::UniquePtr msg);
	void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg);
	void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::UniquePtr msg);
	void trajectory_setpoint_command_callback(const px4_msgs::msg::TrajectorySetpoint::UniquePtr msg);
	void vehicle_command_callback(const px4_msgs::msg::VehicleCommand::UniquePtr msg);
	void offboard_timer_callback();

	// Helper functions
	void publish_vehicle_command(uint32_t command, float param1 = 0.0f, float param2 = 0.0f,
	                             float param3 = 0.0f, float param4 = 0.0f, 
	                             double param5 = 0.0, double param6 = 0.0, float param7 = 0.0f);
	void set_mode_auto_takeoff();
	uint64_t get_timestamp();
};

#endif  // PX4_CONTROL__PX4_CONTROL_HPP_

