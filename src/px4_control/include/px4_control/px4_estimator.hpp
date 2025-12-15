/****************************************************************************
 *
 * PX4 Estimator Node
 * Subscribes to multiple PX4 topics and displays status information in terminal
 *
 ****************************************************************************/

#ifndef PX4_CONTROL__PX4_ESTIMATOR_HPP_
#define PX4_CONTROL__PX4_ESTIMATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/estimator_status_flags.hpp>
#include <px4_msgs/msg/failsafe_flags.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/position_setpoint_triplet.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>

class PX4Estimator : public rclcpp::Node
{
public:
	explicit PX4Estimator();
	
	// Public getter methods for accessing state data
	const px4_msgs::msg::VehicleStatus& get_vehicle_status() const { return vehicle_status_; }
	const px4_msgs::msg::BatteryStatus& get_battery_status() const { return battery_status_; }
	const px4_msgs::msg::VehicleAttitude& get_vehicle_attitude() const { return vehicle_attitude_; }
	const px4_msgs::msg::VehicleLocalPosition& get_vehicle_local_position() const { return vehicle_local_position_; }
	const px4_msgs::msg::VehicleGlobalPosition& get_vehicle_global_position() const { return vehicle_global_position_; }
	const px4_msgs::msg::SensorGps& get_vehicle_gps_position() const { return vehicle_gps_position_; }
	const px4_msgs::msg::SensorCombined& get_sensor_combined() const { return sensor_combined_; }
	const px4_msgs::msg::VehicleControlMode& get_vehicle_control_mode() const { return vehicle_control_mode_; }
	const px4_msgs::msg::EstimatorStatusFlags& get_estimator_status_flags() const { return estimator_status_flags_; }
	const px4_msgs::msg::FailsafeFlags& get_failsafe_flags() const { return failsafe_flags_; }
	
	// Check if data is received
	bool is_status_received() const { return status_received_; }
	bool is_battery_received() const { return battery_received_; }
	bool is_attitude_received() const { return attitude_received_; }
	bool is_local_position_received() const { return local_position_received_; }
	bool is_global_position_received() const { return global_position_received_; }
	bool is_gps_received() const { return gps_received_; }

private:
	void setup_subscriptions(
		const rclcpp::QoS &sensor_qos,
		const rclcpp::QoS &status_qos,
		const rclcpp::QoS &default_qos
	);
	void print_status();
	void print_sensor_section();
	void print_state_section();
	void print_control_section();
	void print_health_section();
	void publish_state();  // Publish state data for MQTT bridge

	// Subscribers
	rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sensor_combined_sub_;
	rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr vehicle_gps_position_sub_;
	rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_status_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr vehicle_global_position_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr vehicle_control_mode_sub_;
	rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manual_control_setpoint_sub_;
	rclcpp::Subscription<px4_msgs::msg::PositionSetpointTriplet>::SharedPtr position_setpoint_triplet_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
	rclcpp::Subscription<px4_msgs::msg::EstimatorStatusFlags>::SharedPtr estimator_status_flags_sub_;
	rclcpp::Subscription<px4_msgs::msg::FailsafeFlags>::SharedPtr failsafe_flags_sub_;
	rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_status_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicle_command_ack_sub_;

	// Timer
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::TimerBase::SharedPtr state_timer_;  // Timer for publishing state data
	
	// Publisher for state data (for MQTT bridge or other nodes)
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

	// Message storage
	px4_msgs::msg::SensorCombined sensor_combined_;
	px4_msgs::msg::SensorGps vehicle_gps_position_;
	px4_msgs::msg::TimesyncStatus timesync_status_;
	px4_msgs::msg::VehicleAttitude vehicle_attitude_;
	px4_msgs::msg::VehicleLocalPosition vehicle_local_position_;
	px4_msgs::msg::VehicleGlobalPosition vehicle_global_position_;
	px4_msgs::msg::VehicleOdometry vehicle_odometry_;
	px4_msgs::msg::VehicleControlMode vehicle_control_mode_;
	px4_msgs::msg::ManualControlSetpoint manual_control_setpoint_;
	px4_msgs::msg::PositionSetpointTriplet position_setpoint_triplet_;
	px4_msgs::msg::VehicleStatus vehicle_status_;
	px4_msgs::msg::EstimatorStatusFlags estimator_status_flags_;
	px4_msgs::msg::FailsafeFlags failsafe_flags_;
	px4_msgs::msg::BatteryStatus battery_status_;
	px4_msgs::msg::VehicleCommandAck vehicle_command_ack_;

	// Receive flags
	bool sensor_combined_received_ = false;
	bool gps_received_ = false;
	bool timesync_received_ = false;
	bool attitude_received_ = false;
	bool local_position_received_ = false;
	bool global_position_received_ = false;
	bool odometry_received_ = false;
	bool control_mode_received_ = false;
	bool manual_control_received_ = false;
	bool setpoint_triplet_received_ = false;
	bool status_received_ = false;
	bool estimator_flags_received_ = false;
	bool failsafe_flags_received_ = false;
	bool battery_received_ = false;
	bool command_ack_received_ = false;
};

#endif  // PX4_CONTROL__PX4_ESTIMATOR_HPP_

