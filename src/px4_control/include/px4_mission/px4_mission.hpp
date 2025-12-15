/****************************************************************************
 *
 * PX4 Mission
 * 接收触发信号，通过ROS2执行航线任务，并监测任务状态
 *
 ****************************************************************************/

#ifndef PX4_MISSION__PX4_MISSION_HPP_
#define PX4_MISSION__PX4_MISSION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>

using namespace std::chrono_literals;

// Mission execution state enumeration
enum class MissionExecState {
	IDLE = 0,              // 空闲，等待触发
	ARMING = 1,            // 解锁中
	MISSION_ACTIVE = 2,    // 任务执行中
	MISSION_COMPLETE = 3,  // 任务完成
	ERROR = 4              // 错误状态
};

class PX4Mission : public rclcpp::Node
{
public:
	explicit PX4Mission();

private:
	// Publishers
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_state_pub_;

	// Subscribers
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_trigger_sub_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr legacy_trigger_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr vehicle_command_ack_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
	rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr mission_count_sub_;
	rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr current_waypoint_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr vehicle_global_position_sub_;

	// Timer for control loop and status monitoring
	rclcpp::TimerBase::SharedPtr control_timer_;
	rclcpp::TimerBase::SharedPtr status_timer_;
	rclcpp::TimerBase::SharedPtr state_publish_timer_;

	// State variables
	MissionExecState current_state_ = MissionExecState::IDLE;
	bool mission_triggered_ = false;
	
	px4_msgs::msg::VehicleStatus vehicle_status_;
	px4_msgs::msg::VehicleGlobalPosition vehicle_global_position_;
	
	bool vehicle_status_received_ = false;
	bool vehicle_global_position_received_ = false;
	
	uint16_t total_waypoints_ = 0;
	bool mission_count_received_ = false;
	uint16_t current_waypoint_ = 0;
	
	double home_lat_ = 0.0;
	double home_lon_ = 0.0;
	double home_alt_ = 0.0;
	bool home_received_ = false;
	
	uint16_t last_reached_waypoint_ = 65535;
	
	rclcpp::Time mission_trigger_time_;
	bool mission_trigger_time_set_ = false;
	
	uint8_t last_nav_state_ = 255;  // Track nav_state changes
	
	// Callbacks
	void mission_trigger_callback(const std_msgs::msg::Bool::UniquePtr msg);
	void vehicle_command_ack_callback(const px4_msgs::msg::VehicleCommandAck::UniquePtr msg);
	void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::UniquePtr msg);
	void mission_count_callback(const std_msgs::msg::UInt16::UniquePtr msg);
	void current_waypoint_callback(const std_msgs::msg::UInt16::UniquePtr msg);
	void vehicle_global_position_callback(const px4_msgs::msg::VehicleGlobalPosition::UniquePtr msg);
	
	// Control loop
	void control_loop_callback();
	void status_monitor_callback();
	void publish_mission_state();  // Publish state data for MQTT bridge
	
	// State handlers
	void handle_arming_state();
	void handle_mission_active_state();
	void handle_mission_complete_state();
	void handle_error_state();
	
	// Helper functions
	uint64_t get_timestamp();
	void publish_vehicle_command(uint32_t command, float param1 = 0.0f, float param2 = 0.0f,
	                             float param3 = 0.0f, float param4 = 0.0f,
	                             double param5 = 0.0, double param6 = 0.0, float param7 = 0.0f);
	void set_mode_auto_mission();
	void arm_vehicle();
	void start_mission();
};

#endif  // PX4_MISSION__PX4_MISSION_HPP_

