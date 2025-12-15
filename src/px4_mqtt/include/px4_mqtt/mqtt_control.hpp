/****************************************************************************
 *
 * MQTT Control Node
 * Subscribes to MQTT broker and publishes control commands to PX4
 *
 ****************************************************************************/

#ifndef PX4_MQTT__MQTT_CONTROL_HPP_
#define PX4_MQTT__MQTT_CONTROL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <memory>
#include <mutex>
#include <string>
#include <mqtt/client.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class MQTTControl : public rclcpp::Node, public virtual mqtt::callback
{
public:
	explicit MQTTControl();

private:
	void setup_publishers();
	
	// MQTT callback implementations
	void connection_lost(const std::string& cause) override;
	void message_arrived(mqtt::const_message_ptr msg) override;
	void delivery_complete(mqtt::delivery_token_ptr token) override;
	
	// MQTT operations
	void init_mqtt();
	void try_reconnect_mqtt();
	bool is_mqtt_connected();
	void subscribe_to_mqtt_topics();
	
	// Command processing
	void process_mqtt_command(const std::string& topic, const std::string& payload);
	void handle_command(const json& cmd);
	
	// PX4 command publishing functions
	void arm();
	void disarm();
	void arm_and_takeoff();
	void set_mode_offboard();
	void set_mode_land();
	void set_mode_hold();
	void set_mode_auto_takeoff();
	void publish_position_setpoint(float x, float y, float z, float yaw = 0.0f);
	void publish_velocity_setpoint(float vx, float vy, float vz, float yaw = 0.0f);
	
	// Helper functions
	void publish_vehicle_command(uint32_t command, float param1 = 0.0f, float param2 = 0.0f,
	                             float param3 = 0.0f, float param4 = 0.0f, 
	                             double param5 = 0.0, double param6 = 0.0, float param7 = 0.0f);
	uint64_t get_timestamp();

	// Publishers
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_command_pub_;

	// Timer for MQTT reconnection
	rclcpp::TimerBase::SharedPtr mqtt_reconnect_timer_;

	// Thread-safe state storage
	std::mutex mqtt_mutex_;
	bool mqtt_connected_ = false;
	int reconnect_attempts_ = 0;
	const int max_reconnect_attempts_ = 10;

	// MQTT configuration
	std::unique_ptr<mqtt::client> mqtt_client_;
	std::string mqtt_broker_;
	std::string mqtt_username_;
	std::string mqtt_password_;
	std::string mqtt_command_topic_;
	std::string uav_name_;
};

#endif  // PX4_MQTT__MQTT_CONTROL_HPP_

