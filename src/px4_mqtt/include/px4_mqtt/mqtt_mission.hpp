/****************************************************************************
 *
 * MQTT Mission Node
 * Subscribes to MQTT broker for mission waypoints and publishes to ROS2
 * Also subscribes to mission status and publishes to MQTT
 *
 ****************************************************************************/

#ifndef PX4_MQTT__MQTT_MISSION_HPP_
#define PX4_MQTT__MQTT_MISSION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <mutex>
#include <string>
#include <mqtt/client.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class MQTTMission : public rclcpp::Node, public virtual mqtt::callback
{
public:
	explicit MQTTMission();

private:
	void setup_publishers();
	void setup_subscribers();
	
	// MQTT callback implementations
	void connection_lost(const std::string& cause) override;
	void message_arrived(mqtt::const_message_ptr msg) override;
	void delivery_complete(mqtt::delivery_token_ptr token) override;
	
	// MQTT operations
	void init_mqtt();
	void try_reconnect_mqtt();
	bool is_mqtt_connected();
	void subscribe_to_mqtt_topics();
	void publish_mission_status_to_mqtt(const std::string& state_json);
	
	// Command processing
	void process_mqtt_mission(const std::string& topic, const std::string& payload);
	void handle_mission_waypoints(const json& mission_data);
	void handle_mission_trigger(const json& trigger_data);
	
	// Helper functions
	std::vector<float> parse_waypoint(const json& wp);
	
	// ROS2 subscribers callbacks
	void px4_mission_state_callback(const std_msgs::msg::String::UniquePtr msg);
	
	// Publishers
	rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr waypoints_pub_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trigger_pub_;
	
	// Subscribers
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mission_state_sub_;
	
	// Timer for MQTT reconnection
	rclcpp::TimerBase::SharedPtr mqtt_reconnect_timer_;
	
	// Thread-safe state storage
	std::mutex mqtt_mutex_;
	bool mqtt_connected_ = false;
	int reconnect_attempts_ = 0;
	const int max_reconnect_attempts_ = 10;
	
	// Mission state (parsed from px4_mission/state)
	
	// MQTT configuration
	std::unique_ptr<mqtt::client> mqtt_client_;
	std::string mqtt_broker_;
	std::string mqtt_username_;
	std::string mqtt_password_;
	std::string mqtt_mission_topic_;
	std::string mqtt_status_topic_;
	std::string uav_name_;
};

#endif  // PX4_MQTT__MQTT_MISSION_HPP_

