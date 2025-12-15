/****************************************************************************
 *
 * MQTT Estimator Node
 * Subscribes to px4_estimator state topic and publishes to MQTT broker
 *
 ****************************************************************************/

#ifndef PX4_MQTT__MQTT_ESTIMATOR_HPP_
#define PX4_MQTT__MQTT_ESTIMATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <mutex>
#include <string>
#include <mqtt/client.h>

class MQTTEstimator : public rclcpp::Node, public virtual mqtt::callback
{
public:
	explicit MQTTEstimator();

private:
	void setup_subscriptions();
	
	// MQTT callback implementations
	void connection_lost(const std::string& cause) override;
	void message_arrived(mqtt::const_message_ptr msg) override;
	void delivery_complete(mqtt::delivery_token_ptr token) override;
	
	// MQTT operations
	void init_mqtt();
	void try_reconnect_mqtt();
	void publish_state_to_mqtt(const std::string& json_data);
	bool is_mqtt_connected();
	
	// ROS2 subscription callback
	void px4_estimator_state_callback(const std_msgs::msg::String::UniquePtr msg);

	// Subscriber
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr px4_estimator_state_sub_;

	// Timer for MQTT reconnection
	rclcpp::TimerBase::SharedPtr mqtt_reconnect_timer_;

	// Thread-safe state storage
	std::mutex state_mutex_;
	std::mutex mqtt_mutex_;
	bool mqtt_connected_ = false;
	int reconnect_attempts_ = 0;
	const int max_reconnect_attempts_ = 10;
	std::string latest_state_json_;
	bool state_received_ = false;

	// MQTT configuration
	std::unique_ptr<mqtt::client> mqtt_client_;
	std::string mqtt_broker_;
	std::string mqtt_username_;
	std::string mqtt_password_;
	std::string mqtt_state_topic_;
	std::string uav_name_;
};

#endif  // PX4_MQTT__MQTT_ESTIMATOR_HPP_
