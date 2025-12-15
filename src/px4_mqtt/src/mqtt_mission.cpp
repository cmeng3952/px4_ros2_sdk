/****************************************************************************
 *
 * MQTT Mission Node Implementation
 *
 ****************************************************************************/

#include "px4_mqtt/mqtt_mission.hpp"
#include <unistd.h>
#include <cmath>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

MQTTMission::MQTTMission() : Node("mqtt_mission")
{
	// Get parameters with defaults
	this->declare_parameter<std::string>("uav_name", "uav1");
	this->declare_parameter<std::string>("mqtt_broker", "tcp://localhost:1883");
	this->declare_parameter<std::string>("mqtt_username", "");
	this->declare_parameter<std::string>("mqtt_password", "");
	this->declare_parameter<std::string>("mqtt_mission_topic", "uavmission/waypoints/");
	this->declare_parameter<std::string>("mqtt_status_topic", "uavmission/status/");
	
	uav_name_ = this->get_parameter("uav_name").as_string();
	mqtt_broker_ = this->get_parameter("mqtt_broker").as_string();
	mqtt_username_ = this->get_parameter("mqtt_username").as_string();
	mqtt_password_ = this->get_parameter("mqtt_password").as_string();
	std::string mission_topic_prefix = this->get_parameter("mqtt_mission_topic").as_string();
	std::string status_topic_prefix = this->get_parameter("mqtt_status_topic").as_string();
	mqtt_mission_topic_ = mission_topic_prefix + uav_name_;
	mqtt_status_topic_ = status_topic_prefix + uav_name_;
	
	// Convert mqtt:// to tcp:// if needed
	if (mqtt_broker_.find("mqtt://") == 0) {
		mqtt_broker_.replace(0, 6, "tcp://");
	}

	RCLCPP_INFO(this->get_logger(), "MQTT Mission starting - UAV: %s, Broker: %s", 
		uav_name_.c_str(), mqtt_broker_.c_str());
	RCLCPP_INFO(this->get_logger(), "Mission Topic: %s, Status Topic: %s", 
		mqtt_mission_topic_.c_str(), mqtt_status_topic_.c_str());

	setup_publishers();
	setup_subscribers();
	init_mqtt();
	
	mqtt_reconnect_timer_ = this->create_wall_timer(
		5s, std::bind(&MQTTMission::try_reconnect_mqtt, this));
	
	// No need for separate timer - we publish when we receive state from px4_mission
}

void MQTTMission::setup_publishers()
{
	// Use default QoS (Reliable) to match mavlink_mission.py subscription
	// mavlink_mission.py uses default QoS (Reliable) for /mission/waypoints
	rclcpp::QoS waypoints_qos(10);
	waypoints_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
	waypoints_qos.history(rclcpp::HistoryPolicy::KeepLast);
	
	// Publisher for mission waypoints (to mavlink_mission for upload to PX4)
	waypoints_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
		"/mission/waypoints", waypoints_qos);
	
	// Use BestEffort for trigger to match px4_mission subscription
	rclcpp::QoS trigger_qos(10);
	trigger_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
	trigger_qos.history(rclcpp::HistoryPolicy::KeepLast);
	
	// Publisher for mission trigger (to px4_mission node for execution)
	trigger_pub_ = this->create_publisher<std_msgs::msg::Bool>(
		"/px4_mission/trigger", trigger_qos);
}

void MQTTMission::setup_subscribers()
{
	rclcpp::QoS qos(10);
	qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
	qos.history(rclcpp::HistoryPolicy::KeepLast);
	
	// Subscribe to mission state from px4_mission node (similar to mqtt_estimator)
	mission_state_sub_ = this->create_subscription<std_msgs::msg::String>(
		"/px4_mission/state", qos,
		std::bind(&MQTTMission::px4_mission_state_callback, this, std::placeholders::_1));
}

void MQTTMission::connection_lost(const std::string& /*cause*/)
{
	std::lock_guard<std::mutex> lock(mqtt_mutex_);
	mqtt_connected_ = false;
	reconnect_attempts_ = 0;
	RCLCPP_WARN(this->get_logger(), "MQTT connection lost");
}

void MQTTMission::message_arrived(mqtt::const_message_ptr msg)
{
	try {
		std::string topic = msg->get_topic();
		std::string payload = msg->to_string();
		
		// MQTT message received, no need to log
		
		process_mqtt_mission(topic, payload);
	} catch (const std::exception& e) {
		RCLCPP_ERROR(this->get_logger(), "Error processing MQTT message: %s", e.what());
	}
}

void MQTTMission::delivery_complete(mqtt::delivery_token_ptr /*token*/)
{
	// Optional: handle delivery confirmation
}

void MQTTMission::init_mqtt()
{
	try {
		std::string client_id = "mqtt_mission_" + uav_name_ + "_" +
		                        std::to_string(this->now().nanoseconds()) + "_" +
		                        std::to_string(getpid());
		
		mqtt_client_ = std::make_unique<mqtt::client>(mqtt_broker_, client_id);
		mqtt_client_->set_callback(*this);
		
		mqtt::connect_options opts;
		opts.set_clean_session(true);
		opts.set_automatic_reconnect(true);
		opts.set_keep_alive_interval(60);
		opts.set_connect_timeout(10);
		
		if (!mqtt_username_.empty() && !mqtt_password_.empty()) {
			opts.set_user_name(mqtt_username_);
			opts.set_password(mqtt_password_);
		}
		
		mqtt_client_->connect(opts);
		{
			std::lock_guard<std::mutex> lock(mqtt_mutex_);
			mqtt_connected_ = true;
			reconnect_attempts_ = 0;
		}
		
		subscribe_to_mqtt_topics();
		
		RCLCPP_INFO(this->get_logger(), "Connected to MQTT server and subscribed to topic: %s", 
			mqtt_mission_topic_.c_str());
	} catch (const mqtt::exception& e) {
		std::lock_guard<std::mutex> lock(mqtt_mutex_);
		mqtt_connected_ = false;
		RCLCPP_ERROR(this->get_logger(), "MQTT connection failed: %s", e.what());
	}
}

void MQTTMission::subscribe_to_mqtt_topics()
{
	try {
		std::lock_guard<std::mutex> lock(mqtt_mutex_);
		if (mqtt_client_ && mqtt_client_->is_connected()) {
			mqtt_client_->subscribe(mqtt_mission_topic_, 0);
			RCLCPP_INFO(this->get_logger(), "Subscribed to MQTT topic: %s", mqtt_mission_topic_.c_str());
		}
	} catch (const mqtt::exception& e) {
		RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to MQTT topic: %s", e.what());
	}
}

void MQTTMission::try_reconnect_mqtt()
{
	std::lock_guard<std::mutex> lock(mqtt_mutex_);
	
	if (mqtt_connected_) {
		return;
	}
	
	if (reconnect_attempts_ >= max_reconnect_attempts_) {
		return;
	}
	
	reconnect_attempts_++;
	
	try {
		if (mqtt_client_) {
			mqtt::connect_options opts;
			opts.set_clean_session(true);
			opts.set_automatic_reconnect(true);
			opts.set_keep_alive_interval(60);
			opts.set_connect_timeout(10);
			
			if (!mqtt_username_.empty() && !mqtt_password_.empty()) {
				opts.set_user_name(mqtt_username_);
				opts.set_password(mqtt_password_);
			}
			
			mqtt_client_->connect(opts);
			mqtt_connected_ = true;
			reconnect_attempts_ = 0;
			
			subscribe_to_mqtt_topics();
			
			RCLCPP_INFO(this->get_logger(), "MQTT reconnected");
		}
	} catch (const mqtt::exception& /*e*/) {
		mqtt_connected_ = false;
	}
}

bool MQTTMission::is_mqtt_connected()
{
	std::lock_guard<std::mutex> lock(mqtt_mutex_);
	return mqtt_connected_ && mqtt_client_ && mqtt_client_->is_connected();
}

void MQTTMission::process_mqtt_mission(const std::string& /*topic*/, const std::string& payload)
{
	try {
		json mission_data = json::parse(payload);
		
		// Check if it's a waypoint mission or trigger command
		if (mission_data.contains("waypoints")) {
			handle_mission_waypoints(mission_data);
		} else if (mission_data.contains("trigger") || mission_data.contains("action")) {
			handle_mission_trigger(mission_data);
		} else {
			RCLCPP_WARN(this->get_logger(), "Unknown mission message format");
		}
	} catch (const json::parse_error& e) {
		RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON mission: %s", e.what());
	} catch (const std::exception& e) {
		RCLCPP_ERROR(this->get_logger(), "Error processing mission: %s", e.what());
	}
}

void MQTTMission::handle_mission_waypoints(const json& mission_data)
{
	try {
		if (!mission_data.contains("waypoints") || !mission_data["waypoints"].is_array()) {
			RCLCPP_ERROR(this->get_logger(), "Invalid waypoints format: 'waypoints' must be an array");
			return;
		}
		
		std::vector<float> waypoint_data;
		for (const auto& wp : mission_data["waypoints"]) {
			std::vector<float> wp_vec = parse_waypoint(wp);
			if (wp_vec.size() == 9) {
				waypoint_data.insert(waypoint_data.end(), wp_vec.begin(), wp_vec.end());
			} else {
				RCLCPP_ERROR(this->get_logger(), "Invalid waypoint format, skipping");
			}
		}
		
		if (waypoint_data.empty()) {
			RCLCPP_ERROR(this->get_logger(), "No valid waypoints found");
			return;
		}
		
		// Publish waypoints
		auto waypoints_msg = std_msgs::msg::Float32MultiArray();
		waypoints_msg.data = waypoint_data;
		waypoints_pub_->publish(waypoints_msg);
		
		// Auto-trigger if specified
		bool auto_trigger = mission_data.value("auto_trigger", false);
		if (auto_trigger) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Wait 1s for upload
			auto trigger_msg = std_msgs::msg::Bool();
			trigger_msg.data = true;
			trigger_pub_->publish(trigger_msg);
			// Mission triggered automatically, no need to log
		}
	} catch (const std::exception& e) {
		RCLCPP_ERROR(this->get_logger(), "Error handling waypoints: %s", e.what());
	}
}

void MQTTMission::handle_mission_trigger(const json& trigger_data)
{
	try {
		bool should_trigger = false;
		
		if (trigger_data.contains("trigger")) {
			should_trigger = trigger_data["trigger"].get<bool>();
		} else if (trigger_data.contains("action")) {
			std::string action = trigger_data["action"].get<std::string>();
			should_trigger = (action == "start" || action == "trigger");
		}
		
		if (should_trigger) {
			auto trigger_msg = std_msgs::msg::Bool();
			trigger_msg.data = true;
			trigger_pub_->publish(trigger_msg);
			// Mission trigger published, no need to log
		}
	} catch (const std::exception& e) {
		RCLCPP_ERROR(this->get_logger(), "Error handling trigger: %s", e.what());
	}
}

std::vector<float> MQTTMission::parse_waypoint(const json& wp)
{
	std::vector<float> waypoint;
	
	// Expected format: [frame, command, lat, lon, alt, param1, param2, param3, param4]
	// Or: {"frame": 3, "command": 16, "lat": 47.397742, ...}
	
	if (wp.is_array() && wp.size() == 9) {
		for (const auto& val : wp) {
			waypoint.push_back(val.get<float>());
		}
	} else if (wp.is_object()) {
		waypoint.push_back(wp.value("frame", 3.0f));
		waypoint.push_back(wp.value("command", 16.0f));
		waypoint.push_back(wp.value("lat", 0.0f));
		waypoint.push_back(wp.value("lon", 0.0f));
		waypoint.push_back(wp.value("alt", 0.0f));
		waypoint.push_back(wp.value("param1", 0.0f));
		waypoint.push_back(wp.value("param2", 1.0f));
		waypoint.push_back(wp.value("param3", 0.0f));
		waypoint.push_back(wp.value("param4", 0.0f));
	}
	
	return waypoint;
}

void MQTTMission::px4_mission_state_callback(const std_msgs::msg::String::UniquePtr msg)
{
	// Forward state from px4_mission to MQTT (similar to mqtt_estimator)
	try {
		json state_json = json::parse(msg->data);
		
		// Add UAV name to the state
		state_json["uav_name"] = uav_name_;
		
		// Publish to MQTT
		publish_mission_status_to_mqtt(state_json.dump());
	} catch (const json::parse_error& e) {
		RCLCPP_ERROR(this->get_logger(), "Failed to parse mission state JSON: %s", e.what());
	} catch (const std::exception& e) {
		RCLCPP_ERROR(this->get_logger(), "Error processing mission state: %s", e.what());
	}
}

void MQTTMission::publish_mission_status_to_mqtt(const std::string& state_json)
{
	try {
		std::lock_guard<std::mutex> lock(mqtt_mutex_);
		if (mqtt_client_ && mqtt_client_->is_connected()) {
			mqtt::message_ptr pubmsg = mqtt::make_message(mqtt_status_topic_, state_json);
			pubmsg->set_qos(0);
			mqtt_client_->publish(pubmsg);
		}
	} catch (const std::exception& e) {
		RCLCPP_ERROR(this->get_logger(), "Error publishing status to MQTT: %s", e.what());
	}
}

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<MQTTMission>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

