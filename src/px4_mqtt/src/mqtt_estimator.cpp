/****************************************************************************
 *
 * MQTT Estimator Node Implementation
 *
 ****************************************************************************/

#include "px4_mqtt/mqtt_estimator.hpp"
#include <unistd.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace std::chrono_literals;

MQTTEstimator::MQTTEstimator() : Node("mqtt_estimator")
{
	// Get parameters with defaults
	this->declare_parameter<std::string>("uav_name", "uav1");
	this->declare_parameter<std::string>("mqtt_broker", "tcp://localhost:1883");
	this->declare_parameter<std::string>("mqtt_username", "");
	this->declare_parameter<std::string>("mqtt_password", "");
	this->declare_parameter<std::string>("mqtt_state_topic", "uavcontrol/state/");
	
	uav_name_ = this->get_parameter("uav_name").as_string();
	mqtt_broker_ = this->get_parameter("mqtt_broker").as_string();
	mqtt_username_ = this->get_parameter("mqtt_username").as_string();
	mqtt_password_ = this->get_parameter("mqtt_password").as_string();
	std::string topic_prefix = this->get_parameter("mqtt_state_topic").as_string();
	mqtt_state_topic_ = topic_prefix + uav_name_;
	
	// Convert mqtt:// to tcp:// if needed
	if (mqtt_broker_.find("mqtt://") == 0) {
		mqtt_broker_.replace(0, 6, "tcp://");
	}

	RCLCPP_INFO(this->get_logger(), "MQTT Estimator starting - UAV: %s, Broker: %s, Topic: %s", 
		uav_name_.c_str(), mqtt_broker_.c_str(), mqtt_state_topic_.c_str());

	setup_subscriptions();
	init_mqtt();
	
	mqtt_reconnect_timer_ = this->create_wall_timer(
		5s, std::bind(&MQTTEstimator::try_reconnect_mqtt, this));
}

void MQTTEstimator::setup_subscriptions()
{
	rclcpp::QoS qos(10);
	qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
	qos.history(rclcpp::HistoryPolicy::KeepLast);
	
	px4_estimator_state_sub_ = this->create_subscription<std_msgs::msg::String>(
		"/px4_estimator/state", qos,
		std::bind(&MQTTEstimator::px4_estimator_state_callback, this, std::placeholders::_1));
}

void MQTTEstimator::connection_lost(const std::string& /*cause*/)
{
	std::lock_guard<std::mutex> lock(mqtt_mutex_);
	mqtt_connected_ = false;
	reconnect_attempts_ = 0;
}

void MQTTEstimator::message_arrived(mqtt::const_message_ptr /*msg*/)
{
	// Not used - we only publish, not subscribe
}

void MQTTEstimator::delivery_complete(mqtt::delivery_token_ptr /*token*/)
{
	// Optional: handle delivery confirmation
}

void MQTTEstimator::init_mqtt()
{
	try {
		std::string client_id = "mqtt_estimator_" + uav_name_ + "_" +
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
		RCLCPP_INFO(this->get_logger(), "Connected to MQTT server");
	} catch (const mqtt::exception& e) {
		std::lock_guard<std::mutex> lock(mqtt_mutex_);
		mqtt_connected_ = false;
		RCLCPP_ERROR(this->get_logger(), "MQTT connection failed: %s", e.what());
	}
}

void MQTTEstimator::try_reconnect_mqtt()
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
			RCLCPP_INFO(this->get_logger(), "MQTT reconnected");
		}
	} catch (const mqtt::exception& /*e*/) {
		mqtt_connected_ = false;
	}
}

bool MQTTEstimator::is_mqtt_connected()
{
	std::lock_guard<std::mutex> lock(mqtt_mutex_);
	return mqtt_connected_ && mqtt_client_ && mqtt_client_->is_connected();
}

void MQTTEstimator::publish_state_to_mqtt(const std::string& json_data)
{
	if (!is_mqtt_connected()) {
		return;
	}
	
	try {
		std::lock_guard<std::mutex> mqtt_lock(mqtt_mutex_);
		if (mqtt_client_ && mqtt_client_->is_connected()) {
			json j = json::parse(json_data);
			j["uav_id"] = uav_name_;
			j["uav_name"] = uav_name_;
			
			auto msg = mqtt::make_message(mqtt_state_topic_, j.dump());
			msg->set_qos(0);
			mqtt_client_->publish(msg);
		}
	} catch (...) {
		std::lock_guard<std::mutex> mqtt_lock(mqtt_mutex_);
		mqtt_connected_ = false;
	}
}

void MQTTEstimator::px4_estimator_state_callback(const std_msgs::msg::String::UniquePtr msg)
{
	std::lock_guard<std::mutex> lock(state_mutex_);
	latest_state_json_ = msg->data;
	state_received_ = true;
	publish_state_to_mqtt(latest_state_json_);
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<MQTTEstimator>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
