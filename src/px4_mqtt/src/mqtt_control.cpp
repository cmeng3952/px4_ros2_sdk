/****************************************************************************
 *
 * MQTT Control Node Implementation
 *
 ****************************************************************************/

#include "px4_mqtt/mqtt_control.hpp"
#include <unistd.h>
#include <cmath>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

MQTTControl::MQTTControl() : Node("mqtt_control")
{
	// Get parameters with defaults
	this->declare_parameter<std::string>("uav_name", "uav1");
	this->declare_parameter<std::string>("mqtt_broker", "tcp://localhost:1883");
	this->declare_parameter<std::string>("mqtt_username", "");
	this->declare_parameter<std::string>("mqtt_password", "");
	this->declare_parameter<std::string>("mqtt_command_topic", "uavcontrol/command/");
	
	uav_name_ = this->get_parameter("uav_name").as_string();
	mqtt_broker_ = this->get_parameter("mqtt_broker").as_string();
	mqtt_username_ = this->get_parameter("mqtt_username").as_string();
	mqtt_password_ = this->get_parameter("mqtt_password").as_string();
	std::string topic_prefix = this->get_parameter("mqtt_command_topic").as_string();
	mqtt_command_topic_ = topic_prefix + uav_name_;
	
	// Convert mqtt:// to tcp:// if needed
	if (mqtt_broker_.find("mqtt://") == 0) {
		mqtt_broker_.replace(0, 6, "tcp://");
	}

	RCLCPP_INFO(this->get_logger(), "MQTT Control starting - UAV: %s, Broker: %s, Topic: %s", 
		uav_name_.c_str(), mqtt_broker_.c_str(), mqtt_command_topic_.c_str());

	setup_publishers();
	init_mqtt();
	
	mqtt_reconnect_timer_ = this->create_wall_timer(
		5s, std::bind(&MQTTControl::try_reconnect_mqtt, this));
}

void MQTTControl::setup_publishers()
{
	rclcpp::QoS qos(10);
	qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
	qos.history(rclcpp::HistoryPolicy::KeepLast);
	
	// Publisher for vehicle commands (arm, disarm, mode changes)
	// Publish to px4_control node, which will forward to PX4
	vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
		"/px4_control/vehicle_command", qos);
	
	// Publisher for trajectory setpoint commands (position/velocity)
	// Publish to px4_control node, which will forward to PX4
	trajectory_setpoint_command_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
		"/px4_control/trajectory_setpoint_command", qos);
}

void MQTTControl::connection_lost(const std::string& /*cause*/)
{
	std::lock_guard<std::mutex> lock(mqtt_mutex_);
	mqtt_connected_ = false;
	reconnect_attempts_ = 0;
	RCLCPP_WARN(this->get_logger(), "MQTT connection lost");
}

void MQTTControl::message_arrived(mqtt::const_message_ptr msg)
{
	try {
		std::string topic = msg->get_topic();
		std::string payload = msg->to_string();
		
		RCLCPP_DEBUG(this->get_logger(), "Received MQTT message on topic '%s': %s", 
			topic.c_str(), payload.c_str());
		
		process_mqtt_command(topic, payload);
	} catch (const std::exception& e) {
		RCLCPP_ERROR(this->get_logger(), "Error processing MQTT message: %s", e.what());
	}
}

void MQTTControl::delivery_complete(mqtt::delivery_token_ptr /*token*/)
{
	// Optional: handle delivery confirmation
}

void MQTTControl::init_mqtt()
{
	try {
		std::string client_id = "mqtt_control_" + uav_name_ + "_" +
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
			mqtt_command_topic_.c_str());
	} catch (const mqtt::exception& e) {
		std::lock_guard<std::mutex> lock(mqtt_mutex_);
		mqtt_connected_ = false;
		RCLCPP_ERROR(this->get_logger(), "MQTT connection failed: %s", e.what());
	}
}

void MQTTControl::subscribe_to_mqtt_topics()
{
	try {
		std::lock_guard<std::mutex> lock(mqtt_mutex_);
		if (mqtt_client_ && mqtt_client_->is_connected()) {
			mqtt_client_->subscribe(mqtt_command_topic_, 0);
			RCLCPP_INFO(this->get_logger(), "Subscribed to MQTT topic: %s", mqtt_command_topic_.c_str());
		}
	} catch (const mqtt::exception& e) {
		RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to MQTT topic: %s", e.what());
	}
}

void MQTTControl::try_reconnect_mqtt()
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

bool MQTTControl::is_mqtt_connected()
{
	std::lock_guard<std::mutex> lock(mqtt_mutex_);
	return mqtt_connected_ && mqtt_client_ && mqtt_client_->is_connected();
}

void MQTTControl::process_mqtt_command(const std::string& topic, const std::string& payload)
{
	try {
		json cmd = json::parse(payload);
		handle_command(cmd);
	} catch (const json::parse_error& e) {
		RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON command: %s", e.what());
	} catch (const std::exception& e) {
		RCLCPP_ERROR(this->get_logger(), "Error processing command: %s", e.what());
	}
}

void MQTTControl::handle_command(const json& cmd)
{
	if (!cmd.contains("command")) {
		RCLCPP_WARN(this->get_logger(), "Command JSON missing 'command' field");
		return;
	}
	
	std::string command = cmd["command"].get<std::string>();
	
	RCLCPP_INFO(this->get_logger(), "Processing command: %s", command.c_str());
	
	if (command == "arm") {
		arm();
	} else if (command == "disarm") {
		disarm();
	} else if (command == "arm_and_takeoff") {
		arm_and_takeoff();
	} else if (command == "set_mode_offboard") {
		set_mode_offboard();
	} else if (command == "set_mode_land") {
		set_mode_land();
	} else if (command == "set_mode_hold") {
		set_mode_hold();
	} else if (command == "position") {
		if (!cmd.contains("x") || !cmd.contains("y") || !cmd.contains("z")) {
			RCLCPP_WARN(this->get_logger(), "Position command missing x, y, or z");
			return;
		}
		float x = cmd["x"].get<float>();
		float y = cmd["y"].get<float>();
		float z = cmd["z"].get<float>();
		float yaw = cmd.value("yaw", 0.0f);
		publish_position_setpoint(x, y, z, yaw);
	} else if (command == "velocity") {
		if (!cmd.contains("vx") || !cmd.contains("vy") || !cmd.contains("vz")) {
			RCLCPP_WARN(this->get_logger(), "Velocity command missing vx, vy, or vz");
			return;
		}
		float vx = cmd["vx"].get<float>();
		float vy = cmd["vy"].get<float>();
		float vz = cmd["vz"].get<float>();
		float yaw = cmd.value("yaw", 0.0f);
		publish_velocity_setpoint(vx, vy, vz, yaw);
	} else {
		RCLCPP_WARN(this->get_logger(), "Unknown command: %s", command.c_str());
	}
}

uint64_t MQTTControl::get_timestamp()
{
	return this->get_clock()->now().nanoseconds() / 1000;
}

void MQTTControl::publish_vehicle_command(uint32_t command, float param1, float param2,
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

void MQTTControl::arm()
{
	RCLCPP_INFO(this->get_logger(), "Arming vehicle via MQTT");
	publish_vehicle_command(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
		1.0f
	);
}

void MQTTControl::disarm()
{
	RCLCPP_INFO(this->get_logger(), "Disarming vehicle via MQTT");
	publish_vehicle_command(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
		0.0f
	);
}

void MQTTControl::arm_and_takeoff()
{
	RCLCPP_INFO(this->get_logger(), "Arming and taking off via MQTT");
	arm();
	// Wait a bit for arm to process
	std::this_thread::sleep_for(std::chrono::milliseconds(3000));
	set_mode_auto_takeoff();
}

void MQTTControl::set_mode_offboard()
{
	RCLCPP_INFO(this->get_logger(), "Setting OFFBOARD mode via MQTT");
	publish_vehicle_command(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
		1.0f, 6.0f, 0.0f
	);
}

void MQTTControl::set_mode_land()
{
	RCLCPP_INFO(this->get_logger(), "Setting LAND mode via MQTT");
	publish_vehicle_command(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
		1.0f, 4.0f, 6.0f
	);
}

void MQTTControl::set_mode_hold()
{
	RCLCPP_INFO(this->get_logger(), "Setting HOLD mode via MQTT");
	publish_vehicle_command(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
		1.0f, 4.0f, 3.0f
	);
}

void MQTTControl::set_mode_auto_takeoff()
{
	publish_vehicle_command(
		px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
		1.0f, 4.0f, 2.0f
	);
}

void MQTTControl::publish_position_setpoint(float x, float y, float z, float yaw)
{
	RCLCPP_INFO(this->get_logger(), "Publishing position setpoint via MQTT: [%.2f, %.2f, %.2f], yaw=%.2f", 
		x, y, z, yaw);
	
	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.timestamp = get_timestamp();
	msg.position[0] = x;
	msg.position[1] = y;
	msg.position[2] = z;
	msg.velocity[0] = std::nanf("");
	msg.velocity[1] = std::nanf("");
	msg.velocity[2] = std::nanf("");
	msg.acceleration[0] = std::nanf("");
	msg.acceleration[1] = std::nanf("");
	msg.acceleration[2] = std::nanf("");
	msg.yaw = yaw;
	msg.yawspeed = 0.0f;

	trajectory_setpoint_command_pub_->publish(msg);
}

void MQTTControl::publish_velocity_setpoint(float vx, float vy, float vz, float yaw)
{
	RCLCPP_INFO(this->get_logger(), "Publishing velocity setpoint via MQTT: [%.2f, %.2f, %.2f], yaw=%.2f", 
		vx, vy, vz, yaw);
	
	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.timestamp = get_timestamp();
	msg.position[0] = std::nanf("");
	msg.position[1] = std::nanf("");
	msg.position[2] = std::nanf("");
	msg.velocity[0] = vx;
	msg.velocity[1] = vy;
	msg.velocity[2] = vz;
	msg.acceleration[0] = std::nanf("");
	msg.acceleration[1] = std::nanf("");
	msg.acceleration[2] = std::nanf("");
	msg.yaw = yaw;
	msg.yawspeed = 0.0f;

	trajectory_setpoint_command_pub_->publish(msg);
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<MQTTControl>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

