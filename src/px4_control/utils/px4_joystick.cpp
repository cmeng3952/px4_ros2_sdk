#include "px4_joystick/px4_joystick.hpp"
#include <cmath>

namespace px4_control
{

PX4Joystick::PX4Joystick()
	: Node("px4_joystick")
{
	// Declare parameters
	this->declare_parameter<int>("roll_axis", 0);
	this->declare_parameter<int>("pitch_axis", 1);
	this->declare_parameter<int>("throttle_axis", 2);
	this->declare_parameter<int>("yaw_axis", 3);
	
	this->declare_parameter<bool>("roll_inverted", false);
	this->declare_parameter<bool>("pitch_inverted", false);
	this->declare_parameter<bool>("throttle_inverted", false);
	this->declare_parameter<bool>("yaw_inverted", false);
	
	this->declare_parameter<double>("dead_zone", 0.05);
	
	// Get parameters
	roll_axis_ = this->get_parameter("roll_axis").as_int();
	pitch_axis_ = this->get_parameter("pitch_axis").as_int();
	throttle_axis_ = this->get_parameter("throttle_axis").as_int();
	yaw_axis_ = this->get_parameter("yaw_axis").as_int();
	
	roll_inverted_ = this->get_parameter("roll_inverted").as_bool();
	pitch_inverted_ = this->get_parameter("pitch_inverted").as_bool();
	throttle_inverted_ = this->get_parameter("throttle_inverted").as_bool();
	yaw_inverted_ = this->get_parameter("yaw_inverted").as_bool();
	
	dead_zone_ = this->get_parameter("dead_zone").as_double();
	
	// Create subscriber
	joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
		"joy", 10,
		std::bind(&PX4Joystick::joyCallback, this, std::placeholders::_1));
	
	// Create publisher with appropriate QoS for PX4
	rclcpp::QoS qos(10);
	qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
	qos.history(rclcpp::HistoryPolicy::KeepLast);
	
	manual_control_pub_ = this->create_publisher<px4_msgs::msg::ManualControlSetpoint>(
		"/fmu/in/manual_control_input", qos);
	
	RCLCPP_INFO(this->get_logger(), "PX4Joystick node started");
	RCLCPP_INFO(this->get_logger(), "  Roll axis: %d (inverted: %s)", roll_axis_, roll_inverted_ ? "yes" : "no");
	RCLCPP_INFO(this->get_logger(), "  Pitch axis: %d (inverted: %s)", pitch_axis_, pitch_inverted_ ? "yes" : "no");
	RCLCPP_INFO(this->get_logger(), "  Throttle axis: %d (inverted: %s)", throttle_axis_, throttle_inverted_ ? "yes" : "no");
	RCLCPP_INFO(this->get_logger(), "  Yaw axis: %d (inverted: %s)", yaw_axis_, yaw_inverted_ ? "yes" : "no");
	RCLCPP_INFO(this->get_logger(), "  Dead zone: %.3f", dead_zone_);
}

uint64_t PX4Joystick::get_timestamp()
{
	return this->get_clock()->now().nanoseconds() / 1000;
}

void PX4Joystick::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
	px4_msgs::msg::ManualControlSetpoint manual_control_msg;
	
	manual_control_msg.timestamp = get_timestamp();
	manual_control_msg.timestamp_sample = get_timestamp();
	manual_control_msg.valid = true;
	manual_control_msg.data_source = px4_msgs::msg::ManualControlSetpoint::SOURCE_MAVLINK_0;
	
	// Helper function to apply dead zone and inversion
	auto process_axis = [this](float value, bool inverted) -> float {
		// Apply dead zone
		if (std::abs(value) < dead_zone_) {
			return 0.0f;
		}
		// Apply inversion
		return inverted ? -value : value;
	};
	
	// Map joystick axes to control channels
	// Check bounds to avoid out-of-range access
	if (roll_axis_ >= 0 && roll_axis_ < static_cast<int>(msg->axes.size())) {
		manual_control_msg.roll = process_axis(msg->axes[roll_axis_], roll_inverted_);
	} else {
		manual_control_msg.roll = 0.0f;
	}
	
	if (pitch_axis_ >= 0 && pitch_axis_ < static_cast<int>(msg->axes.size())) {
		manual_control_msg.pitch = process_axis(msg->axes[pitch_axis_], pitch_inverted_);
	} else {
		manual_control_msg.pitch = 0.0f;
	}
	
	if (throttle_axis_ >= 0 && throttle_axis_ < static_cast<int>(msg->axes.size())) {
		manual_control_msg.throttle = process_axis(msg->axes[throttle_axis_], throttle_inverted_);
	} else {
		manual_control_msg.throttle = -1.0f; // Default to minimum throttle
	}
	
	if (yaw_axis_ >= 0 && yaw_axis_ < static_cast<int>(msg->axes.size())) {
		manual_control_msg.yaw = process_axis(msg->axes[yaw_axis_], yaw_inverted_);
	} else {
		manual_control_msg.yaw = 0.0f;
	}
	
	// Set auxiliary channels to NaN (not used)
	manual_control_msg.flaps = std::nanf("");
	manual_control_msg.aux1 = std::nanf("");
	manual_control_msg.aux2 = std::nanf("");
	manual_control_msg.aux3 = std::nanf("");
	manual_control_msg.aux4 = std::nanf("");
	manual_control_msg.aux5 = std::nanf("");
	manual_control_msg.aux6 = std::nanf("");
	
	// Check if sticks are moving
	manual_control_msg.sticks_moving = (
		std::abs(manual_control_msg.roll) > dead_zone_ ||
		std::abs(manual_control_msg.pitch) > dead_zone_ ||
		std::abs(manual_control_msg.throttle) > dead_zone_ ||
		std::abs(manual_control_msg.yaw) > dead_zone_
	);
	
	// Map buttons (if needed)
	// Combine buttons into uint16_t bitfield
	uint16_t buttons = 0;
	for (size_t i = 0; i < msg->buttons.size() && i < 16; ++i) {
		if (msg->buttons[i] > 0) {
			buttons |= (1 << i);
		}
	}
	manual_control_msg.buttons = buttons;
	
	manual_control_pub_->publish(manual_control_msg);
}

} // namespace px4_control

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<px4_control::PX4Joystick>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

