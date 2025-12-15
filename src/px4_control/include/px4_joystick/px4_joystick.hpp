#ifndef PX4_CONTROL_PX4_JOYSTICK_HPP
#define PX4_CONTROL_PX4_JOYSTICK_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>

namespace px4_control
{

class PX4Joystick : public rclcpp::Node
{
public:
	PX4Joystick();

private:
	void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
	
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
	rclcpp::Publisher<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manual_control_pub_;
	
	// Joystick axis mappings (configurable via parameters)
	int roll_axis_;
	int pitch_axis_;
	int throttle_axis_;
	int yaw_axis_;
	
	// Axis inversion flags
	bool roll_inverted_;
	bool pitch_inverted_;
	bool throttle_inverted_;
	bool yaw_inverted_;
	
	// Dead zone threshold
	double dead_zone_;
	
	// Timestamp helper
	uint64_t get_timestamp();
};

} // namespace px4_control

#endif // PX4_CONTROL_PX4_JOYSTICK_HPP

