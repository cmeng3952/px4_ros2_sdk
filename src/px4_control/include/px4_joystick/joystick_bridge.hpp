#ifndef PX4_CONTROL_JOYSTICK_BRIDGE_HPP
#define PX4_CONTROL_JOYSTICK_BRIDGE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <linux/joystick.h>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <memory>

namespace px4_control
{

class JoystickBridge : public rclcpp::Node
{
public:
	JoystickBridge();
	~JoystickBridge();

private:
	void readJoystick();
	void publishJoy();
	
	rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
	
	// Joystick device file path
	std::string device_path_;
	int joystick_fd_;
	
	// Joystick state
	std::vector<float> axes_;
	std::vector<int> buttons_;
	
	// Threading
	std::thread read_thread_;
	std::atomic<bool> running_;
	
	// Publishing rate
	double publish_rate_;
	rclcpp::TimerBase::SharedPtr publish_timer_;
	
	// Mutex for thread safety
	std::mutex state_mutex_;
};

} // namespace px4_control

#endif // PX4_CONTROL_JOYSTICK_BRIDGE_HPP

