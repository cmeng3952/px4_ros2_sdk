#include "px4_joystick/joystick_bridge.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <cstring>
#include <chrono>
#include <thread>
#include <mutex>

namespace px4_control
{

JoystickBridge::JoystickBridge()
	: Node("joystick_bridge")
	, joystick_fd_(-1)
	, running_(true)
	, publish_rate_(50.0)  // 50 Hz default
{
	// Declare parameters
	this->declare_parameter<std::string>("device", "/dev/input/js0");
	this->declare_parameter<double>("publish_rate", 50.0);
	
	// Get parameters
	device_path_ = this->get_parameter("device").as_string();
	publish_rate_ = this->get_parameter("publish_rate").as_double();
	
	RCLCPP_INFO(this->get_logger(), "Opening joystick device: %s", device_path_.c_str());
	
	// Open joystick device
	joystick_fd_ = open(device_path_.c_str(), O_RDONLY);
	if (joystick_fd_ < 0)
	{
		RCLCPP_ERROR(this->get_logger(), "Could not open joystick device %s: %s", 
			device_path_.c_str(), strerror(errno));
		RCLCPP_INFO(this->get_logger(), "Available joystick devices:");
		RCLCPP_INFO(this->get_logger(), "Try: ls -la /dev/input/js*");
		return;
	}
	
	// Get joystick info
	char name[128];
	if (ioctl(joystick_fd_, JSIOCGNAME(sizeof(name)), name) < 0)
	{
		strncpy(name, "Unknown", sizeof(name));
	}
	
	uint8_t num_axes, num_buttons;
	ioctl(joystick_fd_, JSIOCGAXES, &num_axes);
	ioctl(joystick_fd_, JSIOCGBUTTONS, &num_buttons);
	
	RCLCPP_INFO(this->get_logger(), "Joystick name: %s", name);
	RCLCPP_INFO(this->get_logger(), "Number of axes: %d", num_axes);
	RCLCPP_INFO(this->get_logger(), "Number of buttons: %d", num_buttons);
	
	// Initialize axes and buttons vectors
	axes_.resize(num_axes, 0.0f);
	buttons_.resize(num_buttons, 0);
	
	// Set non-blocking mode
	int flags = fcntl(joystick_fd_, F_GETFL, 0);
	fcntl(joystick_fd_, F_SETFL, flags | O_NONBLOCK);
	
	// Create publisher
	joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("joy", 10);
	
	// Start reading thread
	read_thread_ = std::thread(&JoystickBridge::readJoystick, this);
	
	// Create timer for publishing
	auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_));
	publish_timer_ = this->create_wall_timer(
		period, std::bind(&JoystickBridge::publishJoy, this));
	
	RCLCPP_INFO(this->get_logger(), "Joystick bridge started. Publishing to /joy topic at %.1f Hz", publish_rate_);
}

JoystickBridge::~JoystickBridge()
{
	running_ = false;
	if (read_thread_.joinable())
	{
		read_thread_.join();
	}
	if (joystick_fd_ >= 0)
	{
		close(joystick_fd_);
	}
}

void JoystickBridge::readJoystick()
{
	js_event event;
	
	while (running_ && rclcpp::ok())
	{
		ssize_t bytes = read(joystick_fd_, &event, sizeof(event));
		
		if (bytes == sizeof(event))
		{
			std::lock_guard<std::mutex> lock(state_mutex_);
			
			switch (event.type & ~JS_EVENT_INIT)
			{
				case JS_EVENT_AXIS:
				{
					if (event.number < axes_.size())
					{
						// Normalize axis value to [-1.0, 1.0]
						axes_[event.number] = event.value / 32767.0f;
					}
					break;
				}
				case JS_EVENT_BUTTON:
				{
					if (event.number < buttons_.size())
					{
						buttons_[event.number] = event.value;
					}
					break;
				}
			}
		}
		else if (bytes < 0)
		{
			// No data available (non-blocking read)
			if (errno != EAGAIN && errno != EWOULDBLOCK)
			{
				RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
					"Error reading joystick: %s", strerror(errno));
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
}

void JoystickBridge::publishJoy()
{
	if (joystick_fd_ < 0)
	{
		return;
	}
	
	auto joy_msg = sensor_msgs::msg::Joy();
	
	{
		std::lock_guard<std::mutex> lock(state_mutex_);
		joy_msg.axes = axes_;
		joy_msg.buttons = buttons_;
	}
	
	joy_msg.header.stamp = this->now();
	joy_msg.header.frame_id = "joystick";
	
	joy_pub_->publish(joy_msg);
}

} // namespace px4_control

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<px4_control::JoystickBridge>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

