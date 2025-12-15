/****************************************************************************
 *
 * PX4 Control Node Main Entry Point
 *
 ****************************************************************************/

#include "px4_control/px4_control.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	auto node = std::make_shared<PX4ControlNode>();
	rclcpp::spin(node);
	
	rclcpp::shutdown();
	return 0;
}

