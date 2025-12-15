/****************************************************************************
 *
 * PX4 Estimator Node Main Entry Point
 *
 ****************************************************************************/

#include "px4_control/px4_estimator.hpp"
#include <iostream>

int main(int argc, char *argv[])
{
	std::cout << "Starting PX4 Estimator..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PX4Estimator>());
	rclcpp::shutdown();
	return 0;
}

