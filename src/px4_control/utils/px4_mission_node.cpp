/****************************************************************************
 *
 * PX4 Mission Node
 *
 ****************************************************************************/

#include "px4_mission/px4_mission.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<PX4Mission>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

