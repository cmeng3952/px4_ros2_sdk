/****************************************************************************
 *
 * PX4 Estimator Node Implementation
 *
 ****************************************************************************/

#include "px4_control/px4_estimator.hpp"
#include <iomanip>
#include <cmath>
#include <sstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

using namespace std::chrono_literals;

PX4Estimator::PX4Estimator() : Node("px4_estimator")
{
	// Configure QoS profiles according to PX4 DDS requirements
	// PX4 publishes with BEST_EFFORT + TRANSIENT_LOCAL for most topics
	// Sensor data: BEST_EFFORT reliability, KEEP_LAST history, depth 5
	rmw_qos_profile_t sensor_qos_profile = rmw_qos_profile_sensor_data;
	auto sensor_qos = rclcpp::QoS(
		rclcpp::QoSInitialization(sensor_qos_profile.history, 5), 
		sensor_qos_profile
	);
	
	// Status/Control data: BEST_EFFORT + TRANSIENT_LOCAL (matching PX4 publisher QoS)
	// This matches PX4's default QoS settings for status and control topics
	rclcpp::QoS status_qos(10);
	status_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
	status_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
	status_qos.history(rclcpp::HistoryPolicy::KeepLast);
	
	// Default QoS for other topics: BEST_EFFORT, KEEP_LAST
	rclcpp::QoS default_qos(10);
	default_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
	default_qos.history(rclcpp::HistoryPolicy::KeepLast);
	
	setup_subscriptions(sensor_qos, status_qos, default_qos);
	
	// Create publisher for state data (JSON format for MQTT bridge)
	state_pub_ = this->create_publisher<std_msgs::msg::String>(
		"/px4_estimator/state", 10);
	
	timer_ = this->create_wall_timer(500ms, std::bind(&PX4Estimator::print_status, this));
	
	// Create timer to publish state data (for MQTT bridge)
	state_timer_ = this->create_wall_timer(1000ms, [this]() {
		this->publish_state();
	});
	
	RCLCPP_INFO(this->get_logger(), "PX4 Estimator started");
}

void PX4Estimator::setup_subscriptions(
	const rclcpp::QoS &sensor_qos,
	const rclcpp::QoS &status_qos,
	const rclcpp::QoS &default_qos)
{
	// Sensor topics - use sensor_qos (BEST_EFFORT, KEEP_LAST depth 5)
	// These are high-frequency, loss-tolerant data streams
	sensor_combined_sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
		"/fmu/out/sensor_combined", sensor_qos,
		[this](const px4_msgs::msg::SensorCombined::UniquePtr msg) {
			sensor_combined_ = *msg;
			sensor_combined_received_ = true;
		});

	vehicle_gps_position_sub_ = this->create_subscription<px4_msgs::msg::SensorGps>(
		"/fmu/out/vehicle_gps_position", sensor_qos,
		[this](const px4_msgs::msg::SensorGps::UniquePtr msg) {
			vehicle_gps_position_ = *msg;
			gps_received_ = true;
		});

	timesync_status_sub_ = this->create_subscription<px4_msgs::msg::TimesyncStatus>(
		"/fmu/out/timesync_status", default_qos,
		[this](const px4_msgs::msg::TimesyncStatus::UniquePtr msg) {
			timesync_status_ = *msg;
			timesync_received_ = true;
		});

	// Vehicle state topics - use default_qos (BEST_EFFORT, KEEP_LAST depth 10)
	// These are high-frequency state updates
	vehicle_attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
		"/fmu/out/vehicle_attitude", default_qos,
		[this](const px4_msgs::msg::VehicleAttitude::UniquePtr msg) {
			vehicle_attitude_ = *msg;
			attitude_received_ = true;
		});

	vehicle_local_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
		"/fmu/out/vehicle_local_position", default_qos,
		[this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
			vehicle_local_position_ = *msg;
			local_position_received_ = true;
		});

	vehicle_global_position_sub_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
		"/fmu/out/vehicle_global_position", default_qos,
		[this](const px4_msgs::msg::VehicleGlobalPosition::UniquePtr msg) {
			vehicle_global_position_ = *msg;
			global_position_received_ = true;
		});

	vehicle_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
		"/fmu/out/vehicle_odometry", default_qos,
		[this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
			vehicle_odometry_ = *msg;
			odometry_received_ = true;
		});

	// Control topics - use status_qos (BEST_EFFORT + TRANSIENT_LOCAL)
	// Matching PX4's publisher QoS settings for control topics
	vehicle_control_mode_sub_ = this->create_subscription<px4_msgs::msg::VehicleControlMode>(
		"/fmu/out/vehicle_control_mode", status_qos,
		[this](const px4_msgs::msg::VehicleControlMode::UniquePtr msg) {
			vehicle_control_mode_ = *msg;
			control_mode_received_ = true;
		});

	manual_control_setpoint_sub_ = this->create_subscription<px4_msgs::msg::ManualControlSetpoint>(
		"/fmu/out/manual_control_setpoint", status_qos,
		[this](const px4_msgs::msg::ManualControlSetpoint::UniquePtr msg) {
			manual_control_setpoint_ = *msg;
			manual_control_received_ = true;
		});

	position_setpoint_triplet_sub_ = this->create_subscription<px4_msgs::msg::PositionSetpointTriplet>(
		"/fmu/out/position_setpoint_triplet", status_qos,
		[this](const px4_msgs::msg::PositionSetpointTriplet::UniquePtr msg) {
			position_setpoint_triplet_ = *msg;
			setpoint_triplet_received_ = true;
		});

	// Health status topics - use status_qos (BEST_EFFORT + TRANSIENT_LOCAL)
	// Matching PX4's publisher QoS settings for status topics
	vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
		"/fmu/out/vehicle_status", status_qos,
		[this](const px4_msgs::msg::VehicleStatus::UniquePtr msg) {
			vehicle_status_ = *msg;
			status_received_ = true;
		});

	estimator_status_flags_sub_ = this->create_subscription<px4_msgs::msg::EstimatorStatusFlags>(
		"/fmu/out/estimator_status_flags", status_qos,
		[this](const px4_msgs::msg::EstimatorStatusFlags::UniquePtr msg) {
			estimator_status_flags_ = *msg;
			estimator_flags_received_ = true;
		});

	failsafe_flags_sub_ = this->create_subscription<px4_msgs::msg::FailsafeFlags>(
		"/fmu/out/failsafe_flags", status_qos,
		[this](const px4_msgs::msg::FailsafeFlags::UniquePtr msg) {
			failsafe_flags_ = *msg;
			failsafe_flags_received_ = true;
		});

	battery_status_sub_ = this->create_subscription<px4_msgs::msg::BatteryStatus>(
		"/fmu/out/battery_status", status_qos,
		[this](const px4_msgs::msg::BatteryStatus::UniquePtr msg) {
			battery_status_ = *msg;
			battery_received_ = true;
		});

	vehicle_command_ack_sub_ = this->create_subscription<px4_msgs::msg::VehicleCommandAck>(
		"/fmu/out/vehicle_command_ack", status_qos,
		[this](const px4_msgs::msg::VehicleCommandAck::UniquePtr msg) {
			vehicle_command_ack_ = *msg;
			command_ack_received_ = true;
		});
}

void PX4Estimator::print_status()
{
	std::cout << "\033[2J\033[H";
	std::cout << "================================================================================\n";
	std::cout << "                            PX4 ESTIMATOR STATUS                               \n";
	std::cout << "================================================================================\n\n";
	
	print_sensor_section();
	print_state_section();
	print_control_section();
	print_health_section();
	
	std::cout << "\nPress Ctrl+C to exit\n";
}

void PX4Estimator::print_sensor_section()
{
	std::cout << "[SENSOR DATA]\n";
	std::cout << "--------------------------------------------------------------------------------\n";
	
	if (sensor_combined_received_) {
		std::cout << "IMU:  Gyro=[" << std::fixed << std::setprecision(3) 
		          << sensor_combined_.gyro_rad[0] << "," << sensor_combined_.gyro_rad[1] 
		          << "," << sensor_combined_.gyro_rad[2] << "] rad/s  "
		          << "Accel=[" << sensor_combined_.accelerometer_m_s2[0] << ","
		          << sensor_combined_.accelerometer_m_s2[1] << "," 
		          << sensor_combined_.accelerometer_m_s2[2] << "] m/s2\n";
	} else {
		std::cout << "IMU:  No data\n";
	}
	
	if (gps_received_) {
		std::cout << "GPS:  Lat=" << std::fixed << std::setprecision(7) 
		          << vehicle_gps_position_.latitude_deg << "  Lon=" 
		          << vehicle_gps_position_.longitude_deg << "  Alt=" 
		          << std::setprecision(2) << vehicle_gps_position_.altitude_msl_m 
		          << "m  Fix=" << static_cast<int>(vehicle_gps_position_.fix_type)
		          << "  Sats=" << static_cast<int>(vehicle_gps_position_.satellites_used)
		          << "  Speed=" << vehicle_gps_position_.vel_m_s << " m/s\n";
	} else {
		std::cout << "GPS:  No data\n";
	}
	
	if (timesync_received_) {
		std::cout << "TimeSync:  Offset=" << timesync_status_.estimated_offset 
		          << " us  RTT=" << timesync_status_.round_trip_time 
		          << " us  Status=" << (timesync_status_.estimated_offset != 0 ? "Synced" : "Not synced") << "\n";
	} else {
		std::cout << "TimeSync:  No data\n";
	}
	
	std::cout << "\n";
}

void PX4Estimator::print_state_section()
{
	std::cout << "[VEHICLE STATE]\n";
	std::cout << "--------------------------------------------------------------------------------\n";
	
	if (attitude_received_) {
		double q0 = vehicle_attitude_.q[0];
		double q1 = vehicle_attitude_.q[1];
		double q2 = vehicle_attitude_.q[2];
		double q3 = vehicle_attitude_.q[3];
		
		double roll = std::atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
		double sin_pitch = 2*(q0*q2 - q3*q1);
		sin_pitch = std::max(-1.0, std::min(1.0, sin_pitch));
		double pitch = std::asin(sin_pitch);
		double yaw = std::atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
		
		std::cout << "Attitude:  Roll=" << std::fixed << std::setprecision(2) 
		          << roll * 180.0 / M_PI << "deg  Pitch=" << pitch * 180.0 / M_PI 
		          << "deg  Yaw=" << yaw * 180.0 / M_PI << "deg\n";
	} else {
		std::cout << "Attitude:  No data\n";
	}
	
	if (local_position_received_) {
		std::cout << "LocalPos:  X=" << std::fixed << std::setprecision(2) 
		          << vehicle_local_position_.x << "m  Y=" << vehicle_local_position_.y 
		          << "m  Z=" << vehicle_local_position_.z << "m  "
		          << "Vx=" << vehicle_local_position_.vx << "m/s  Vy=" 
		          << vehicle_local_position_.vy << "m/s  Vz=" 
		          << vehicle_local_position_.vz << "m/s\n";
	} else {
		std::cout << "LocalPos:  No data\n";
	}
	
	if (global_position_received_) {
		std::cout << "GlobalPos: Lat=" << std::fixed << std::setprecision(7) 
		          << vehicle_global_position_.lat << "  Lon=" 
		          << vehicle_global_position_.lon << "  Alt=" 
		          << std::setprecision(2) << vehicle_global_position_.alt << "m\n";
	} else {
		std::cout << "GlobalPos: No data\n";
	}
	
	if (odometry_received_) {
		std::cout << "Odometry:  Pos=[" << std::fixed << std::setprecision(2)
		          << vehicle_odometry_.position[0] << "," << vehicle_odometry_.position[1] 
		          << "," << vehicle_odometry_.position[2] << "]m  Vel=[" 
		          << vehicle_odometry_.velocity[0] << "," << vehicle_odometry_.velocity[1] 
		          << "," << vehicle_odometry_.velocity[2] << "]m/s\n";
	} else {
		std::cout << "Odometry:  No data\n";
	}
	
	std::cout << "\n";
}

void PX4Estimator::print_control_section()
{
	std::cout << "[CONTROL]\n";
	std::cout << "--------------------------------------------------------------------------------\n";
	
	if (control_mode_received_) {
		std::cout << "ControlMode:  Manual=" << (vehicle_control_mode_.flag_control_manual_enabled ? "ON" : "OFF")
		          << "  Position=" << (vehicle_control_mode_.flag_control_position_enabled ? "ON" : "OFF")
		          << "  Offboard=" << (vehicle_control_mode_.flag_control_offboard_enabled ? "ON" : "OFF")
		          << "  Attitude=" << (vehicle_control_mode_.flag_control_attitude_enabled ? "ON" : "OFF") << "\n";
	} else {
		std::cout << "ControlMode:  No data\n";
	}
	
	if (manual_control_received_) {
		if (manual_control_setpoint_.valid) {
			std::cout << "ManualCtrl:  Throttle=" << std::fixed << std::setprecision(2) 
			          << manual_control_setpoint_.throttle << "  Pitch=" 
			          << manual_control_setpoint_.pitch << "  Roll=" 
			          << manual_control_setpoint_.roll << "  Yaw=" 
			          << manual_control_setpoint_.yaw 
			          << "  Source=" << static_cast<int>(manual_control_setpoint_.data_source) << "\n";
		} else {
			std::cout << "ManualCtrl:  Invalid (no RC/manual input)\n";
		}
	} else {
		std::cout << "ManualCtrl:  No data (topic not published - no RC/manual input in SITL)\n";
	}
	
	if (setpoint_triplet_received_) {
		if (position_setpoint_triplet_.current.valid) {
			std::cout << "Setpoint:  Current Lat=" << std::fixed << std::setprecision(7)
			          << position_setpoint_triplet_.current.lat << "  Lon=" 
			          << position_setpoint_triplet_.current.lon << "  Alt=" 
			          << std::setprecision(2) << position_setpoint_triplet_.current.alt << "m  "
			          << "Vel=[" << position_setpoint_triplet_.current.vx << ","
			          << position_setpoint_triplet_.current.vy << "," 
			          << position_setpoint_triplet_.current.vz << "] m/s\n";
		}
		if (position_setpoint_triplet_.next.valid) {
			std::cout << "Setpoint:  Next Lat=" << std::fixed << std::setprecision(7)
			          << position_setpoint_triplet_.next.lat << "  Lon=" 
			          << position_setpoint_triplet_.next.lon << "  Alt=" 
			          << std::setprecision(2) << position_setpoint_triplet_.next.alt << "m\n";
		}
		if (!position_setpoint_triplet_.current.valid && !position_setpoint_triplet_.next.valid) {
			std::cout << "Setpoint:  No valid setpoint\n";
		}
	} else {
		std::cout << "Setpoint:  No data\n";
	}
	
	std::cout << "\n";
}

void PX4Estimator::print_health_section()
{
	std::cout << "[HEALTH STATUS]\n";
	std::cout << "--------------------------------------------------------------------------------\n";
	
	if (status_received_) {
		std::string arm_status;
		if (vehicle_status_.arming_state == 2) {  // ARMING_STATE_ARMED
			arm_status = "ARMED/UNLOCKED";
		} else if (vehicle_status_.arming_state == 1) {  // ARMING_STATE_DISARMED
			arm_status = "DISARMED/LOCKED";
		} else {
			arm_status = "UNKNOWN";
		}
		
		std::cout << "VehicleStatus:  ArmingState=" << arm_status
		          << "  NavState=" << static_cast<int>(vehicle_status_.nav_state)
		          << "  SystemID=" << static_cast<int>(vehicle_status_.system_id)
		          << "  ComponentID=" << static_cast<int>(vehicle_status_.component_id) << "\n";
	} else {
		std::cout << "VehicleStatus:  No data\n";
	}
	
	if (battery_received_) {
		std::cout << "Battery:  Voltage=" << std::fixed << std::setprecision(2) 
		          << battery_status_.voltage_v << "V  Current=" << battery_status_.current_a 
		          << "A  Remaining=" << std::setprecision(1) 
		          << battery_status_.remaining * 100.0 << "%\n";
	} else {
		std::cout << "Battery:  No data\n";
	}
	
	if (estimator_flags_received_) {
		std::cout << "Estimator:  GPS=" << (estimator_status_flags_.cs_gps ? "OK" : "FAIL")
		          << "  TiltAlign=" << (estimator_status_flags_.cs_tilt_align ? "OK" : "FAIL")
		          << "  YawAlign=" << (estimator_status_flags_.cs_yaw_align ? "OK" : "FAIL")
		          << "  BaroHgt=" << (estimator_status_flags_.cs_baro_hgt ? "OK" : "FAIL")
		          << "  InAir=" << (estimator_status_flags_.cs_in_air ? "YES" : "NO") << "\n";
	} else {
		std::cout << "Estimator:  No data\n";
	}
	
	if (failsafe_flags_received_) {
		std::cout << "Failsafe:  GlobalPos=" << (failsafe_flags_.global_position_invalid ? "INVALID" : "OK")
		          << "  LocalPos=" << (failsafe_flags_.local_position_invalid ? "INVALID" : "OK")
		          << "  Attitude=" << (failsafe_flags_.attitude_invalid ? "INVALID" : "OK")
		          << "  Battery=" << (failsafe_flags_.battery_unhealthy ? "UNHEALTHY" : "OK")
		          << "  BatteryWarn=" << static_cast<int>(failsafe_flags_.battery_warning)
		          << "  ManualCtrl=" << (failsafe_flags_.manual_control_signal_lost ? "LOST" : "OK")
		          << "  GCS=" << (failsafe_flags_.gcs_connection_lost ? "LOST" : "OK") << "\n";
	} else {
		std::cout << "Failsafe:  No data\n";
	}
	
	std::cout << "\n";
}

void PX4Estimator::publish_state()
{
	// Skip publishing if no essential data received
	if (!status_received_ && !attitude_received_ && !local_position_received_) {
		return;
	}
	
	json j;
	
	// Vehicle status
	if (status_received_) {
		j["armed"] = (vehicle_status_.arming_state == 2);
		j["arming_state"] = static_cast<int>(vehicle_status_.arming_state);
		j["nav_state"] = static_cast<int>(vehicle_status_.nav_state);
		j["system_id"] = static_cast<int>(vehicle_status_.system_id);
		j["component_id"] = static_cast<int>(vehicle_status_.component_id);
	}
	
	// Battery status
	if (battery_received_) {
		j["battery_v"] = battery_status_.voltage_v;
		j["battery_b"] = battery_status_.remaining * 100.0;
		j["battery_current"] = battery_status_.current_a;
	}
	
	// Position (local)
	if (local_position_received_) {
		j["position"] = {vehicle_local_position_.x, 
		                  vehicle_local_position_.y, 
		                  vehicle_local_position_.z};
		j["velocity"] = {vehicle_local_position_.vx, 
		                 vehicle_local_position_.vy, 
		                 vehicle_local_position_.vz};
	}
	
	// Global position (GPS)
	if (global_position_received_) {
		j["latitude"] = vehicle_global_position_.lat;
		j["longitude"] = vehicle_global_position_.lon;
		j["altitude"] = vehicle_global_position_.alt;
	} else if (gps_received_) {
		j["latitude"] = vehicle_gps_position_.latitude_deg;
		j["longitude"] = vehicle_gps_position_.longitude_deg;
		j["altitude"] = vehicle_gps_position_.altitude_msl_m;
	}
	
	// Attitude (Euler angles in degrees)
	if (attitude_received_) {
		double q0 = vehicle_attitude_.q[0];
		double q1 = vehicle_attitude_.q[1];
		double q2 = vehicle_attitude_.q[2];
		double q3 = vehicle_attitude_.q[3];
		
		double roll = std::atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
		double sin_pitch = 2*(q0*q2 - q3*q1);
		sin_pitch = std::max(-1.0, std::min(1.0, sin_pitch));
		double pitch = std::asin(sin_pitch);
		double yaw = std::atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
		
		j["attitude_rpy"] = {roll * 180.0 / M_PI, 
		                     pitch * 180.0 / M_PI, 
		                     yaw * 180.0 / M_PI};
		j["attitude"] = {roll, pitch, yaw};
	}
	
	// GPS status
	if (gps_received_) {
		j["gps_status"] = static_cast<int>(vehicle_gps_position_.fix_type);
		j["gps_satellites"] = static_cast<int>(vehicle_gps_position_.satellites_used);
		j["gps_speed"] = vehicle_gps_position_.vel_m_s;
	}
	
	// Control mode
	if (control_mode_received_) {
		j["control_mode"] = {
			{"manual", vehicle_control_mode_.flag_control_manual_enabled},
			{"position", vehicle_control_mode_.flag_control_position_enabled},
			{"offboard", vehicle_control_mode_.flag_control_offboard_enabled},
			{"attitude", vehicle_control_mode_.flag_control_attitude_enabled}
		};
	}
	
	// Estimator status
	if (estimator_flags_received_) {
		j["estimator"] = {
			{"gps_ok", estimator_status_flags_.cs_gps},
			{"tilt_align", estimator_status_flags_.cs_tilt_align},
			{"yaw_align", estimator_status_flags_.cs_yaw_align},
			{"baro_hgt", estimator_status_flags_.cs_baro_hgt},
			{"in_air", estimator_status_flags_.cs_in_air}
		};
	}
	
	// Failsafe flags
	if (failsafe_flags_received_) {
		j["failsafe"] = {
			{"global_pos_invalid", failsafe_flags_.global_position_invalid},
			{"local_pos_invalid", failsafe_flags_.local_position_invalid},
			{"attitude_invalid", failsafe_flags_.attitude_invalid},
			{"battery_unhealthy", failsafe_flags_.battery_unhealthy},
			{"battery_warning", static_cast<int>(failsafe_flags_.battery_warning)},
			{"manual_ctrl_lost", failsafe_flags_.manual_control_signal_lost},
			{"gcs_connection_lost", failsafe_flags_.gcs_connection_lost}
		};
	}
	
	// IMU data
	if (sensor_combined_received_) {
		j["imu"] = {
			{"gyro", {sensor_combined_.gyro_rad[0], 
			          sensor_combined_.gyro_rad[1], 
			          sensor_combined_.gyro_rad[2]}},
			{"accel", {sensor_combined_.accelerometer_m_s2[0], 
			           sensor_combined_.accelerometer_m_s2[1], 
			           sensor_combined_.accelerometer_m_s2[2]}}
		};
	}
	
	// Timestamp
	j["timestamp"] = this->now().nanoseconds();
	
	// Publish as string message
	auto msg = std_msgs::msg::String();
	msg.data = j.dump();
	state_pub_->publish(msg);
}

