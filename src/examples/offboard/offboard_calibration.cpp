/**
 * @brief Offboard calibration utility for Kalibr
 * @file offboard_control.cpp
 * @addtogroup utility
 * @author Muzyka Legion <muzyka.legion@gmail.com>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardCalibration : public rclcpp::Node
{
public:
	OffboardCalibration() : Node("offboard_calibration")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		// get common timestamp
	    timesync_sub_ =
		    this->create_subscription<px4_msgs::msg::TimesyncStatus>("/fmu/out/timesync_status", 
			qos,
			[this](const px4_msgs::msg::TimesyncStatus::UniquePtr msg) {
				timestamp_.store(msg->timestamp);
			});

		offboard_setpoint_counter_ = 0;
	
		//this->publish_gps_global_origin(50.123456, 30.123456, 1.0);

		subscription_ =
			this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position",
				qos,
				[this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
					RCLCPP_INFO(this->get_logger(), "Local pos: %.2f %.2f %.2f", msg->x, msg->y, msg->z);

					if((waypoints[wpIndex][0] + threshold > msg->x && waypoints[wpIndex][0] - threshold < msg->x)
					&& (waypoints[wpIndex][1] + threshold > msg->y && waypoints[wpIndex][1] - threshold < msg->y)) {

						wpIndex++;

						if (wpIndex >= waypoints.size()) {
							exit(0);
						}
						RCLCPP_INFO(this->get_logger(), "Next waypoint: %.2f %.2f %.2f. Index: %d", 
						waypoints[wpIndex][0], waypoints[wpIndex][1], waypoints[wpIndex][2], wpIndex);
					}
				}
			);

		auto send_commands_callback = [this]() -> void {
		    if (offboard_setpoint_counter_ == 10) {
			    // Change to Offboard mode after 10 setpoints
			    this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				
			    // Arm the vehicle
			    this->arm();
		    }

			publish_offboard_control_mode();
			publish_trajectory_setpoint();

			// stop the counter after reaching 10
		    if (offboard_setpoint_counter_ < 11) {
			    offboard_setpoint_counter_++;
		    }
			
	    };
	    command_timer_ = this->create_wall_timer(100ms, send_commands_callback);
	}

	void arm();
	void disarm();

private:

	uint8_t wpIndex = 0;

	float threshold = 0.09;

	std::vector<std::vector<float>> waypoints = {
		// Forward-backward x3
		{0.0, 0.0, -1.5,},
		{0.5, 0.0, -1.5,},
		{0.0, 0.0, -1.5,},
		{-0.5, 0.0, -1.5,},
		{0.0, 0.0, -1.5,},
		{0.0, 0.0, -1.5,},
		{0.5, 0.0, -1.5,},
		{0.0, 0.0, -1.5,},
		{-0.5, 0.0, -1.5,},
		{0.0, 0.0, -1.5,},
		{0.0, 0.0, -1.5,},
		{0.5, 0.0, -1.5,},
		{0.0, 0.0, -1.5,},
		{-0.5, 0.0, -1.5,},
		{0.0, 0.0, -1.5,},
		// Left-right x3
		{0.0, 0.0, -1.5,},
		{0.0, 0.5, -1.5,},
		{0.0, 0.0, -1.5,},
		{0.0, -0.5, -1.5,},
		{0.0, 0.0, -1.5,},
		{0.0, 0.0, -1.5,},
		{0.0, 0.5, -1.5,},
		{0.0, 0.0, -1.5,},
		{0.0, -0.5, -1.5,},
		{0.0, 0.0, -1.5,},
		{0.0, 0.0, -1.5,},
		{0.0, 0.5, -1.5,},
		{0.0, 0.0, -1.5,},
		{0.0, -0.5, -1.5,},
		{0.0, 0.0, -1.5,},
		// Land
		{0.0, 0.0, 0.0}
	};

	rclcpp::TimerBase::SharedPtr command_timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;
	//
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void publish_gps_global_origin(float lat = 0.0, float lon = 0.0, float alt = 0.0);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardCalibration::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardCalibration::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardCalibration::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = timestamp_.load();
	offboard_control_mode_publisher_->publish(msg);
}

void OffboardCalibration::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};
    msg.timestamp = timestamp_.load();
	msg.position = {waypoints[wpIndex][0], waypoints[wpIndex][1], waypoints[wpIndex][2]};
    msg.velocity = {0.05, 0.05, 0.5};
    msg.yaw = 0;		//-3.14; // [-PI:PI]
    trajectory_setpoint_publisher_->publish(msg);

	//RCLCPP_INFO(this->get_logger(), "Next waypoint: %.2f %.2f %.2f", waypoints[wpIndex][0], waypoints[wpIndex][1], waypoints[wpIndex][2]);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardCalibration::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = timestamp_.load();
	vehicle_command_publisher_->publish(msg);
}

/**
 * @brief Publish SET_GPS_GLOBAL_ORIGIN command
 * @param lat    Latitude
 * @param lon    Longitude
 * @param alt    Altitude
 */
void OffboardCalibration::publish_gps_global_origin(float lat, float lon, float alt)
{
	VehicleCommand msg{};
	msg.param5 = lat;
	msg.param6 = lon;
	msg.param7 = alt;
	msg.command = VehicleCommand::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = timestamp_.load();
	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardCalibration>());

	rclcpp::shutdown();
	return 0;
}