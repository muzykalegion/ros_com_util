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

		subscription_ =
			this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position",
				10,
				[this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
					X = msg->x;
					Y = msg->y;
					float Z = msg->z;
					if(!start_trj && (p0_x + 1.0 > X && p0_x - 1.0 < X)&&(p0_y + 1.0 > Y && p0_y - 1.0 < Y)&&(p0_z + 1.0 > Z && p0_z - 1.0 < Z)){
						start_trj = true;
						std::cout << "start trj!" << std::endl;
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
			// the spiral, in polar coordinates (theta, rho), is given by
			// theta = theta_0 + omega*t
			// rho = rho_0 + K*theta
			float theta = theta_0 + omega * 0.1 * discrete_time_index;
			float rho = rho_0 + K * theta;
			
			// from polar to cartesian coordinates
			des_x = rho * cos(theta);
			des_y = rho * sin(theta);

			// velocity computation
			float dot_rho = K*omega;
			dot_des_x = dot_rho*cos(theta) - rho*sin(theta)*omega;
			dot_des_y = dot_rho*sin(theta) + rho*cos(theta)*omega;
			// desired heading direction
			gamma = atan2(dot_des_y, dot_des_x);

        	// offboard_control_mode needs to be paired with trajectory_setpoint
		    publish_offboard_control_mode();
		    publish_trajectory_setpoint();

       		     // stop the counter after reaching 11
		    if (offboard_setpoint_counter_ < 11) {
			    offboard_setpoint_counter_++;
		    }
			if (start_trj){
				discrete_time_index++;
			}
	    };
	    command_timer_ = this->create_wall_timer(100ms, send_commands_callback);
	}

	void arm();
	void disarm();

private:

	bool start_trj = true;

	const float omega = 0.3; 	// angular speed of the POLAR trajectory
	const float K = 2;			// [m] gain that regulates the spiral pitch

	
	const float rho_0 = 2;
	const float theta_0 = 0;
	const float p0_z = -10.0;
	float p0_x = rho_0*cos(theta_0);
	float p0_y = rho_0*sin(theta_0);
	float des_x = p0_x, des_y = p0_y, des_z = p0_z;
	float dot_des_x = 0.0, dot_des_y = 0.0;
	float gamma = M_PI_4;

	float X;
	float Y;
    
	uint32_t discrete_time_index = 0;

	rclcpp::TimerBase::SharedPtr command_timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;
	//
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_ = 0;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
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
    msg.position = {des_x, des_y, des_z};
    msg.velocity = {dot_des_x, dot_des_y, 0.0};
    msg.yaw = gamma;		//-3.14; // [-PI:PI]
    trajectory_setpoint_publisher_->publish(msg);

	RCLCPP_INFO(this->get_logger(), "Next waypoint: %.2f %.2f %.2f",des_x, des_y, des_z);
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

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardCalibration>());

	rclcpp::shutdown();
	return 0;
}