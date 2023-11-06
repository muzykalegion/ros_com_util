/****************************************************************************
 *
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 *           2018 PX4 Pro Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Sensor Combined uORB topic listener remaping IMU data to `/fmu/out/imu` topic
 * @file sensor_combined_imu_publisher.cpp
 * @addtogroup utility
 * @author Muzyka Legion <muzyka.legion@gmail.com>
 */

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <sensor_msgs/msg/imu.hpp>

/**
 * @brief Sensor Combined uORB topic data callback with Imu data publisher
 */
class SensorCombinedImuListener : public rclcpp::Node
{
public:
	explicit SensorCombinedImuListener() : Node("sensor_combined_imu_listener")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/fmu/out/imu", 10);
		
		subscription_ = this->create_subscription<px4_msgs::msg::SensorCombined>("/fmu/out/sensor_combined", qos,
		[this](const px4_msgs::msg::SensorCombined::UniquePtr msg) {
			sensor_msgs::msg::Imu imu_msg;
			imu_msg.header.stamp.sec      = msg->timestamp / 1000000;
			imu_msg.header.stamp.nanosec  = (msg->timestamp % 1000000) * 1000;
			imu_msg.header.frame_id       = "/pixhawk/imu";
			imu_msg.linear_acceleration.x = msg->accelerometer_m_s2[0];
			imu_msg.linear_acceleration.y = msg->accelerometer_m_s2[1];
			imu_msg.linear_acceleration.z = msg->accelerometer_m_s2[2];
			imu_msg.angular_velocity.x    = msg->gyro_rad[0];
			imu_msg.angular_velocity.y    = msg->gyro_rad[1];
			imu_msg.angular_velocity.z    = msg->gyro_rad[2];

			this->publisher_->publish(imu_msg);
		});
	}

private:
	rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr subscription_;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;

};

int main(int argc, char *argv[])
{
	std::cout << "Starting sensor_combined_imu listener node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SensorCombinedImuListener>());

	rclcpp::shutdown();
	return 0;
}
