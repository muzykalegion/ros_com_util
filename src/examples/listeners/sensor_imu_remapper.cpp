/**
 * @brief Sensor Remapper uORB topic listener remaps IMU Pose topics
 * @file sensor_remapper.cpp
 * @addtogroup utility
 * @author Muzyka Legion <muzyka.legion@gmail.com>
 */

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <sensor_msgs/msg/imu.hpp>

/**
 * @brief Sensor Remapper uORB topic data callback with IMU data 
 */
class SensorImuRemapper : public rclcpp::Node
{
public:
	explicit SensorImuRemapper() : Node("sensor_imu_remapper")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/fmu/out/imu", 10);
		
		sensor_combined_sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>("/fmu/out/sensor_combined", 
			qos,
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

				this->imu_pub_->publish(imu_msg);
		});
	}

private:
	rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sensor_combined_sub_;
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting sensor IMU remapping node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SensorImuRemapper>());

	rclcpp::shutdown();
	return 0;
}
