/**
 * @brief Sensor Remapper uORB topic listener remaps OMSKF Pose topics
 * @file pose_imu_remapper.cpp
 * @addtogroup utility
 * @author Muzyka Legion <muzyka.legion@gmail.com>
 */

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <ros_com_util/frame_transforms.h>

using namespace px4_ros_com::frame_transforms;

/**
 * @brief Sensor Remapper uORB topic data callback with OMSKF Pose data 
 */
class PoseImuRemapper : public rclcpp::Node
{
public:
	explicit PoseImuRemapper() : Node("pose_imu_remapper")
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		vis_odom_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("fmu/in/vehicle_visual_odometry", 10);

		pose_imu_sub_ =	this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
			"/ov_msckf/poseimu",
			qos,
			[this](const geometry_msgs::msg::PoseWithCovarianceStamped::UniquePtr& msg) {
				// tf2::Quaternion q(
				// 	msg->pose.pose.orientation.x,
				// 	msg->pose.pose.orientation.y,
				// 	msg->pose.pose.orientation.z,
				// 	msg->pose.pose.orientation.w);
				// tf2::Matrix3x3 m(q);
				// double roll, pitch, yaw;
				// m.getRPY(roll, pitch, yaw);
				
				//float Z = msg->pose.pose.orientation.z;

				// des_x = msg->pose.pose.position.x;
				// des_y = msg->pose.pose.position.y;
				// des_z = -10.0 + msg->pose.pose.position.z;
				// gamma = -yaw;

				px4_msgs::msg::VehicleOdometry px4_odom_msg;
				// It seems that time is updated automatically on PX4 side
				// after v1.14
				//px4_odom_msg.timestamp = 0; //static_cast<uint64_t>(msg.header.stamp.sec*1e6) + static_cast<uint64_t>(msg.header.stamp.nanosec/1e3);
				px4_odom_msg.timestamp = static_cast<uint64_t>(msg->header.stamp.sec) * 1000000 + static_cast<uint64_t>(msg->header.stamp.nanosec) / 1000;
				px4_odom_msg.timestamp_sample = 0;

				px4_odom_msg.pose_frame = px4_odom_msg.POSE_FRAME_FRD;
				px4_odom_msg.velocity_frame = px4_odom_msg.VELOCITY_FRAME_FRD;

				// position
				Eigen::Vector3d position_enu(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
				Eigen::Vector3d position_ned = enu_to_ned_local_frame(position_enu);
				// Velocity
				// Eigen::Vector3d velocity_enu(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
				// Eigen::Vector3d velocity_ned = enu_to_ned_local_frame(velocity_enu);
				// Orientation
				Eigen::Quaterniond q_enu(msg->pose.pose.orientation.w,
										msg->pose.pose.orientation.x,
										msg->pose.pose.orientation.y,
										msg->pose.pose.orientation.z);
				Eigen::Quaterniond q_ned = ros_to_px4_orientation(q_enu);

				// Angular Velocity
				// Eigen::Vector3d angular_vel_ros(msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg->twist.twist.angular.z);
				// Eigen::Vector3d angular_vel_px4 = baselink_to_aircraft_body_frame(angular_vel_ros);
				

				// Pose covariance
				Covariance3d cov_pos_ros = {
					msg->pose.covariance[0], msg->pose.covariance[1], msg->pose.covariance[2],
					msg->pose.covariance[6], msg->pose.covariance[7], msg->pose.covariance[8],
					msg->pose.covariance[12], msg->pose.covariance[13], msg->pose.covariance[14]
				};

				Covariance3d cov_rot_ros = {
					msg->pose.covariance[21], msg->pose.covariance[22], msg->pose.covariance[23],
					msg->pose.covariance[27], msg->pose.covariance[28], msg->pose.covariance[29],
					msg->pose.covariance[33], msg->pose.covariance[34], msg->pose.covariance[35]
				};

				// Velocity covariance

				// Covariance3d cov_vel_ros = {
				// 	msg.twist.covariance[0], msg.twist.covariance[1], msg.twist.covariance[2],
				// 	msg.twist.covariance[6], msg.twist.covariance[7], msg.twist.covariance[8],
				// 	msg.twist.covariance[12], msg.twist.covariance[13], msg.twist.covariance[14]
				// };

				Covariance3d cov_pos_px4 = transform_static_frame(cov_pos_ros, StaticTF::ENU_TO_NED);
				// Covariance3d cov_vel_px4 = transform_static_frame(cov_vel_ros, StaticTF::ENU_TO_NED);
				Covariance3d cov_rot_px4 = transform_frame(cov_rot_ros, q_ned);

				px4_odom_msg.position[0] = position_ned.x();
				px4_odom_msg.position[1] = position_ned.y();
				px4_odom_msg.position[2] = position_ned.z();

				// px4_odom_msg.velocity[0] = velocity_ned.x();
				// px4_odom_msg.velocity[1] = velocity_ned.y();
				// px4_odom_msg.velocity[2] = velocity_ned.z();
				
				px4_odom_msg.q[0] = q_ned.w();
				px4_odom_msg.q[1] = q_ned.x();
				px4_odom_msg.q[2] = q_ned.y();
				px4_odom_msg.q[3] = q_ned.z();

				// px4_odom_msg.angular_velocity[0] = angular_vel_px4.x();
				// px4_odom_msg.angular_velocity[1] = angular_vel_px4.y();
				// px4_odom_msg.angular_velocity[2] = angular_vel_px4.z();

				px4_odom_msg.position_variance[0] = cov_pos_px4[0]; // diagonal elements
				px4_odom_msg.position_variance[1] = cov_pos_px4[4];
				px4_odom_msg.position_variance[2] = cov_pos_px4[8];

				// px4_odom_msg.velocity_variance[0] = cov_vel_px4[0];
				// px4_odom_msg.velocity_variance[1] = cov_vel_px4[4];
				// px4_odom_msg.velocity_variance[2] = cov_vel_px4[8];

				px4_odom_msg.orientation_variance[0] = cov_rot_px4[0];
				px4_odom_msg.orientation_variance[1] = cov_rot_px4[4];
				px4_odom_msg.orientation_variance[2] = cov_rot_px4[8];
				
				vis_odom_pub_->publish(px4_odom_msg);
			}
		);
	}

private:
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_imu_sub_;
	rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr vis_odom_pub_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting pose IMU remapping node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PoseImuRemapper>());

	rclcpp::shutdown();
	return 0;
}
