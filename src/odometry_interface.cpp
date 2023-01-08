#include "odometry_interface.hpp"
#include "motor_interface.hpp"
#include "swerve_drivetrain_node.hpp"
#include <ck_utilities/geometry/geometry.hpp>
#include <ck_utilities/CKMath.hpp>
#include <nav_msgs/Odometry.h>

void publishOdometryData()
{
	geometry::Translation wheel_vel_sum;
	for (int i = 0; i < robot_num_wheels; i++)
	{
		double curr_vel_m_s = motor_map[drive_motor_ids[i]].sensor_velocity * M_PI * ck::math::inches_to_meters(wheel_diameter_inches) / 60.0;
		double curr_angle = ck::math::normalize_to_2_pi(ck::math::deg2rad(motor_map[steering_motor_ids[i]].sensor_position * 360.0));
		geometry::Translation wheel_vel;
		wheel_vel.x(curr_vel_m_s * std::cos(curr_angle));
		wheel_vel.y(curr_vel_m_s * std::sin(curr_angle));
		wheel_vel_sum += wheel_vel;
	}
	wheel_vel_sum /= robot_num_wheels;

	nav_msgs::Odometry odometry_data;
    odometry_data.header.stamp = ros::Time::now();
	odometry_data.header.frame_id = "odom";
	odometry_data.child_frame_id = "base_link";

	odometry_data.pose.pose.orientation.w = 0;
	odometry_data.pose.pose.orientation.x = 0;
	odometry_data.pose.pose.orientation.y = 0;
	odometry_data.pose.pose.orientation.z = 0;
	odometry_data.pose.pose.position.x = 0;
	odometry_data.pose.pose.position.y = 0;
	odometry_data.pose.pose.position.z = 0;

	odometry_data.twist.twist.linear.x = wheel_vel_sum.x();
	odometry_data.twist.twist.linear.y = wheel_vel_sum.y();
	odometry_data.twist.twist.linear.z = 0;

	odometry_data.twist.twist.angular.x = 0;
	odometry_data.twist.twist.angular.y = 0;
	odometry_data.twist.twist.angular.z = 0;

	odometry_data.pose.covariance =
	   { 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0,
		 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0,
		 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0,
		 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0,
		 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0,
		 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001,};

	odometry_data.twist.covariance =
	   { 0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
		 0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
		 0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
		 0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
		 0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
		 0.0, 0.0, 0.0, 0.0, 0.0, 0.001,};

	static ros::Publisher odometry_publisher = node->advertise<nav_msgs::Odometry>("/RobotOdometry", 100);
	odometry_publisher.publish(odometry_data);
}