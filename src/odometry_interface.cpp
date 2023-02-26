#include <ros/ros.h>
#include "odometry_interface.hpp"
#include "motor_interface.hpp"
#include "swerve_drivetrain_node.hpp"
#include <ck_utilities/geometry/geometry.hpp>
#include <ck_utilities/CKMath.hpp>
#include <nav_msgs/Odometry.h>
#include "config_params.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ck_utilities/geometry/geometry.hpp>
#include <ck_utilities/geometry/geometry_ros_helpers.hpp>
#include <nav_msgs/Odometry.h>
#include <ck_ros_msgs_node/Swerve_Drivetrain_Diagnostics.h>
#include <ck_utilities/Logger.hpp>
#include <deque>


tf2_ros::TransformBroadcaster * tfBroadcaster;
static geometry::Transform robot_transform;

void robot_odometry_subscriber(const nav_msgs::Odometry &odom)
{
	geometry::Twist drivetrain_twist = geometry::to_twist(odom.twist.twist);
	robot_transform.angular = geometry::to_rotation(odom.pose.pose.orientation);
	robot_transform.linear = geometry::to_translation(odom.pose.pose.position);

    geometry::Rotation robot_to_field_rot = robot_transform.angular;
    robot_to_field_rot.yaw(robot_to_field_rot.yaw());
    geometry::Twist field_twist = drivetrain_twist.rotate(robot_to_field_rot);
	drivetrain_diagnostics.field_actual_x_translation_m_s = field_twist.linear.x();
	drivetrain_diagnostics.field_actual_y_translation_m_s = field_twist.linear.y();
	drivetrain_diagnostics.actual_total_speed_m_s = drivetrain_twist.linear.norm();

    float hypotenuse = std::sqrt(drivetrain_twist.linear.x() * drivetrain_twist.linear.x() + drivetrain_twist.linear.y() * drivetrain_twist.linear.y());
    float angle = ck::math::rad2deg(std::asin(drivetrain_twist.linear.y() / hypotenuse));
    drivetrain_diagnostics.actual_track = angle;
}

void raw_gyro_subscriber(const nav_msgs::Odometry &odom)
{
	geometry::Twist drivetrain_twist = geometry::to_twist(odom.twist.twist);
    static std::deque<float> angular_rates(3);
    while (angular_rates.size() < 3)
    {
        angular_rates.push_back(0);
    }
    angular_rates.pop_front();
    angular_rates.push_back(drivetrain_twist.angular.yaw());

    float value;
    for ( auto &i : angular_rates)
    {
        value += i;
    }
    value /= angular_rates.size();

	drivetrain_diagnostics.actual_angular_speed_deg_s = value;
}

void tf2_init()
{
	static bool init_complete = false;
	if(!init_complete)
	{
		tfBroadcaster = new tf2_ros::TransformBroadcaster();
		init_complete = true;
		static ros::Subscriber odometry_subscriber = node->subscribe("/odometry/filtered", 10, robot_odometry_subscriber, ros::TransportHints().tcpNoDelay());
		static ros::Subscriber raw_gyro_sub = node->subscribe("/RobotIMU", 10, raw_gyro_subscriber, ros::TransportHints().tcpNoDelay());
	}
}

geometry::Transform get_robot_transform()
{
	return robot_transform;
}

void publishOdometryData()
{
	tf2_init();
	geometry::Translation wheel_vel_sum;
	for (int i = 0; i < config_params::robot_num_wheels; i++)
	{
		double curr_vel_m_s = motor_map[config_params::drive_motor_ids[i]].sensor_velocity * M_PI * ck::math::inches_to_meters(config_params::wheel_diameter_inches) / 60.0;
		double curr_angle = ck::math::normalize_to_2_pi(ck::math::deg2rad(motor_map[config_params::steering_motor_ids[i]].sensor_position * 360.0));
		geometry::Translation wheel_vel;
		wheel_vel.x(curr_vel_m_s * std::cos(curr_angle));
		wheel_vel.y(curr_vel_m_s * std::sin(curr_angle));
		wheel_vel_sum += wheel_vel;
	}
	wheel_vel_sum /= config_params::robot_num_wheels;

	nav_msgs::Odometry odometry_data;
    odometry_data.header.stamp = ros::Time::now();
	odometry_data.header.frame_id = "odom";
	odometry_data.child_frame_id = "base_link";

    geometry::Pose blank_pose;
	odometry_data.pose.pose = geometry::to_msg(blank_pose);

    geometry::Twist linear_twist;
    linear_twist.linear.x(wheel_vel_sum.x());
    linear_twist.linear.y(wheel_vel_sum.y());
    odometry_data.twist.twist = geometry::to_msg(linear_twist);

    geometry::Covariance pose_covariance;
	odometry_data.pose.covariance = geometry::to_msg(pose_covariance);

    geometry::Covariance twist_covariance;
    twist_covariance.x_var(.0001);
    twist_covariance.y_var(.0001);
	odometry_data.twist.covariance = geometry::to_msg(twist_covariance);

	static ros::Publisher odometry_publisher = node->advertise<nav_msgs::Odometry>("/RobotOdometry", 100);
	odometry_publisher.publish(odometry_data);

	drivetrain_diagnostics.body_actual_x_translation_m_s = wheel_vel_sum.x();
	drivetrain_diagnostics.body_actual_y_translation_m_s = wheel_vel_sum.y();
}

void publish_robot_base_tf()
{
    geometry_msgs::TransformStamped stamped_base_link;
    stamped_base_link.header.frame_id = "odom";
    stamped_base_link.header.stamp = ros::Time::now();
    stamped_base_link.child_frame_id = "base_link";
    stamped_base_link.transform = geometry::to_msg(robot_transform);
    tfBroadcaster->sendTransform(stamped_base_link);
}

void publish_motor_links()
{
	tf2_init();
	//Update swerve steering transforms
	for (int i = 0; i < config_params::robot_num_wheels; i++)
	{
        geometry::Transform swerve_transform = wheel_transforms[i];
        swerve_transform.angular.yaw(ck::math::normalize_to_2_pi(ck::math::deg2rad(motor_map[(uint16_t)config_params::steering_motor_ids[i]].sensor_position * 360.0)));
		geometry_msgs::TransformStamped transform;
		transform.header.frame_id = "base_link";
		transform.header.stamp = ros::Time::now();
		std::stringstream s;
		s << "swerve_" << i;
		transform.child_frame_id = s.str().c_str();
		transform.transform = geometry::to_msg(swerve_transform);

		tfBroadcaster->sendTransform(transform);
	}
}