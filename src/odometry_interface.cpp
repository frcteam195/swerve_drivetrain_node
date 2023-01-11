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


tf2_ros::TransformBroadcaster * tfBroadcaster;
tf2_ros::TransformListener *tfListener;
tf2_ros::Buffer tfBuffer;
extern ck_ros_msgs_node::Swerve_Drivetrain_Diagnostics drivetrain_diagnostics;

void robot_odometry_subscriber(const nav_msgs::Odometry &odom)
{

	geometry::Twist drivetrain_twist = geometry::to_twist(odom.twist.twist);
	drivetrain_diagnostics.field_actual_x_translation_m_s = drivetrain_twist.linear.x();
	drivetrain_diagnostics.field_actual_y_translation_m_s = drivetrain_twist.linear.y();
	drivetrain_diagnostics.actual_angular_speed_deg_s = ck::math::rad2deg(drivetrain_twist.angular.yaw());
	drivetrain_diagnostics.actual_total_speed_m_s = drivetrain_twist.linear.norm();
}

void tf2_init()
{
	static bool init_complete = false;
	if(!init_complete)
	{
		tfBroadcaster = new tf2_ros::TransformBroadcaster();
		tfListener = new tf2_ros::TransformListener(tfBuffer);
		init_complete = true;
		static ros::Subscriber odometry_subscriber = node->subscribe("/odometry/filtered", 10, robot_odometry_subscriber, ros::TransportHints().tcpNoDelay());
	}
}

// void update_drivetrain_diagnostics_position()
// {
// 	// geometry::Transform robot_transform = get_robot_transform();


// }

geometry::Transform get_robot_transform()
{
	tf2_init();
    tf2::Stamped<tf2::Transform> robot_base_to_hub;
	geometry::Transform transform;

    try
    {
        tf2::convert(tfBuffer.lookupTransform("map", "base_link", ros::Time(0)), robot_base_to_hub);
		tf2::Transform tf_transform(robot_base_to_hub);
		transform = geometry::to_transform(tf2::toMsg(tf_transform));
    }

    catch (...)
    {
        static ros::Time prevPubTime(0);
        if (ros::Time::now() - prevPubTime > ros::Duration(1))
        {
            ROS_WARN("Robot Position Lookup Failed");
            prevPubTime = ros::Time::now();
        }
    }

    return transform;
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

	drivetrain_diagnostics.body_actual_x_translation_m_s = wheel_vel_sum.x();
	drivetrain_diagnostics.body_actual_y_translation_m_s = wheel_vel_sum.y();
}


void publish_motor_links()
{
	tf2_init();
	//Update swerve steering transforms
	for (int i = 0; i < config_params::robot_num_wheels; i++)
	{
		tf2::Quaternion quat_tf;
		quat_tf.setRPY(0, 0, ck::math::normalize_to_2_pi(ck::math::deg2rad(motor_map[(uint16_t)config_params::steering_motor_ids[i]].sensor_position * 360.0)));
		geometry_msgs::TransformStamped transform;
		transform.header.frame_id = "base_link";
		transform.header.stamp = ros::Time::now() + ros::Duration(5);
		std::stringstream s;
		s << "swerve_" << i;
		transform.child_frame_id = s.str().c_str();
		transform.transform = geometry::to_msg(wheel_transforms[i]);
		transform.transform.rotation = tf2::toMsg(quat_tf);

		tfBroadcaster->sendTransform(transform);
	}
}