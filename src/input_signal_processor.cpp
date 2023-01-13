#include "input_signal_processor.hpp"
#include "swerve_drivetrain_node.hpp"
#include "odometry_interface.hpp"
#include "config_params.hpp"
#include "ck_utilities/MovingAverage.hpp"
#include <ck_ros_msgs_node/Swerve_Drivetrain_Diagnostics.h>
#include <ck_ros_msgs_node/Swerve_Drivetrain_Auto_Control.h>
#include <ck_utilities/CKMath.hpp>
#include <ck_utilities/geometry/geometry_ros_helpers.hpp>
#include "swerve_drive_helper.hpp"
#include <algorithm>

extern ck_ros_msgs_node::Swerve_Drivetrain_Diagnostics drivetrain_diagnostics;

float determine_average_angular_velocity()
{
	float temp = ck::math::deg2rad(drivetrain_diagnostics.actual_angular_speed_deg_s);
	return temp;
}


// This isn't a really great place for this to live, but because of how the downstream
// control is laid out I'm going to put it here - MGT
geometry::Twist perform_heading_stabilization(geometry::Twist twist, geometry::Pose heading_pose, bool enable_absolute_heading)
{
	static bool resist_rotation = true;

	double target_angular_velocity = twist.angular.yaw();
	static float last_target_angular_velocity = target_angular_velocity;
	bool disable_velocity_resistance = false;
	static double last_reset = ros::Time::now().toSec();
	if(abs(target_angular_velocity) < abs(last_target_angular_velocity) ||
	   (target_angular_velocity == 0 && !last_target_angular_velocity == 0))
	{
		last_reset = ros::Time::now().toSec();
	}
	if (ros::Time::now().toSec() < last_reset + 1.5)
	{
		disable_velocity_resistance = true;
	}

	last_target_angular_velocity = target_angular_velocity;

	if(resist_rotation && !disable_velocity_resistance)
	{
		float average_angular_velocity = determine_average_angular_velocity();
		if(abs(target_angular_velocity) < abs(average_angular_velocity))
		{
			float angular_velocity_error = target_angular_velocity - average_angular_velocity;
			float angular_response_kP = 1.25;
			float angular_command_offset = angular_velocity_error * angular_response_kP;
			target_angular_velocity += angular_command_offset;
		}
	}

	target_angular_velocity = std::clamp(target_angular_velocity, -config_params::robot_max_ang_vel, config_params::robot_max_ang_vel);

	drivetrain_diagnostics.compensated_target_angular_speed_deg_s = ck::math::rad2deg(target_angular_velocity);

	geometry::Pose robot_pose;
	robot_pose.transform(get_robot_transform());

	if (enable_absolute_heading)
	{
		float heading_error = smallest_traversal(robot_pose.orientation.yaw(), heading_pose.orientation.yaw());
		float heading_response_kP = 1.0;
		float heading_command_offset = heading_error * heading_response_kP;
		target_angular_velocity += heading_command_offset;
	}

	target_angular_velocity = std::clamp(target_angular_velocity, -config_params::robot_max_ang_vel, config_params::robot_max_ang_vel);

	drivetrain_diagnostics.heading_absolute_compensated_angular_speed_deg_s = target_angular_velocity;

	twist.angular.yaw(target_angular_velocity);
	return twist;
}

geometry::Twist perform_field_alignment(geometry::Twist input, bool field_orient)
{
	geometry::Twist return_twist = input;
	geometry::Twist body_twist = return_twist;

	if(field_orient)
	{
		geometry::Rotation robot_inverse_rotation(-get_robot_transform().angular);
		return_twist.linear = return_twist.linear.Rotate(robot_inverse_rotation);
	}

	if (!field_orient)
	{
		drivetrain_diagnostics.body_target_x_translation_m_s = return_twist.linear.x();
		drivetrain_diagnostics.body_target_y_translation_m_s = return_twist.linear.y();
		geometry::Rotation robot_field_orienting_rotation(get_robot_transform().angular);
		geometry::Translation field_transform = return_twist.linear.Rotate(robot_field_orienting_rotation);
		drivetrain_diagnostics.field_target_x_translation_m_s = field_transform.x();
		drivetrain_diagnostics.field_target_y_translation_m_s = field_transform.y();
	}
	else
	{
		drivetrain_diagnostics.body_target_x_translation_m_s = body_twist.linear.x();
		drivetrain_diagnostics.body_target_y_translation_m_s = body_twist.linear.y();
		drivetrain_diagnostics.field_target_x_translation_m_s = return_twist.linear.x();
		drivetrain_diagnostics.field_target_y_translation_m_s = return_twist.linear.y();
	}

	drivetrain_diagnostics.target_total_speed_m_s = return_twist.linear.norm();
	return return_twist;
}

geometry::Twist get_twist_from_auto()
{
    geometry::Twist return_twist;

	return_twist = geometry::to_twist(auto_control.twist);
	geometry::Pose heading_pose = geometry::to_pose(auto_control.pose);

	// Always call both perform field alignment, and heading stabilization even if you know you'll
	// never use field oriented so that the proper debugging data is set;
	return_twist = perform_field_alignment(return_twist, false);
	return_twist = perform_heading_stabilization(return_twist, heading_pose, true);

    return return_twist;
}

geometry::Twist get_twist_from_HMI()
{
	bool field_orient = true;
	drivetrain_diagnostics.field_orient = field_orient;

	double percent_max_fwd_vel = hmi_signals.drivetrain_swerve_percent_fwd_vel;
	double direction = hmi_signals.drivetrain_swerve_direction;
	double percent_max_ang_vel = hmi_signals.drivetrain_swerve_percent_angular_rot;

	float target_angular_velocity = (percent_max_ang_vel * config_params::robot_max_ang_vel);
	drivetrain_diagnostics.target_angular_speed_deg_s = ck::math::rad2deg(target_angular_velocity);

	geometry::Twist return_twist;

	return_twist.linear.x(percent_max_fwd_vel * std::cos(direction) * config_params::robot_max_fwd_vel);
	return_twist.linear.y(percent_max_fwd_vel * std::sin(direction) * config_params::robot_max_fwd_vel);
	return_twist.angular.yaw(target_angular_velocity);

	// No heading pose used in teleop
	geometry::Pose heading_pose;
	return_twist = perform_heading_stabilization(return_twist, heading_pose, false);
	return_twist = perform_field_alignment(return_twist, field_orient);

	return return_twist;
}