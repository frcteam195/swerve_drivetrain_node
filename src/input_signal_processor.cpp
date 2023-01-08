#include "input_signal_processor.hpp"
#include "swerve_drivetrain_node.hpp"
#include "odometry_interface.hpp"
#include "config_params.hpp"
#include "ck_utilities/MovingAverage.hpp"

float determine_average_angular_velocity()
{
	geometry::Transform robot_transform = get_robot_transform();
	static float last_angle = robot_transform.angular.yaw();
	float current_angle = robot_transform.angular.yaw();
	static ck::MovingAverage average_delta_position(10);
	average_delta_position.addSample(current_angle - last_angle);
	last_angle = current_angle;

	static ros::Time last_time = ros::Time::now();
	ros::Time current_time = ros::Time::now();
	float duration_m_s = (current_time.toSec() - last_time.toSec()) * 1000.0;
	static ck::MovingAverage average_time(10);
	average_time.addSample(duration_m_s);

	return average_delta_position.getAverage() / average_time.getAverage();
}

geometry::Twist get_twist_from_HMI()
{
	bool field_orient = true;

	double percent_max_fwd_vel = hmi_signals.drivetrain_swerve_percent_fwd_vel;
	double direction = hmi_signals.drivetrain_swerve_direction;
	double percent_max_ang_vel = hmi_signals.drivetrain_swerve_percent_angular_rot;

	static bool resist_rotation = false;

	float target_angular_velocity = (percent_max_ang_vel * config_params::robot_max_ang_vel);

	if(resist_rotation)
	{
		float average_angular_velocity = determine_average_angular_velocity();
		if(abs(target_angular_velocity) < abs(average_angular_velocity))
		{
			float angular_velocity_error = target_angular_velocity - average_angular_velocity;
			float angular_response_kP = 1.0;
			float angular_command_offset = angular_velocity_error * angular_response_kP;
			target_angular_velocity += angular_command_offset;
		}
	}

	geometry::Twist return_twist;

	return_twist.linear.x(percent_max_fwd_vel * std::cos(direction) * config_params::robot_max_fwd_vel);
	return_twist.linear.y(percent_max_fwd_vel * std::sin(direction) * config_params::robot_max_fwd_vel);
	return_twist.angular.yaw(target_angular_velocity);

	if(field_orient)
	{
		geometry::Rotation robot_inverse_rotation(-get_robot_transform().angular);
		return_twist.linear = return_twist.linear.Rotate(robot_inverse_rotation);
	}

	return return_twist;
}

geometry::Twist get_twist_from_auto()
{
    //MGT TBD TODO FIXME - needs to be implemented
    geometry::Twist return_twist;
    return return_twist;
}