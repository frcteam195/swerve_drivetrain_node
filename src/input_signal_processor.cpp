#include "input_signal_processor.hpp"

geometry::Twist get_twist_from_HMI()
{
	bool field_orient = true;

	double percent_max_fwd_vel = hmi_signals.drivetrain_swerve_percent_fwd_vel;
	double direction = hmi_signals.drivetrain_swerve_direction;
	double percent_max_ang_vel = hmi_signals.drivetrain_swerve_percent_angular_rot;

	geometry::Twist return_twist;

	return_twist.linear.x(percent_max_fwd_vel * std::cos(direction) * config_params::robot_max_fwd_vel);
	return_twist.linear.y(percent_max_fwd_vel * std::sin(direction) * config_params::robot_max_fwd_vel);
	return_twist.angular.yaw(percent_max_ang_vel * config_params::robot_max_ang_vel);

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