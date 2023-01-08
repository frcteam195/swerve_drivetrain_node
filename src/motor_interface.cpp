#include <ck_utilities/Motor.hpp>
#include "motor_interface.hpp"
#include <std_msgs/Float32MultiArray.h>
#include <ck_utilities/CKMath.hpp>
#include "swerve_drivetrain_node.hpp"
#include "swerve_drive_helper.hpp"
#include "config_params.hpp"

std::vector<Motor*> drive_motors;
std::vector<Motor*> steering_motors;

void set_swerve_output(std::vector<std::pair<geometry::Pose, geometry::Twist>> sdo)
{
	std_msgs::Float32MultiArray msg;
	for (size_t i = 0; i < drive_motors.size(); i++)
	{
		constexpr double MAX_DRIVE_VEL_L1_FALCON = 6380.0 / 8.14 * (0.1016 * M_PI) / 60.0;
		drive_motors[i]->set( Motor::Control_Mode::PERCENT_OUTPUT, sdo[i].second.linear.x() / MAX_DRIVE_VEL_L1_FALCON, 0 );
		float delta = smallest_traversal(ck::math::normalize_to_2_pi(motor_map[config_params::steering_motor_ids[i]].sensor_position * 2.0 * M_PI), ck::math::normalize_to_2_pi(sdo[i].first.orientation.yaw()));
		msg.data.push_back(delta);
		float target = (motor_map[config_params::steering_motor_ids[i]].sensor_position * 2.0 * M_PI) + delta;
		steering_motors[i]->set( Motor::Control_Mode::MOTION_MAGIC, target / (2.0 * M_PI), 0 );
	}
}

void set_brake_mode(bool brake)
{
	if (brake)
	{
		for (Motor* mF : drive_motors)
		{
			mF->config().set_neutral_mode(MotorConfig::NeutralMode::BRAKE);
			mF->config().apply();
		}
	}
	else
	{
		for (Motor* mF : drive_motors)
		{
			mF->config().set_neutral_mode(MotorConfig::NeutralMode::COAST);
			mF->config().apply();
		}
	}
}

void init_swerve_motors()
{
	//Drive Motors
    for (int i = 0; i < config_params::robot_num_wheels; i++)
	{
        Motor* drive_motor = new Motor( config_params::drive_motor_ids[i], (Motor::Motor_Type)config_params::motor_type );
		drive_motor->set( Motor::Control_Mode::PERCENT_OUTPUT, 0, 0 );
		drive_motor->config().set_fast_master(true);
		drive_motor->config().set_inverted( config_params::drive_motor_inverted[i] );
		drive_motor->config().set_neutral_mode( config_params::brake_mode_default ? MotorConfig::NeutralMode::BRAKE : MotorConfig::NeutralMode::COAST);
		drive_motor->config().set_voltage_compensation_saturation( config_params::voltage_comp_saturation );
		drive_motor->config().set_voltage_compensation_enabled( config_params::voltage_comp_enabled );
		drive_motor->config().set_open_loop_ramp(config_params::open_loop_ramp);
		drive_motor->config().set_supply_current_limit(true, config_params::supply_current_limit, config_params::supply_current_limit_threshold, config_params::supply_current_limit_threshold_exceeded_time);
		drive_motor->config().set_closed_loop_ramp(config_params::drive_closed_loop_ramp);
		drive_motor->config().apply();
		drive_motors.push_back(drive_motor);
	}

	//Steering Motors
    for (int i = 0; i < config_params::robot_num_wheels; i++)
	{
        Motor* steering_motor = new Motor( config_params::steering_motor_ids[i], (Motor::Motor_Type)config_params::motor_type );
		steering_motor->set( Motor::Control_Mode::PERCENT_OUTPUT, 0, 0 );
		steering_motor->config().set_fast_master(true);
		steering_motor->config().set_inverted( config_params::steering_motor_inverted[i] );
		steering_motor->config().set_neutral_mode( MotorConfig::NeutralMode::BRAKE );
		steering_motor->config().set_voltage_compensation_saturation( config_params::voltage_comp_saturation );
		steering_motor->config().set_voltage_compensation_enabled( config_params::voltage_comp_enabled );
		steering_motor->config().set_open_loop_ramp(config_params::open_loop_ramp);
		steering_motor->config().set_supply_current_limit(true, config_params::supply_current_limit, config_params::supply_current_limit_threshold, config_params::supply_current_limit_threshold_exceeded_time);
		steering_motor->config().set_kP(config_params::steering_velocity_kP);
		steering_motor->config().set_kI(config_params::steering_velocity_kI);
		steering_motor->config().set_kD(config_params::steering_velocity_kD);
		steering_motor->config().set_kF(config_params::steering_velocity_kF);
		steering_motor->config().set_motion_cruise_velocity(config_params::steering_motion_cruise_velocity);
		steering_motor->config().set_motion_acceleration(config_params::steering_motion_accel);
		steering_motor->config().set_motion_s_curve_strength(config_params::steering_motion_s_curve_strength);
		steering_motor->config().set_i_zone(config_params::steering_velocity_iZone);
		steering_motor->config().set_max_i_accum(config_params::steering_velocity_maxIAccum);
		steering_motor->config().set_closed_loop_ramp(config_params::steering_closed_loop_ramp);
		steering_motor->config().apply();
		steering_motors.push_back(steering_motor);
    }

	//Init swerve configuration
	for (int i = 0; i < config_params::robot_num_wheels; i++)
	{
		geometry::Transform wheel;
		wheel.linear.x(ck::math::inches_to_meters(config_params::robot_wheel_inches_from_center_x[i]));
		wheel.linear.y(ck::math::inches_to_meters(config_params::robot_wheel_inches_from_center_y[i]));
		wheel_transforms.push_back(wheel);
	}
}