#include <ck_utilities/Motor.hpp>
#include "motor_interface.hpp"
#include <std_msgs/Float32MultiArray.h>
#include <ck_utilities/CKMath.hpp>
#include <ck_ros_msgs_node/Swerve_Drivetrain_Diagnostics.h>
#include "swerve_drivetrain_node.hpp"
#include "swerve_drive_helper.hpp"
#include "config_params.hpp"
#include <ck_utilities/ValueRamper.hpp>

std::vector<Motor*> drive_motors;
std::vector<Motor*> steering_motors;

extern ck_ros_msgs_node::Swerve_Drivetrain_Diagnostics drivetrain_diagnostics;

void set_swerve_output(std::vector<std::pair<geometry::Pose, geometry::Twist>> sdo)
{
	for (size_t i = 0; i < drive_motors.size(); i++)
	{
		double speed_target = sdo[i].second.linear.x() / (0.1016 * M_PI) * 60.0;
		drive_motors[i]->set( Motor::Control_Mode::VELOCITY, speed_target, 0 );
		float delta = smallest_traversal(ck::math::normalize_to_2_pi(motor_map[config_params::steering_motor_ids[i]].sensor_position * 2.0 * M_PI), ck::math::normalize_to_2_pi(sdo[i].first.orientation.yaw()));
		float target = (motor_map[config_params::steering_motor_ids[i]].sensor_position * 2.0 * M_PI) + delta;
		steering_motors[i]->set( Motor::Control_Mode::POSITION, target / (2.0 * M_PI), 0 );

		drivetrain_diagnostics.modules[i].target_steering_angle_deg = ck::math::rad2deg(ck::math::normalize_to_2_pi(sdo[i].first.orientation.yaw()));
		drivetrain_diagnostics.modules[i].actual_steering_angle_deg = ck::math::rad2deg(ck::math::normalize_to_2_pi(motor_map[config_params::steering_motor_ids[i]].sensor_position * 2.0 * M_PI));
		drivetrain_diagnostics.modules[i].target_speed_m_s = sdo[i].second.linear.x();
		drivetrain_diagnostics.modules[i].actual_speed_m_s = motor_map[config_params::drive_motor_ids[i]].sensor_velocity * (0.1016 * M_PI) / 60.0;

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
		drive_motor->config().set_supply_current_limit(true, config_params::drive_supply_current_limit, config_params::drive_supply_current_limit_threshold, config_params::drive_supply_current_limit_threshold_exceeded_time);
		drive_motor->config().set_kP(config_params::drive_velocity_kP);
		drive_motor->config().set_kI(config_params::drive_velocity_kI);
		drive_motor->config().set_kD(config_params::drive_velocity_kD);
		drive_motor->config().set_kF(config_params::drive_velocity_kF);
		drive_motor->config().set_i_zone(config_params::drive_velocity_iZone);
		drive_motor->config().set_max_i_accum(config_params::drive_velocity_maxIAccum);
		drive_motor->config().set_closed_loop_ramp(config_params::drive_closed_loop_ramp);
		drive_motor->config().apply();
		drive_motors.push_back(drive_motor);
		ck_ros_msgs_node::Swerve_Drivetrain_Module_Diagnostics diagnostic_module;
		diagnostic_module.drive_motor_id = config_params::drive_motor_ids[i];
		diagnostic_module.steering_motor_id = config_params::steering_motor_ids[i];
		drivetrain_diagnostics.modules.push_back(diagnostic_module);
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
		steering_motor->config().set_supply_current_limit(true, config_params::steering_supply_current_limit, config_params::steering_supply_current_limit_threshold, config_params::steering_supply_current_limit_threshold_exceeded_time);
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
		drivetrain_diagnostics.modules[i].x_transform_m = wheel.linear.x();
		drivetrain_diagnostics.modules[i].y_transform_m = wheel.linear.y();
	}
}