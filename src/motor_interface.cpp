#include <ck_utilities/Motor.hpp>
#include "motor_interface.hpp"
#include <std_msgs/Float32MultiArray.h>
#include <ck_utilities/CKMath.hpp>
#include <ck_ros_msgs_node/Swerve_Drivetrain_Diagnostics.h>
#include "swerve_drivetrain_node.hpp"
#include "swerve_drive_helper.hpp"
#include "config_params.hpp"
#include <ck_utilities/AccelRamper.hpp>
#include <ck_utilities/Logger.hpp>
#include <ck_utilities/CKMath.hpp>

std::vector<Motor*> drive_motors;
std::vector<Motor*> steering_motors;

extern ck_ros_msgs_node::Swerve_Drivetrain_Diagnostics drivetrain_diagnostics;

double accel_m_s_s_to_rpm_s(double accel)
{
    return accel / (ck::math::inches_to_meters(config_params::wheel_diameter_inches) * M_PI) * 60;
}

double calculate_percent_output_from_speed(double current_speed, double target_speed, double accel, double decel, AccelRamper * ramper)
{
    float kv = 82.3;
    float ka = 450;
    float ks = 0.4 / 4.5;

    float available_voltage = 12.0 - (std::abs(current_speed) / kv);
    float available_accel = available_voltage * ka;
    available_accel = available_accel < accel ? available_accel : accel;
    float available_decel = decel;

    ramper->update_params(available_accel, available_decel, 0, 1);
    target_speed = ramper->calculateOutput(target_speed);

    double error = target_speed - current_speed;
    int8_t error_sign = error / std::abs(error);
    double error_dampening = 1.0 / (ck::math::inches_to_meters(config_params::wheel_diameter_inches) * M_PI) * 60.0;
    error = std::abs(error);
    error /= error_dampening;
    error = ck::math::limit(error, 1.0);

    available_accel *= error;

    float output_value = (target_speed / kv / 12.0) + ((available_accel / ka / 12.0) * error_sign);

    if (output_value < ks && target_speed > 0)
    {
        output_value = ks;
    }

    ck::log_error << "AV: " << available_voltage << " AA: " << available_accel << " CS: " << current_speed << " TS: " << target_speed << " OV: " << output_value << " IA: " << accel << std::flush;

    return output_value;
    // abject horror ^^
}

std::vector<float> calculate_accel_fade()
{
    double track_to_heading = drivetrain_diagnostics.target_track - drivetrain_diagnostics.actual_heading;

    geometry::Rotation accel_rotation_matrix;
    accel_rotation_matrix.yaw(track_to_heading);

    bool first = true;

    float min_x = 0;
    float max_x = 0;

    std::vector<float> results;

    float k_accel_fade = 0.01; // m/s

    for (size_t i = 0; i < drive_motors.size(); i ++)
    {
        geometry::Transform accel_adjusted_transform = wheel_transforms[i].rotate(accel_rotation_matrix);

        if (first)
        {
            min_x = accel_adjusted_transform.linear.x();
            max_x = accel_adjusted_transform.linear.x();
            first = false;
        }

        if (accel_adjusted_transform.linear.x() < min_x)
        {
            min_x = accel_adjusted_transform.linear.x();
        }

        if (accel_adjusted_transform.linear.x() > max_x)
        {
            max_x = accel_adjusted_transform.linear.x();
        }
    }

    for (size_t i = 0; i < drive_motors.size(); i ++)
    {
        if(max_x - min_x > 0.05)
        {
            geometry::Transform accel_adjusted_transform = wheel_transforms[i].rotate(accel_rotation_matrix);

            float fade = (accel_adjusted_transform.linear.x() - min_x) / (max_x - min_x);
            fade = 1 - fade;

            fade *= k_accel_fade;

            results.push_back(fade);
        }
        else
        {
            results.push_back(0);
        }
    }

    return results;
}

void set_swerve_output(std::vector<std::pair<geometry::Pose, geometry::Twist>> sdo)
{
    double accel = config_params::robot_max_fwd_accel;
    double decel = config_params::robot_max_fwd_decel;

    std::vector<float> fades = calculate_accel_fade();
    static std::vector<AccelRamper *> value_rampers;

    static ros::Time last_run = ros::Time::now();

    static bool first_pass = true;
    if (first_pass)
    {
        for (size_t i = 0; i < drive_motors.size(); i++)
        {
            value_rampers.push_back(new AccelRamper(accel, decel, 0, 1));
        }
        first_pass = false;
    }

    if (ros::Time::now().toSec() - last_run.toSec() > 0.1)
    {
        for (auto &i : value_rampers)
        {
            i->reset();
        }
    }

    last_run = ros::Time::now();

	for (size_t i = 0; i < drive_motors.size(); i++)
	{
        double local_accel = accel - fades[i];
        ck::log_error << "Fade: " << fades[i] << std::flush;
        local_accel = std::max(0.0, local_accel);
        double accel_rpm_s = accel_m_s_s_to_rpm_s(local_accel);

        double local_decel = decel;
        double decel_rpm_s = accel_m_s_s_to_rpm_s(local_decel);

        double speed_target = sdo[i].second.linear.x() / (ck::math::inches_to_meters(config_params::wheel_diameter_inches) * M_PI) * 60.0;
        // drive_motors[i]->set( Motor::Control_Mode::VELOCITY, speed_target, 0 );
        double percent_output = calculate_percent_output_from_speed(motor_map[config_params::drive_motor_ids[i]].sensor_velocity, speed_target, accel_rpm_s, decel_rpm_s, value_rampers[i]);

        drivetrain_diagnostics.modules[i].target_speed_rpm = speed_target;
        drivetrain_diagnostics.modules[i].ramped_target_speed_rpm = value_rampers[i]->get_value();

        drivetrain_diagnostics.modules[i].commanded_percent_out = percent_output;
        drivetrain_diagnostics.modules[i].target_accel = local_accel;

        drive_motors[i]->set( Motor::Control_Mode::PERCENT_OUTPUT, percent_output, 0 );

        float delta = smallest_traversal(ck::math::normalize_to_2_pi(motor_map[config_params::steering_motor_ids[i]].sensor_position * 2.0 * M_PI), ck::math::normalize_to_2_pi(sdo[i].first.orientation.yaw()));
        float target = (motor_map[config_params::steering_motor_ids[i]].sensor_position * 2.0 * M_PI) + delta;
        steering_motors[i]->set( Motor::Control_Mode::POSITION, target / (2.0 * M_PI), 0 );

        drivetrain_diagnostics.modules[i].target_steering_angle_deg = ck::math::rad2deg(ck::math::normalize_to_2_pi(sdo[i].first.orientation.yaw()));
        drivetrain_diagnostics.modules[i].actual_steering_angle_deg = ck::math::rad2deg(ck::math::normalize_to_2_pi(motor_map[config_params::steering_motor_ids[i]].sensor_position * 2.0 * M_PI));
        drivetrain_diagnostics.modules[i].target_speed_m_s = sdo[i].second.linear.x();
        drivetrain_diagnostics.modules[i].actual_speed_m_s = motor_map[config_params::drive_motor_ids[i]].sensor_velocity * (ck::math::inches_to_meters(config_params::wheel_diameter_inches) * M_PI) / 60.0;
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