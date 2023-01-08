#include <ck_utilities/Motor.hpp>
#include "motor_interface.hpp"
#include <ck_utilities/ParameterHelper.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <ck_utilities/CKMath.hpp>
#include "swerve_drivetrain_node.hpp"
#include "swerve_drive_helper.hpp"

int robot_num_wheels;
std::vector<int> drive_motor_ids;
std::vector<int> steering_motor_ids;
std::vector<bool> drive_motor_inverted;
std::vector<bool> drive_sensor_inverted;
std::vector<bool> steering_motor_inverted;
std::vector<bool> steering_sensor_inverted;
std::vector<double> robot_wheel_inches_from_center_x;
std::vector<double> robot_wheel_inches_from_center_y;
double robot_max_fwd_vel;
double robot_max_ang_vel;

int motor_type;
double voltage_comp_saturation;
bool voltage_comp_enabled;

bool brake_mode_default;

double wheel_diameter_inches;

double drive_closed_loop_ramp;
double drive_motion_cruise_velocity;
double drive_motion_accel;
double drive_motion_s_curve_strength;

double steering_velocity_kP;
double steering_velocity_kI;
double steering_velocity_kD;
double steering_velocity_kF;
double steering_velocity_iZone;
double steering_velocity_maxIAccum;
double steering_closed_loop_ramp;
double steering_motion_cruise_velocity;
double steering_motion_accel;
double steering_motion_s_curve_strength;

double open_loop_ramp;
double supply_current_limit;
double supply_current_limit_threshold;
double supply_current_limit_threshold_exceeded_time;

std::vector<Motor*> drive_motors;
std::vector<Motor*> steering_motors;

bool init_params(ros::NodeHandle &n)
{
	bool required_params_found = true;

	required_params_found &= n.getParam(CKSP(robot_num_wheels), robot_num_wheels);
	required_params_found &= robot_num_wheels > 0;
	required_params_found &= n.getParam(CKSP(drive_motor_ids), drive_motor_ids);
	required_params_found &= n.getParam(CKSP(steering_motor_ids), steering_motor_ids);
	required_params_found &= n.getParam(CKSP(drive_motor_inverted), drive_motor_inverted);
	required_params_found &= n.getParam(CKSP(drive_sensor_inverted), drive_sensor_inverted);
	required_params_found &= n.getParam(CKSP(steering_motor_inverted), steering_motor_inverted);
	required_params_found &= n.getParam(CKSP(steering_sensor_inverted), steering_sensor_inverted);
	required_params_found &= n.getParam(CKSP(robot_wheel_inches_from_center_x), robot_wheel_inches_from_center_x);
	required_params_found &= n.getParam(CKSP(robot_wheel_inches_from_center_y), robot_wheel_inches_from_center_y);
	required_params_found &= drive_motor_ids.size() == (size_t)robot_num_wheels;
	required_params_found &= steering_motor_ids.size() == (size_t)robot_num_wheels;
	required_params_found &= drive_motor_inverted.size() == (size_t)robot_num_wheels;
	required_params_found &= drive_sensor_inverted.size() == (size_t)robot_num_wheels;
	required_params_found &= steering_motor_inverted.size() == (size_t)robot_num_wheels;
	required_params_found &= steering_sensor_inverted.size() == (size_t)robot_num_wheels;
	required_params_found &= robot_wheel_inches_from_center_x.size() == (size_t)robot_num_wheels;
	required_params_found &= robot_wheel_inches_from_center_y.size() == (size_t)robot_num_wheels;
	required_params_found &= n.getParam(CKSP(robot_max_fwd_vel), robot_max_fwd_vel);
	required_params_found &= n.getParam(CKSP(robot_max_ang_vel), robot_max_ang_vel);
	required_params_found &= n.getParam(CKSP(motor_type), motor_type);
	required_params_found &= n.getParam(CKSP(voltage_comp_saturation), voltage_comp_saturation);
	required_params_found &= n.getParam(CKSP(voltage_comp_enabled), voltage_comp_enabled);
	required_params_found &= n.getParam(CKSP(brake_mode_default), brake_mode_default);
	required_params_found &= n.getParam(CKSP(wheel_diameter_inches), wheel_diameter_inches);
	required_params_found &= n.getParam(CKSP(drive_closed_loop_ramp), drive_closed_loop_ramp);
	required_params_found &= n.getParam(CKSP(drive_motion_cruise_velocity), drive_motion_cruise_velocity);
	required_params_found &= n.getParam(CKSP(drive_motion_accel), drive_motion_accel);
	required_params_found &= n.getParam(CKSP(drive_motion_s_curve_strength), drive_motion_s_curve_strength);
	required_params_found &= n.getParam(CKSP(steering_velocity_kP), steering_velocity_kP);
	required_params_found &= n.getParam(CKSP(steering_velocity_kI), steering_velocity_kI);
	required_params_found &= n.getParam(CKSP(steering_velocity_kD), steering_velocity_kD);
	required_params_found &= n.getParam(CKSP(steering_velocity_kF), steering_velocity_kF);
	required_params_found &= n.getParam(CKSP(steering_velocity_iZone), steering_velocity_iZone);
	required_params_found &= n.getParam(CKSP(steering_velocity_maxIAccum), steering_velocity_maxIAccum);
	required_params_found &= n.getParam(CKSP(steering_closed_loop_ramp), steering_closed_loop_ramp);
	required_params_found &= n.getParam(CKSP(steering_motion_cruise_velocity), steering_motion_cruise_velocity);
	required_params_found &= n.getParam(CKSP(steering_motion_accel), steering_motion_accel);
	required_params_found &= n.getParam(CKSP(steering_motion_s_curve_strength), steering_motion_s_curve_strength);
	required_params_found &= n.getParam(CKSP(open_loop_ramp), open_loop_ramp);
	required_params_found &= n.getParam(CKSP(supply_current_limit), supply_current_limit);
	required_params_found &= n.getParam(CKSP(supply_current_limit_threshold), supply_current_limit_threshold);
	required_params_found &= n.getParam(CKSP(supply_current_limit_threshold_exceeded_time), supply_current_limit_threshold_exceeded_time);

	if (!required_params_found)
	{
		ROS_ERROR("Missing required parameters for node %s. Please check the list and make sure all required parameters are included", ros::this_node::getName().c_str());
		return false;
	}

	return true;
}

void set_swerve_output(std::vector<std::pair<geometry::Pose, geometry::Twist>> sdo)
{
	std_msgs::Float32MultiArray msg;
	for (size_t i = 0; i < drive_motors.size(); i++)
	{
		constexpr double MAX_DRIVE_VEL_L1_FALCON = 6380.0 / 8.14 * (0.1016 * M_PI) / 60.0;
		drive_motors[i]->set( Motor::Control_Mode::PERCENT_OUTPUT, sdo[i].second.linear.x() / MAX_DRIVE_VEL_L1_FALCON, 0 );
		float delta = smallest_traversal(ck::math::normalize_to_2_pi(motor_map[steering_motor_ids[i]].sensor_position * 2.0 * M_PI), ck::math::normalize_to_2_pi(sdo[i].first.orientation.yaw()));
		msg.data.push_back(delta);
		float target = (motor_map[steering_motor_ids[i]].sensor_position * 2.0 * M_PI) + delta;
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
    for (int i = 0; i < robot_num_wheels; i++)
	{
        Motor* drive_motor = new Motor( drive_motor_ids[i], (Motor::Motor_Type)motor_type );
		drive_motor->set( Motor::Control_Mode::PERCENT_OUTPUT, 0, 0 );
		drive_motor->config().set_fast_master(true);
		drive_motor->config().set_inverted( drive_motor_inverted[i] );
		drive_motor->config().set_neutral_mode( brake_mode_default ? MotorConfig::NeutralMode::BRAKE : MotorConfig::NeutralMode::COAST);
		drive_motor->config().set_voltage_compensation_saturation( voltage_comp_saturation );
		drive_motor->config().set_voltage_compensation_enabled( voltage_comp_enabled );
		drive_motor->config().set_open_loop_ramp(open_loop_ramp);
		drive_motor->config().set_supply_current_limit(true, supply_current_limit, supply_current_limit_threshold, supply_current_limit_threshold_exceeded_time);
		drive_motor->config().set_closed_loop_ramp(drive_closed_loop_ramp);
		drive_motor->config().apply();
		drive_motors.push_back(drive_motor);
	}

	//Steering Motors
    for (int i = 0; i < robot_num_wheels; i++)
	{
        Motor* steering_motor = new Motor( steering_motor_ids[i], (Motor::Motor_Type)motor_type );
		steering_motor->set( Motor::Control_Mode::PERCENT_OUTPUT, 0, 0 );
		steering_motor->config().set_fast_master(true);
		steering_motor->config().set_inverted( steering_motor_inverted[i] );
		steering_motor->config().set_neutral_mode( MotorConfig::NeutralMode::BRAKE );
		steering_motor->config().set_voltage_compensation_saturation( voltage_comp_saturation );
		steering_motor->config().set_voltage_compensation_enabled( voltage_comp_enabled );
		steering_motor->config().set_open_loop_ramp(open_loop_ramp);
		steering_motor->config().set_supply_current_limit(true, supply_current_limit, supply_current_limit_threshold, supply_current_limit_threshold_exceeded_time);
		steering_motor->config().set_kP(steering_velocity_kP);
		steering_motor->config().set_kI(steering_velocity_kI);
		steering_motor->config().set_kD(steering_velocity_kD);
		steering_motor->config().set_kF(steering_velocity_kF);
		steering_motor->config().set_motion_cruise_velocity(steering_motion_cruise_velocity);
		steering_motor->config().set_motion_acceleration(steering_motion_accel);
		steering_motor->config().set_motion_s_curve_strength(steering_motion_s_curve_strength);
		steering_motor->config().set_i_zone(steering_velocity_iZone);
		steering_motor->config().set_max_i_accum(steering_velocity_maxIAccum);
		steering_motor->config().set_closed_loop_ramp(steering_closed_loop_ramp);
		steering_motor->config().apply();
		steering_motors.push_back(steering_motor);
    }

	//Init swerve configuration
	for (int i = 0; i < robot_num_wheels; i++)
	{
		geometry::Transform wheel;
		wheel.linear.x(ck::math::inches_to_meters(robot_wheel_inches_from_center_x[i]));
		wheel.linear.y(ck::math::inches_to_meters(robot_wheel_inches_from_center_y[i]));
		wheel_transforms.push_back(wheel);
	}
}