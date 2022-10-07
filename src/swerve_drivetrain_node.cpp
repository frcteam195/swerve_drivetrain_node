#include "swerve_drivetrain_node.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <map>
#include <mutex>
#include <iostream>
#include <iomanip>
#include <functional>

#include <nav_msgs/Odometry.h>
#include <rio_control_node/Joystick_Status.h>
#include <rio_control_node/Robot_Status.h>
#include <rio_control_node/Motor_Control.h>
#include <rio_control_node/Motor_Configuration.h>
#include <rio_control_node/Motor_Status.h>
#include <ck_utilities/Motor.hpp>
#include <ck_utilities/CKMath.hpp>
#include <ck_utilities/ParameterHelper.hpp>
#include <ck_utilities/ValueRamper.hpp>
#include <ck_utilities/MovingAverage.hpp>
#include <ck_utilities/swerve/SwerveDriveOutput.hpp>
#include <ck_utilities/swerve/SwerveDriveConfig.hpp>

#include <hmi_agent_node/HMI_Signals.h>

#include "swerve_drive_helper.hpp"
#include "swerve_drivetrain_node/Swerve_Drivetrain_Diagnostics.h"
#include <quesadilla_auto_node/Planner_Output.h>
#include <ck_utilities/geometry/geometry.hpp>

//#define CHARACTERIZE_DRIVE
#ifdef CHARACTERIZE_DRIVE
#include "drive_physics_characterizer_node/Drive_Characterization_Output.h"
#endif

#define INCHES_TO_METERS 0.0254

//#define DYNAMIC_RECONFIGURE_TUNING
#ifdef DYNAMIC_RECONFIGURE_TUNING
#include <dynamic_reconfigure/server.h>
#include <drivetrain_node/DriveTuningConfig.h>
#endif

ros::NodeHandle* node;
static constexpr double ENCODER_TICKS_TO_M_S = 1.0;

int mRobotStatus;
float mJoystick1x;
float mJoystick1y;
std::mutex mThreadCtrlLock;
uint32_t mConfigUpdateCounter;
static bool about_to_shoot = false;

std::vector<Motor*> drive_motors;
std::vector<Motor*> steering_motors;
ck::swerve::SwerveDriveConfig swerve_drive_config;

swerve_drivetrain_node::Swerve_Drivetrain_Diagnostics swerve_drivetrain_diagnostics;
quesadilla_auto_node::Planner_Output drive_planner_output_msg;

static constexpr double kDriveGearReduction = (50.0 / 11.0) * (44.0 / 30.0);
static constexpr double kDriveRotationsPerTick = 1.0 / 2048.0 * 1.0 / kDriveGearReduction;

enum class DriveControlMode : int
{
	SWERVE_VELOCITY_FEEDFORWARD = 0
};

static DriveControlMode drive_control_config {DriveControlMode::SWERVE_VELOCITY_FEEDFORWARD};

#ifdef DYNAMIC_RECONFIGURE_TUNING
enum class DriveTuningMode : int
{
	Normal = 0,
	TuningVelocityPID = 1,
	TuningkVkA = 2
};

static DriveTuningMode tuning_control_mode = DriveTuningMode::Normal;
static double tuning_velocity_target = 0;
void tuning_config_callback(drivetrain_node::DriveTuningConfig &config, uint32_t level)
{
	(void) level;
	leftMasterMotor->config().set_kP(config.drive_kP);
	leftMasterMotor->config().set_kI(config.drive_kI);
	leftMasterMotor->config().set_kD(config.drive_kD);
	leftMasterMotor->config().set_kF(config.drive_kF);
	leftMasterMotor->config().set_i_zone(config.drive_iZone);
	leftMasterMotor->config().set_max_i_accum(config.drive_maxIAccum);
	leftMasterMotor->config().set_closed_loop_ramp(config.drive_closed_loop_ramp);

	rightMasterMotor->config().set_kP(config.drive_kP);
	rightMasterMotor->config().set_kI(config.drive_kI);
	rightMasterMotor->config().set_kD(config.drive_kD);
	rightMasterMotor->config().set_kF(config.drive_kF);
	rightMasterMotor->config().set_i_zone(config.drive_iZone);
	rightMasterMotor->config().set_max_i_accum(config.drive_maxIAccum);
	rightMasterMotor->config().set_closed_loop_ramp(config.drive_closed_loop_ramp);

	tuning_control_mode = (DriveTuningMode)config.drive_control_mode;
	tuning_velocity_target = config.drive_tuning_velocity_target;
	drive_Kv = config.drive_kV;
	drive_Ka = config.drive_kA;
}
#endif

bool forEachWheel(std::function<void(int)> func)
{
	try
	{
		for(int i = 0; i < robot_num_wheels; i++)
		{
			func(i);
		}
	}
	catch (...)
	{
		return false;
	}
	return true;
}

void robotStatusCallback(const rio_control_node::Robot_Status& msg)
{
	std::lock_guard<std::mutex> lock(mThreadCtrlLock);
	mRobotStatus = msg.robot_state;
}


#ifdef CHARACTERIZE_DRIVE
static bool characterizing_drive = false;
static double characterizing_drive_left_output = true;
static double characterizing_drive_right_output = true;
void drive_characterization_callback(const drive_physics_characterizer_node::Drive_Characterization_Output& msg)
{
	characterizing_drive = msg.characterizing_drive;
	characterizing_drive_left_output = msg.left_drive_output;
	characterizing_drive_right_output = msg.right_drive_output;
}
#endif

geometry_msgs::Twist get_twist_from_input(double percent_max_fwd_vel, double direction, double percent_max_ang_vel)
{
	geometry_msgs::Twist return_twist;

	return_twist.linear.x = percent_max_fwd_vel * std::cos(direction) * robot_max_fwd_vel;
	return_twist.linear.y = percent_max_fwd_vel * std::sin(direction) * robot_max_fwd_vel;
	return_twist.angular.z = percent_max_ang_vel * robot_max_ang_vel;

	return return_twist;
}

ck::swerve::SwerveDriveOutput calculate_swerve_output_from_twist(geometry_msgs::Twist twist)
{
	ck::swerve::SwerveDriveOutput sdo;
	auto swrv = calculate_swerve_outputs(twist, swerve_drive_config, 0.01);
	for(int i = 0; i < robot_num_wheels; i++)
	{
		{
			tf2::Quaternion q;
			tf2::fromMsg(swrv[i].first.orientation, q);
			double r = 0;
			double p = 0;
			double y = 0;
			tf2::Matrix3x3(q).getRPY(r, p, y);
			sdo.wheels[i].angle = y;
		}

		{
			double x = swrv[i].second.linear.x;
			double y = swrv[i].second.linear.y;
			sdo.wheels[i].velocity = ck::math::hypotenuse(x, y);
		}
	}

	return sdo;
}



// void turret_status_callback(const turret_node::Turret_Status& msg)
// {
// 	about_to_shoot = msg.about_to_shoot;
// }

void planner_callback(const quesadilla_auto_node::Planner_Output& msg)
{
	std::lock_guard<std::mutex> lock(mThreadCtrlLock);
	drive_planner_output_msg = msg;
}

void publishOdometryData(std::map<uint16_t, rio_control_node::Motor_Info>& motor_map)
{
	tf2::Vector3 wheel_vel_sum(0, 0, 0);
	for (int i = 0; i < robot_num_wheels; i++)
	{
		double curr_vel_m_s = motor_map[drive_motor_ids[i]].sensor_velocity * M_PI * ck::math::inches_to_meters(wheel_diameter_inches);
		double curr_angle = ck::math::normalize_to_2_pi(ck::math::deg2rad(motor_map[steering_motor_ids[i]].sensor_position * 360.0));
		tf2::Vector3 wheel_vel(curr_vel_m_s * std::cos(curr_angle), curr_vel_m_s * std::sin(curr_angle), 0);
		wheel_vel_sum += wheel_vel;
	}

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

	static ros::Publisher odometry_publisher = node->advertise<nav_msgs::Odometry>("/RobotOdometry", 1);
	odometry_publisher.publish(odometry_data);
}

void motorStatusCallback(const rio_control_node::Motor_Status& msg)
{
	static ros::Time prev_time(0);
	static std::map<uint16_t, rio_control_node::Motor_Info> motor_map;
	for (const rio_control_node::Motor_Info& m : msg.motors)
	{
		motor_map[m.id] = m;
	}

	publishOdometryData(motor_map);

	ros::Time curr_time = ros::Time::now();
	double dt = (curr_time - prev_time).toSec();



	swerve_drivetrain_diagnostics.dt = dt;
	prev_time = curr_time;

	//Update swerve steering transforms
	for (int i = 0; i < robot_num_wheels; i++)
	{
		tf2::Quaternion quat_tf;
		quat_tf.setRPY(0, 0, ck::math::normalize_to_2_pi(ck::math::deg2rad(motor_map[steering_motor_ids[i]].sensor_position * 360.0)));
		swerve_drive_config.wheels[i].transform.rotation = tf2::toMsg(quat_tf);
	}
}

void hmiSignalsCallback(const hmi_agent_node::HMI_Signals& msg)
{
	std::lock_guard<std::mutex> lock(mThreadCtrlLock);
	ROS_INFO("Start HMI Callback");
	bool brake_mode = msg.drivetrain_brake;
	ROS_INFO("Robot Mode: %d", mRobotStatus);
	switch (mRobotStatus)
	{
	case rio_control_node::Robot_Status::AUTONOMOUS:
	{
		// double left_rps = drive_planner_output_msg.left_motor_output_rad_per_sec;
		// double left_ff_voltage = drive_planner_output_msg.left_motor_feedforward_voltage;
		// double left_accel_rps2 = drive_planner_output_msg.left_motor_accel_rad_per_sec2;

		// swerve_drivetrain_diagnostics.targetVelocityLeft = left_rps;
		// swerve_drivetrain_diagnostics.targetAccelLeft = left_accel_rps2;

		// double right_rps = drive_planner_output_msg.right_motor_output_rad_per_sec;
		// double right_ff_voltage = drive_planner_output_msg.right_motor_feedforward_voltage;
		// double right_accel_rps2 = drive_planner_output_msg.right_motor_accel_rad_per_sec2;

		// swerve_drivetrain_diagnostics.targetVelocityRight = right_rps;
		// swerve_drivetrain_diagnostics.targetAccelRight = right_accel_rps2;

		// double left_accel_out = ck::math::radians_per_second_to_ticks_per_100ms(left_accel_rps2, kDriveRotationsPerTick) / 1000.0;
		// double right_accel_out = ck::math::radians_per_second_to_ticks_per_100ms(right_accel_rps2, kDriveRotationsPerTick) / 1000.0;

		// // leftMasterMotor->set( Motor::Control_Mode::VELOCITY,
		// // 						ck::math::rads_per_sec_to_rpm(left_rps),
		// // 						left_ff_voltage / 12.0 + velocity_kD * left_accel_out / 1023.0
		// // 						);

		// // rightMasterMotor->set( Motor::Control_Mode::VELOCITY,
		// // 						ck::math::rads_per_sec_to_rpm(right_rps),
		// // 						right_ff_voltage / 12.0 + velocity_kD * right_accel_out / 1023.0
		// // 						);

		// leftMasterMotor->set( Motor::Control_Mode::VELOCITY,
		// 						ck::math::rads_per_sec_to_rpm(left_rps),
		// 						0
		// 						);

		// rightMasterMotor->set( Motor::Control_Mode::VELOCITY,
		// 						ck::math::rads_per_sec_to_rpm(right_rps),
		// 						0
		// 						);
		// swerve_drivetrain_diagnostics.leftAppliedArbFF = left_ff_voltage / 12.0 + velocity_kD * left_accel_out / 1023.0;
		// swerve_drivetrain_diagnostics.rightAppliedArbFF = right_ff_voltage / 12.0 + velocity_kD * right_accel_out / 1023.0;
		// swerve_drivetrain_diagnostics.rawLeftMotorOutput = ck::math::rads_per_sec_to_rpm(left_rps);
		// swerve_drivetrain_diagnostics.rawRightMotorOutput = ck::math::rads_per_sec_to_rpm(right_rps);
	}
    break;
	case rio_control_node::Robot_Status::TELEOP:
	{
		ROS_INFO("Testy test\n");
		ck::swerve::SwerveDriveOutput sdo;
		float shoot_multiplier = 1.0;
		if(about_to_shoot)
		{
			shoot_multiplier = 0.0;
			brake_mode = true;
		}

		switch (drive_control_config)
		{
			case DriveControlMode::SWERVE_VELOCITY_FEEDFORWARD:
			{
				ROS_INFO("Got into the switch\n");
				geometry_msgs::Twist twist = get_twist_from_input(msg.drivetrain_swerve_percent_fwd_vel, msg.drivetrain_swerve_direction, msg.drivetrain_swerve_percent_angular_rot);
				sdo = calculate_swerve_output_from_twist(twist);
				break;
			}

			// case DriveControlMode::VELOCITY_CURVATURE_DRIVE:
			// {
			// 	//Not Implemented
			// 	leftPre = 0;
			// 	rightPre = 0;
			// 	break;
			// }
		}

#ifdef CHARACTERIZE_DRIVE
		if (characterizing_drive)
		{
			left = characterizing_drive_left_output;
			right = characterizing_drive_right_output;
		}
#endif

#ifdef DYNAMIC_RECONFIGURE_TUNING
		switch (tuning_control_mode)
		{
			case DriveTuningMode::Normal:
			{
				leftMasterMotor->set( Motor::Control_Mode::PERCENT_OUTPUT, left, 0 );
				rightMasterMotor->set( Motor::Control_Mode::PERCENT_OUTPUT, right, 0 );
				break;
			}
			case DriveTuningMode::TuningVelocityPID:
			{
				swerve_drivetrain_diagnostics.tuningVelocityRPMTarget = tuning_velocity_target;
				leftMasterMotor->set( Motor::Control_Mode::VELOCITY, tuning_velocity_target, 0 );
				rightMasterMotor->set( Motor::Control_Mode::VELOCITY, tuning_velocity_target, 0 );
				break;
			}
			case DriveTuningMode::TuningkVkA:
			{
				swerve_drivetrain_diagnostics.tuningVelocityRPMTarget = tuning_velocity_target;

				//Start?? 0.001852534562
				leftMasterMotor->set( Motor::Control_Mode::PERCENT_OUTPUT, (drive_Kv * tuning_velocity_target) / 12.0, 0 );
				rightMasterMotor->set( Motor::Control_Mode::PERCENT_OUTPUT, (drive_Kv * tuning_velocity_target) /12.0, 0 );
				break;
			}
		}
#else
		ROS_INFO("Setting Motors");
		for (int i = 0; i < robot_num_wheels; i++)
		{
			drive_motors[i]->set( Motor::Control_Mode::PERCENT_OUTPUT, shoot_multiplier * sdo.wheels[i].velocity * drive_velocity_kF, 0 );
			steering_motors[i]->set( Motor::Control_Mode::MOTION_MAGIC, sdo.wheels[i].angle, 0 );
		}
		ROS_INFO("Set Motors");

        // leftMasterMotor->set( Motor::Control_Mode::PERCENT_OUTPUT, left, 0 );
		// rightMasterMotor->set( Motor::Control_Mode::PERCENT_OUTPUT, right, 0 );
#endif
	}
	break;
	default:
	{
		for (Motor* mF : drive_motors)
		{
			mF->set( Motor::Control_Mode::PERCENT_OUTPUT, 0, 0 );
		}
	}
	break;
	}

	if (brake_mode)
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
	ROS_INFO("Set BrakeCoast");
	static ros::Publisher swerve_drivetrain_diagnostics_publisher = node->advertise<swerve_drivetrain_node::Swerve_Drivetrain_Diagnostics>("/DrivetrainDiagnostics", 1);
	swerve_drivetrain_diagnostics_publisher.publish(swerve_drivetrain_diagnostics);
	ROS_INFO("End of HMI Callback");
}


void initMotors()
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
		drive_motor->config().set_kP(drive_velocity_kP);
		drive_motor->config().set_kI(drive_velocity_kI);
		drive_motor->config().set_kD(drive_velocity_kD);
		drive_motor->config().set_kF(drive_velocity_kF);
		drive_motor->config().set_i_zone(drive_velocity_iZone);
		drive_motor->config().set_max_i_accum(drive_velocity_maxIAccum);
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
		steering_motor->config().set_i_zone(steering_velocity_iZone);
		steering_motor->config().set_max_i_accum(steering_velocity_maxIAccum);
		steering_motor->config().set_closed_loop_ramp(steering_closed_loop_ramp);
		steering_motor->config().apply();
		steering_motors.push_back(steering_motor);
    }
}

int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "drivetrain");

	ros::NodeHandle n;

	node = &n;

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
	required_params_found &= n.getParam(CKSP(robot_track_width_inches), robot_track_width_inches);
	required_params_found &= n.getParam(CKSP(robot_track_length_inches), robot_track_length_inches);
	required_params_found &= n.getParam(CKSP(robot_linear_inertia), robot_linear_inertia);
	required_params_found &= n.getParam(CKSP(robot_angular_inertia), robot_angular_inertia);
	required_params_found &= n.getParam(CKSP(robot_angular_drag), robot_angular_drag);
	required_params_found &= n.getParam(CKSP(robot_scrub_factor), robot_scrub_factor);
	required_params_found &= n.getParam(CKSP(drive_Ks_v_intercept), drive_Ks_v_intercept);
	required_params_found &= n.getParam(CKSP(drive_Kv), drive_Kv);
	required_params_found &= n.getParam(CKSP(drive_Ka), drive_Ka);
	required_params_found &= n.getParam(CKSP(drive_velocity_kP), drive_velocity_kP);
	required_params_found &= n.getParam(CKSP(drive_velocity_kI), drive_velocity_kI);
	required_params_found &= n.getParam(CKSP(drive_velocity_kD), drive_velocity_kD);
	required_params_found &= n.getParam(CKSP(drive_velocity_kF), drive_velocity_kF);
	required_params_found &= n.getParam(CKSP(drive_velocity_iZone), drive_velocity_iZone);
	required_params_found &= n.getParam(CKSP(drive_velocity_maxIAccum), drive_velocity_maxIAccum);
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
	required_params_found &= n.getParam(CKSP(joystick_input_ramp_accel), joystick_input_ramp_accel);
	required_params_found &= n.getParam(CKSP(joystick_input_ramp_decel), joystick_input_ramp_decel);
	required_params_found &= n.getParam(CKSP(joystick_input_ramp_zero_val), joystick_input_ramp_zero_val);
	required_params_found &= n.getParam(CKSP(joystick_input_ramp_max_val), joystick_input_ramp_max_val);
	required_params_found &= n.getParam(CKSP(drive_control_mode), drive_control_mode);

	if (!required_params_found)
	{
		ROS_ERROR("Missing required parameters for node %s. Please check the list and make sure all required parameters are included", ros::this_node::getName().c_str());
		return 1;
	}

	drive_control_config = (DriveControlMode)drive_control_mode;

	ros::Subscriber joystickStatus = node->subscribe("/HMISignals", 1, hmiSignalsCallback);
	ros::Subscriber motorStatus = node->subscribe("MotorStatus", 1, motorStatusCallback);
	ros::Subscriber robotStatus = node->subscribe("RobotStatus", 1, robotStatusCallback);
	ros::Subscriber planner_sub = node->subscribe("/QuesadillaPlannerOutput", 1, planner_callback);
	// ros::Subscriber turret_status_subscriber = node->subscribe("/TurretStatus", 1, turret_status_callback);
#ifdef CHARACTERIZE_DRIVE
	ros::Subscriber drive_characterization_subscriber = node->subscribe("/DriveCharacterizationOutput", 1, drive_characterization_callback);
#endif

    initMotors();

#ifdef DYNAMIC_RECONFIGURE_TUNING
	dynamic_reconfigure::Server<drivetrain_node::DriveTuningConfig> server;
	dynamic_reconfigure::Server<drivetrain_node::DriveTuningConfig>::CallbackType f;
	f = boost::bind(&tuning_config_callback, _1, _2);
	server.setCallback(f);
#endif

	//Init swerve configuration
	for (int i = 0; i < robot_num_wheels; i++)
	{
		ck::swerve::WheelConfig wheel;
		geometry_msgs::Transform wheel_transform;
		wheel_transform.translation.x = ck::math::inches_to_meters(robot_wheel_inches_from_center_x[i]);
		wheel_transform.translation.y = ck::math::inches_to_meters(robot_wheel_inches_from_center_y[i]);
		wheel.transform = wheel_transform;
		swerve_drive_config.wheels.push_back(wheel);
	}

	ros::spin();
	return 0;
}
