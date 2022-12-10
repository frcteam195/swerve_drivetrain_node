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
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ck_utilities/CKMath.hpp>
#include <ck_utilities/Motor.hpp>
#include <ck_utilities/ParameterHelper.hpp>
#include <ck_utilities/ValueRamper.hpp>
#include <ck_utilities/MovingAverage.hpp>
#include <ck_utilities/geometry/geometry.hpp>
#include <ck_utilities/geometry/geometry_ros_helpers.hpp>

#include "swerve_drive_helper.hpp"
#include "swerve_drivetrain_node/Swerve_Drivetrain_Diagnostics.h"
#include <ck_utilities/geometry/geometry.hpp>
#include <frc_robot_utilities/frc_robot_utilities.hpp>


ros::NodeHandle* node;

float mJoystick1x;
float mJoystick1y;
std::mutex mThreadCtrlLock;
uint32_t mConfigUpdateCounter;

std::vector<Motor*> drive_motors;
std::vector<Motor*> steering_motors;
std::vector<geometry::Transform> wheel_transforms;

swerve_drivetrain_node::Swerve_Drivetrain_Diagnostics swerve_drivetrain_diagnostics;

tf2_ros::TransformBroadcaster * tfBroadcaster;
tf2_ros::TransformListener *tfListener;
tf2_ros::Buffer tfBuffer;

geometry::Transform get_robot_transform()
{
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

geometry::Twist get_twist_from_input(double percent_max_fwd_vel, double direction, double percent_max_ang_vel)
{
	bool field_orient = true;

	geometry::Twist return_twist;

	return_twist.linear.x(percent_max_fwd_vel * std::cos(direction) * robot_max_fwd_vel);
	return_twist.linear.y(percent_max_fwd_vel * std::sin(direction) * robot_max_fwd_vel);
	return_twist.angular.yaw(percent_max_ang_vel * robot_max_ang_vel);

	if(field_orient)
	{
		geometry::Rotation robot_inverse_rotation(-get_robot_transform().angular);
		return_twist.linear = return_twist.linear.Rotate(robot_inverse_rotation);
	}

	return return_twist;
}

void publishOdometryData(std::map<uint16_t, rio_control_node::Motor_Info>& motor_map)
{
	geometry::Translation wheel_vel_sum;
	for (int i = 0; i < robot_num_wheels; i++)
	{
		double curr_vel_m_s = motor_map[drive_motor_ids[i]].sensor_velocity * M_PI * ck::math::inches_to_meters(wheel_diameter_inches) / 60.0;
		double curr_angle = ck::math::normalize_to_2_pi(ck::math::deg2rad(motor_map[steering_motor_ids[i]].sensor_position * 360.0));
		geometry::Translation wheel_vel;
		wheel_vel.x(curr_vel_m_s * std::cos(curr_angle));
		wheel_vel.y(curr_vel_m_s * std::sin(curr_angle));
		wheel_vel_sum += wheel_vel;
	}
	wheel_vel_sum /= robot_num_wheels;

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

void publish_motor_links(std::map<uint16_t, rio_control_node::Motor_Info> motor_map)
{
	//Update swerve steering transforms
	for (int i = 0; i < robot_num_wheels; i++)
	{
		tf2::Quaternion quat_tf;
		quat_tf.setRPY(0, 0, ck::math::normalize_to_2_pi(ck::math::deg2rad(motor_map[(uint16_t)steering_motor_ids[i]].sensor_position * 360.0)));
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

static std::map<uint16_t, rio_control_node::Motor_Info> motor_map;

void update_motors(std::map<uint16_t, rio_control_node::Motor_Info>& motor_map)
{
	static ros::Time prev_time(0);
	publishOdometryData(motor_map);

	ros::Time curr_time = ros::Time::now();
	double dt = (curr_time - prev_time).toSec();



	swerve_drivetrain_diagnostics.dt = dt;
	prev_time = curr_time;
	swerve_drivetrain_diagnostics.actual_motor_rotation.clear();
	//Update swerve steering transforms
	for (int i = 0; i < robot_num_wheels; i++)
	{
		geometry::Rotation rotation;
		rotation.yaw(ck::math::normalize_to_2_pi(ck::math::deg2rad(motor_map[steering_motor_ids[i]].sensor_position * 360.0)));
		wheel_transforms[i].angular = rotation;
		swerve_drivetrain_diagnostics.actual_motor_rotation.push_back(rotation.yaw());
	}

	publish_motor_links(motor_map);
}

void init_swerve()
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

void set_swerve_output()
{
	geometry::Twist twist = get_twist_from_input(hmi_updates.get().drivetrain_swerve_percent_fwd_vel, hmi_updates.get().drivetrain_swerve_direction, hmi_updates.get().drivetrain_swerve_percent_angular_rot);
	std::vector<std::pair<geometry::Pose, geometry::Twist>> sdo = calculate_swerve_outputs(twist, wheel_transforms, 0.01);

	// std::stringstream s;
	// s << "-----------------------------------------------" << std::endl;
	// s << std::endl << "Motor Outputs:" << robot_num_wheels<<  std::endl;

	for (size_t i = 0; i < drive_motors.size(); i++)
	{
		constexpr double MAX_DRIVE_VEL_L1_FALCON = 6380.0 / 8.14 * (0.1016 * M_PI) / 60.0;
		drive_motors[i]->set( Motor::Control_Mode::PERCENT_OUTPUT, sdo[i].second.linear.x() / MAX_DRIVE_VEL_L1_FALCON, 0 );
		// s << "Speed %: " << sdo[i].second.linear.x() << std::endl;
		float delta = smallest_traversal(ck::math::normalize_to_2_pi(motor_map[steering_motor_ids[i]].sensor_position * 2.0 * M_PI), ck::math::normalize_to_2_pi(sdo[i].first.orientation.yaw()));
		float target = (motor_map[steering_motor_ids[i]].sensor_position * 2.0 * M_PI) + delta;
		steering_motors[i]->set( Motor::Control_Mode::MOTION_MAGIC, target / (2.0 * M_PI), 0 );
		// s << "Steer: " << sdo[i].first.orientation.yaw() << std::endl;
		// s << "--------" << std::endl;
	}
	// ROS_DEBUG("%s", s.str().c_str());
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "drivetrain");

	ros::NodeHandle n;

	node = &n;
	if (!init_params(n))
	{
		return 1;
	}

	init_swerve();

	tfBroadcaster = new tf2_ros::TransformBroadcaster();
    tfListener = new tf2_ros::TransformListener(tfBuffer);

	register_for_robot_updates(node);

	while (ros::ok())
	{
		ros::spinOnce();
		update_motors(motor_updates.get());

		bool brake_mode = hmi_updates.get().drivetrain_brake;
		set_brake_mode(brake_mode);

		switch (robot_status.get_mode())
		{
		case RobotMode::AUTONOMOUS:
		{

		}
		break;
		case RobotMode::TELEOP:
		{
			set_swerve_output();
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
		
		static ros::Publisher swerve_drivetrain_diagnostics_publisher = node->advertise<swerve_drivetrain_node::Swerve_Drivetrain_Diagnostics>("/DrivetrainDiagnostics", 1);
		swerve_drivetrain_diagnostics_publisher.publish(swerve_drivetrain_diagnostics);
	}
	return 0;
}
