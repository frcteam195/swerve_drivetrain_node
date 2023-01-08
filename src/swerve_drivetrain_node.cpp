#include "swerve_drivetrain_node.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

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
#include <ck_utilities/CKMath.hpp>
#include <ck_utilities/Motor.hpp>
#include <ck_utilities/ValueRamper.hpp>
#include <ck_utilities/MovingAverage.hpp>
#include <ck_utilities/geometry/geometry.hpp>
#include <ck_utilities/geometry/geometry_ros_helpers.hpp>

#include "swerve_drive_helper.hpp"
#include "motor_interface.hpp"
#include "odometry_interface.hpp"
#include "ck_ros_msgs_node/Swerve_Drivetrain_Diagnostics.h"
#include "ck_ros_msgs_node/HMI_Signals.h"
#include <ck_utilities/geometry/geometry.hpp>
#include "config_params.hpp"
#include "input_signal_processor.hpp"


ros::NodeHandle* node;

float mJoystick1x;
float mJoystick1y;
std::mutex mThreadCtrlLock;
uint32_t mConfigUpdateCounter;

std::vector<geometry::Transform> wheel_transforms;

ck_ros_msgs_node::Swerve_Drivetrain_Diagnostics swerve_drivetrain_diagnostics;


std::map<uint16_t, rio_control_node::Motor_Info>& motor_map;
rio_control_node::Robot_Status robot_status;
ck_ros_msgs_node::HMI_Signals hmi_signals;

void update_motors()
{
	static ros::Time prev_time(0);
	publishOdometryData();

	ros::Time curr_time = ros::Time::now();
	double dt = (curr_time - prev_time).toSec();

	swerve_drivetrain_diagnostics.dt = dt;
	prev_time = curr_time;
	swerve_drivetrain_diagnostics.actual_motor_rotation.clear();
	//Update swerve steering transforms
	for (int i = 0; i < config_params::robot_num_wheels; i++)
	{
		geometry::Rotation rotation;
		rotation.yaw(ck::math::normalize_to_2_pi(ck::math::deg2rad(motor_map[config_params::steering_motor_ids[i]].sensor_position * 360.0)));
		wheel_transforms[i].angular = rotation;
		swerve_drivetrain_diagnostics.actual_motor_rotation.push_back(rotation.yaw());
	}
}


void apply_robot_twist(geometry::Twist desired_twist)
{
	std::vector<std::pair<geometry::Pose, geometry::Twist>> sdo = calculate_swerve_outputs(desired_twist, wheel_transforms, 0.01);
	set_swerve_output(sdo);
}

void process_swerve_logic()
{
	static int8_t frame_counter = 0;
	frame_counter ++;
	frame_counter = frame_counter % 20;

	if(frame_counter == 0)
	{
		publish_motor_links();
	}

	update_motors();

	geometry::Twist desired_robot_twist;

	switch (robot_status.robot_state)
	{
		case rio_control_node::Robot_Status::AUTONOMOUS:
		{
			set_brake_mode(true);
			desired_robot_twist = get_twist_from_auto();
		}
		break;
		case rio_control_node::Robot_Status::TELEOP:
		{
			set_brake_mode(hmi_signals.drivetrain_brake);
			desired_robot_twist = get_twist_from_HMI();
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

	apply_robot_twist(desired_robot_twist);
}

void motor_status_callback(const rio_control_node::Motor_Status& motor_status_)
{
	std::map<uint16_t, rio_control_node::Motor_Info> receipt_map;
	for(auto &i : motor_status_.motors)
	{
		receipt_map[i.id] = i;
	}
	motor_map = receipt_map;

	process_swerve_logic();
}

void robot_status_callback(const rio_control_node::Robot_Status& robot_status_)
{
	robot_status = robot_status_;
}

void hmi_signals_callback(const ck_ros_msgs_node::HMI_Signals& hmi_signals_)
{
	hmi_signals = hmi_signals_;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "drivetrain");

	ros::NodeHandle n;

	node = &n;
	if (!config_params::init_params(n))
	{
		ROS_ERROR("NOT ALL SWERVE PARAMETERS SET - NODE TERMINATING");
		return 1;
	}

	init_swerve_motors();

	static ros::Subscriber motor_status_subscriber = node->subscribe("/MotorStatus", 1, motor_status_callback, ros::TransportHints().tcpNoDelay());
	static ros::Subscriber robot_status_subscriber = node->subscribe("/RobotStatus", 1, robot_status_callback, ros::TransportHints().tcpNoDelay());
	static ros::Subscriber hmi_signals_subscriber = node->subscribe("/HMISignals", 1, hmi_signals_callback, ros::TransportHints().tcpNoDelay());

	ros::spin();
	return 0;
}
