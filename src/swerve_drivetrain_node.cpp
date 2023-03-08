#include <ros/ros.h>
#include <string>
#include <map>
#include <iostream>

#include <nav_msgs/Odometry.h>
#include <ck_ros_base_msgs_node/Joystick_Status.h>
#include <ck_ros_base_msgs_node/Robot_Status.h>
#include <ck_ros_base_msgs_node/Motor_Status.h>
#include <ck_utilities/Logger.hpp>
#include <ck_utilities/geometry/geometry.hpp>
#include <ck_utilities/geometry/geometry_ros_helpers.hpp>
#include <ck_utilities/CKMath.hpp>
#include <ck_ros_msgs_node/Swerve_Drivetrain_Diagnostics.h>
#include <ck_ros_msgs_node/HMI_Signals.h>
#include <swerve_trajectory_node/StartTrajectory.h>

#include "swerve_drivetrain_node.hpp"

#include "swerve_drive_helper.hpp"
#include "motor_interface.hpp"
#include "odometry_interface.hpp"
#include "config_params.hpp"
#include "input_signal_processor.hpp"
#include <std_msgs/Float32.h>

ros::NodeHandle* node;

std::vector<geometry::Transform> wheel_transforms;

ck_ros_msgs_node::Swerve_Drivetrain_Diagnostics swerve_drivetrain_diagnostics;

std::map<uint16_t, ck_ros_base_msgs_node::Motor_Info> motor_map;
ck_ros_base_msgs_node::Robot_Status robot_status;
ck_ros_msgs_node::HMI_Signals hmi_signals;
ck_ros_msgs_node::Swerve_Drivetrain_Auto_Control auto_control;

static ros::Publisher * diagnostics_publisher;
ck_ros_msgs_node::Swerve_Drivetrain_Diagnostics drivetrain_diagnostics;

bool run_once = true;

void update_motor_transforms()
{
	//Update swerve steering transforms
	for (int i = 0; i < config_params::robot_num_wheels; i++)
	{
		geometry::Rotation rotation;
		rotation.yaw(ck::math::normalize_to_2_pi(ck::math::deg2rad(motor_map[config_params::steering_motor_ids[i]].sensor_position * 360.0)));
		wheel_transforms[i].angular = rotation;
	}
}

void kill_motors()
{
    for (Motor* mF : drive_motors)
    {
        mF->set( Motor::Control_Mode::PERCENT_OUTPUT, 0, 0 );
    }
    for (Motor* mS : steering_motors)
    {
        mS->set( Motor::Control_Mode::PERCENT_OUTPUT, 0, 0 );
    }

	for (size_t i = 0; i < drive_motors.size(); i++)
	{
		drivetrain_diagnostics.modules[i].target_steering_angle_deg = ck::math::rad2deg(ck::math::normalize_to_2_pi(motor_map[config_params::steering_motor_ids[i]].sensor_position * 2.0 * M_PI));
		drivetrain_diagnostics.modules[i].actual_steering_angle_deg = ck::math::rad2deg(ck::math::normalize_to_2_pi(motor_map[config_params::steering_motor_ids[i]].sensor_position * 2.0 * M_PI));
		drivetrain_diagnostics.modules[i].target_speed_m_s = 0;
		drivetrain_diagnostics.modules[i].actual_speed_m_s = motor_map[config_params::drive_motor_ids[i]].sensor_velocity * (0.1016 * M_PI) / 60.0;
	}
}

void apply_robot_twist(geometry::Twist desired_twist, bool useDeadband=true)
{
	double linear_deadband = 0.2;
	double angular_deadband = 20;

	if (!useDeadband)
	{
		linear_deadband = 0.1;
		angular_deadband = 10;
	}

	if (std::abs(desired_twist.linear.norm()) > linear_deadband ||
        std::abs(desired_twist.angular.yaw()) > ck::math::deg2rad(angular_deadband))
    {
        std::vector<std::pair<geometry::Pose, geometry::Twist>> sdo = calculate_swerve_outputs(desired_twist, wheel_transforms);
        set_swerve_output(sdo);
    }
    else
    {
        kill_motors();
    }
}

void publish_diagnostic_data()
{
	diagnostics_publisher->publish(drivetrain_diagnostics);
}

void apply_x_mode()
{
    std::vector<std::pair<geometry::Pose, geometry::Twist>> sdo;
    float heading = 45;
    for (int i = 0; i < 4; i++)
    {
        geometry::Twist empty_twist;
        geometry::Pose pose;
        pose.orientation.yaw(ck::math::deg2rad(heading));
        std::pair<geometry::Pose, geometry::Twist> module(pose, empty_twist);
        sdo.push_back(module);
        heading -= 90;
    }
    set_swerve_output(sdo);
    return;
}

void process_swerve_logic()
{
	static int8_t frame_counter = 0;
	frame_counter ++;
	frame_counter = frame_counter % 100;

	update_motor_transforms();

	if(frame_counter % 20 == 0)
	{
		publish_motor_links();
	}

    if (frame_counter % 5 == 0)
    {
        publish_robot_base_tf();
    }

	geometry::Twist desired_robot_twist;

	switch (robot_status.robot_state)
	{
		case ck_ros_base_msgs_node::Robot_Status::AUTONOMOUS:
		{
			set_brake_mode(true);
			desired_robot_twist = get_twist_from_auto();
            if (auto_control.x_mode)
            {
                apply_x_mode();
            }
            else
            {
	            apply_robot_twist(desired_robot_twist, false);
            }
		}
		break;
		case ck_ros_base_msgs_node::Robot_Status::TELEOP:
		{
			set_brake_mode(hmi_signals.drivetrain_brake);
			desired_robot_twist = get_twist_from_HMI();
            apply_robot_twist(desired_robot_twist);
		}
		break;
		default:
		{
			run_once = true;
            kill_motors();
		}
		break;
	}

	geometry::Transform robot_pose = get_robot_transform();
	drivetrain_diagnostics.actual_heading = ck::math::rad2deg(robot_pose.angular.yaw());

	publishOdometryData();
	publish_diagnostic_data();
}

void motor_status_callback(const ck_ros_base_msgs_node::Motor_Status& motor_status_)
{
	std::map<uint16_t, ck_ros_base_msgs_node::Motor_Info> receipt_map;
	for(auto &i : motor_status_.motors)
	{
		receipt_map[i.id] = i;
	}

	motor_map = receipt_map;

	process_swerve_logic();
}

void robot_status_callback(const ck_ros_base_msgs_node::Robot_Status& robot_status_)
{
	robot_status = robot_status_;
}

void hmi_signals_callback(const ck_ros_msgs_node::HMI_Signals& hmi_signals_)
{
	hmi_signals = hmi_signals_;
}

void auto_control_callback(const ck_ros_msgs_node::Swerve_Drivetrain_Auto_Control& auto_control_)
{
	auto_control = auto_control_;
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
	static ros::Subscriber auto_signals_subscriber = node->subscribe("/SwerveAutoControl", 1, auto_control_callback, ros::TransportHints().tcpNoDelay());
	ros::Publisher diagnostics_publisher_ = node->advertise<ck_ros_msgs_node::Swerve_Drivetrain_Diagnostics>("/SwerveDiagnostics", 10);
	diagnostics_publisher = &diagnostics_publisher_;
	ros::spin();
	return 0;
}
