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
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
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


ros::NodeHandle* node;

float mJoystick1x;
float mJoystick1y;
std::mutex mThreadCtrlLock;
uint32_t mConfigUpdateCounter;

std::vector<geometry::Transform> wheel_transforms;

ck_ros_msgs_node::Swerve_Drivetrain_Diagnostics swerve_drivetrain_diagnostics;

tf2_ros::TransformBroadcaster * tfBroadcaster;
tf2_ros::TransformListener *tfListener;
tf2_ros::Buffer tfBuffer;

std::map<uint16_t, rio_control_node::Motor_Info>& motor_map;
rio_control_node::Robot_Status robot_status;
ck_ros_msgs_node::HMI_Signals hmi_signals;

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

geometry::Twist get_twist_from_input()
{
	bool field_orient = true;

	double percent_max_fwd_vel = hmi_signals.drivetrain_swerve_percent_fwd_vel;
	double direction = hmi_signals.drivetrain_swerve_direction;
	double percent_max_ang_vel = hmi_signals.drivetrain_swerve_percent_angular_rot;

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

void publish_motor_links()
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
	for (int i = 0; i < robot_num_wheels; i++)
	{
		geometry::Rotation rotation;
		rotation.yaw(ck::math::normalize_to_2_pi(ck::math::deg2rad(motor_map[steering_motor_ids[i]].sensor_position * 360.0)));
		wheel_transforms[i].angular = rotation;
		swerve_drivetrain_diagnostics.actual_motor_rotation.push_back(rotation.yaw());
	}

	publish_motor_links();
}


void apply_robot_twist(geometry::Twist desired_twist)
{
	std::vector<std::pair<geometry::Pose, geometry::Twist>> sdo = calculate_swerve_outputs(desired_twist, wheel_transforms, 0.01);
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

	geometry::Twist desired_robot_twist;

	switch (robot_status.robot_state)
	{
		case rio_control_node::Robot_Status::AUTONOMOUS:
		{
			set_brake_mode(true);
		}
		break;
		case rio_control_node::Robot_Status::TELEOP:
		{
			set_brake_mode(hmi_signals.drivetrain_brake);
			desired_robot_twist = get_twist_from_input();
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
	if (!init_params(n))
	{
		ROS_ERROR("NOT ALL SWERVE PARAMETERS SET - NODE TERMINATING");
		return 1;
	}

	init_swerve_motors();

	tfBroadcaster = new tf2_ros::TransformBroadcaster();
    tfListener = new tf2_ros::TransformListener(tfBuffer);

	static ros::Subscriber motor_status_subscriber = node->subscribe("/MotorStatus", 1, motor_status_callback, ros::TransportHints().tcpNoDelay());
	static ros::Subscriber robot_status_subscriber = node->subscribe("/RobotStatus", 1, robot_status_callback, ros::TransportHints().tcpNoDelay());
	static ros::Subscriber hmi_signals_subscriber = node->subscribe("/HMISignals", 1, hmi_signals_callback, ros::TransportHints().tcpNoDelay());

	ros::spin();
	return 0;
}
