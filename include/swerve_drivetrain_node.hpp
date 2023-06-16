#pragma once
#include <map>
#include <ck_ros_base_msgs_node/Motor_Info.h>
#include <ck_ros_base_msgs_node/Robot_Status.h>
#include <ck_ros_msgs_node/HMI_Signals.h>
#include <ck_ros_msgs_node/Swerve_Drivetrain_Auto_Control.h>
#include <ck_ros_msgs_node/Swerve_Drivetrain_Diagnostics.h>
#include <ck_utilities/geometry/geometry.hpp>
#include <ck_utilities/PIDController.hpp>
#include <ros/ros.h>
#include <ck_utilities/team254_swerve/SwerveSetpointGenerator.hpp>

extern std::map<uint16_t, ck_ros_base_msgs_node::Motor_Info> motor_map;
extern ck_ros_base_msgs_node::Robot_Status robot_status;
extern ck_ros_msgs_node::HMI_Signals hmi_signals;
extern ck_ros_msgs_node::Swerve_Drivetrain_Auto_Control auto_control;
extern std::vector<geometry::Transform> wheel_transforms;
extern ros::NodeHandle* node;
extern ck_ros_msgs_node::Swerve_Drivetrain_Diagnostics drivetrain_diagnostics;

extern ck::team254_swerve::SwerveDriveKinematics* swerve_kinematics;
extern ck::team254_swerve::SwerveSetpointGenerator* swerve_setpoint_gen;
extern ck::team254_swerve::KinematicLimits swerve_kinematic_limits;
extern ck::team254_swerve::SwerveSetpoint swerve_setpoint;