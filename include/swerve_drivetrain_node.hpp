#pragma once
#include <map>
#include <rio_control_node/Motor_Info.h>
#include <rio_control_node/Robot_Status.h>
#include <ck_ros_msgs_node/HMI_Signals.h>
#include <ck_utilities/geometry/geometry.hpp>
#include <ros/ros.h>

extern std::map<uint16_t, rio_control_node::Motor_Info>& motor_map;
extern rio_control_node::Robot_Status robot_status;
extern ck_ros_msgs_node::HMI_Signals hmi_signals;
extern std::vector<geometry::Transform> wheel_transforms;
extern ros::NodeHandle* node;