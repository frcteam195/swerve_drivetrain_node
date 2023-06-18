#pragma once

#include <ck_utilities/Motor.hpp>
#include <ck_utilities/geometry/geometry.hpp>
#include <ros/ros.h>
#include <ck_utilities/team254_swerve/SwerveSetpoint.hpp>

extern std::vector<Motor*> drive_motors;
extern std::vector<Motor*> steering_motors;


void set_swerve_idle();

void set_swerve_output(std::vector<std::pair<geometry::Pose, geometry::Twist>> sdo);

void set_swerve_output_auto(std::vector<std::pair<geometry::Pose, geometry::Twist>> sdo);

void set_swerve_output_team254(const ck::team254_swerve::SwerveSetpoint& swerve_setpoint);

void set_brake_mode(bool brake);

void init_swerve_motors();