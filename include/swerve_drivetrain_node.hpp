#pragma once
#include <vector>
#include <stdint.h>

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