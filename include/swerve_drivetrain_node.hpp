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

double gear_ratio_motor_to_output_shaft;
double wheel_diameter_inches;
double robot_track_width_inches;
double robot_track_length_inches;
double robot_linear_inertia;
double robot_angular_inertia;
double robot_angular_drag;
double robot_scrub_factor;

double drive_Ks_v_intercept;
double drive_Kv;
double drive_Ka;

double drive_velocity_kP;
double drive_velocity_kI;
double drive_velocity_kD;
double drive_velocity_kF;
double drive_velocity_iZone;
double drive_velocity_maxIAccum;
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

double joystick_input_ramp_accel;
double joystick_input_ramp_decel;
double joystick_input_ramp_zero_val;
double joystick_input_ramp_max_val;

int drive_control_mode;