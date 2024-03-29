#pragma once

#include <ros/ros.h>
#include <vector>

namespace config_params
{
    extern int robot_num_wheels;
    extern std::vector<int> drive_motor_ids;
    extern std::vector<int> steering_motor_ids;
    extern std::vector<bool> drive_motor_inverted;
    extern std::vector<bool> drive_sensor_inverted;
    extern std::vector<bool> steering_motor_inverted;
    extern std::vector<bool> steering_sensor_inverted;
    extern std::vector<double> robot_wheel_inches_from_center_x;
    extern std::vector<double> robot_wheel_inches_from_center_y;
    extern double robot_max_fwd_vel;
    extern double robot_max_ang_vel;

    extern int motor_type;
    extern double voltage_comp_saturation;
    extern bool voltage_comp_enabled;

    extern bool brake_mode_default;

    extern double wheel_diameter_inches;

    extern double drive_closed_loop_ramp;
    extern double drive_motion_cruise_velocity;
    extern double drive_motion_accel;
    extern double drive_motion_s_curve_strength;
    extern double drive_velocity_kP;
    extern double drive_velocity_kI;
    extern double drive_velocity_kD;
    extern double drive_velocity_kF;
    extern double drive_velocity_iZone;
    extern double drive_velocity_maxIAccum;

    extern double steering_velocity_kP;
    extern double steering_velocity_kI;
    extern double steering_velocity_kD;
    extern double steering_velocity_kF;
    extern double steering_velocity_iZone;
    extern double steering_velocity_maxIAccum;
    extern double steering_closed_loop_ramp;
    extern double steering_motion_cruise_velocity;
    extern double steering_motion_accel;
    extern double steering_motion_s_curve_strength;

    extern double open_loop_ramp;
    extern double drive_supply_current_limit;
    extern double drive_supply_current_limit_threshold;
    extern double drive_supply_current_limit_threshold_exceeded_time;
    extern double steering_supply_current_limit;
    extern double steering_supply_current_limit_threshold;
    extern double steering_supply_current_limit_threshold_exceeded_time;

    extern double robot_max_fwd_accel;
    extern double quattro_decel;
    extern double robot_teleop_max_fwd_vel;

    enum class DriveControlMode : int
    {
        SWERVE_VELOCITY_FF = 0,
        TEAM254_SETPOINT_GEN = 1
    };

    extern DriveControlMode drive_control_mode;

    bool init_params(ros::NodeHandle &n);
}