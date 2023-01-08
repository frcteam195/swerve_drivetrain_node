#include <ck_utilities/Motor.hpp>
#include <ck_utilities/geometry/geometry.hpp>
#include <ros/ros.h>

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
extern double supply_current_limit;
extern double supply_current_limit_threshold;
extern double supply_current_limit_threshold_exceeded_time;

extern std::vector<Motor*> drive_motors;
extern std::vector<Motor*> steering_motors;

bool init_params(ros::NodeHandle &n);

void set_swerve_output(std::vector<std::pair<geometry::Pose, geometry::Twist>> sdo);

void set_brake_mode(bool brake);

void init_swerve_motors();