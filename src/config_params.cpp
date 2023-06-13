#include "config_params.hpp"
#include <ck_utilities/ParameterHelper.hpp>

namespace config_params
{
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
    double drive_velocity_kP;
    double drive_velocity_kI;
    double drive_velocity_kD;
    double drive_velocity_kF;
    double drive_velocity_iZone;
    double drive_velocity_maxIAccum;

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
	double drive_supply_current_limit;
	double drive_supply_current_limit_threshold;
	double drive_supply_current_limit_threshold_exceeded_time;
	double steering_supply_current_limit;
	double steering_supply_current_limit_threshold;
	double steering_supply_current_limit_threshold_exceeded_time;

    double robot_max_fwd_accel;
    double quattro_decel;
    double robot_teleop_max_fwd_vel;

	DriveControlMode drive_control_mode;

	bool init_params(ros::NodeHandle &n)
	{
		bool required_params_found = true;

		required_params_found &= n.getParam(CKSP(robot_num_wheels), robot_num_wheels);
		required_params_found &= robot_num_wheels > 0;
		required_params_found &= n.getParam(CKSP(drive_motor_ids), drive_motor_ids);
		required_params_found &= n.getParam(CKSP(steering_motor_ids), steering_motor_ids);
		required_params_found &= n.getParam(CKSP(drive_motor_inverted), drive_motor_inverted);
		required_params_found &= n.getParam(CKSP(drive_sensor_inverted), drive_sensor_inverted);
		required_params_found &= n.getParam(CKSP(steering_motor_inverted), steering_motor_inverted);
		required_params_found &= n.getParam(CKSP(steering_sensor_inverted), steering_sensor_inverted);
		required_params_found &= n.getParam(CKSP(robot_wheel_inches_from_center_x), robot_wheel_inches_from_center_x);
		required_params_found &= n.getParam(CKSP(robot_wheel_inches_from_center_y), robot_wheel_inches_from_center_y);
		required_params_found &= drive_motor_ids.size() == (size_t)robot_num_wheels;
		required_params_found &= steering_motor_ids.size() == (size_t)robot_num_wheels;
		required_params_found &= drive_motor_inverted.size() == (size_t)robot_num_wheels;
		required_params_found &= drive_sensor_inverted.size() == (size_t)robot_num_wheels;
		required_params_found &= steering_motor_inverted.size() == (size_t)robot_num_wheels;
		required_params_found &= steering_sensor_inverted.size() == (size_t)robot_num_wheels;
		required_params_found &= robot_wheel_inches_from_center_x.size() == (size_t)robot_num_wheels;
		required_params_found &= robot_wheel_inches_from_center_y.size() == (size_t)robot_num_wheels;
		required_params_found &= n.getParam(CKSP(robot_max_fwd_vel), robot_max_fwd_vel);
		required_params_found &= n.getParam(CKSP(robot_max_ang_vel), robot_max_ang_vel);
		required_params_found &= n.getParam(CKSP(motor_type), motor_type);
		required_params_found &= n.getParam(CKSP(voltage_comp_saturation), voltage_comp_saturation);
		required_params_found &= n.getParam(CKSP(voltage_comp_enabled), voltage_comp_enabled);
		required_params_found &= n.getParam(CKSP(brake_mode_default), brake_mode_default);
		required_params_found &= n.getParam(CKSP(wheel_diameter_inches), wheel_diameter_inches);
		required_params_found &= n.getParam(CKSP(drive_closed_loop_ramp), drive_closed_loop_ramp);
		required_params_found &= n.getParam(CKSP(drive_motion_cruise_velocity), drive_motion_cruise_velocity);
		required_params_found &= n.getParam(CKSP(drive_motion_accel), drive_motion_accel);
		required_params_found &= n.getParam(CKSP(drive_motion_s_curve_strength), drive_motion_s_curve_strength);
		required_params_found &= n.getParam(CKSP(drive_velocity_kP),drive_velocity_kP);
		required_params_found &= n.getParam(CKSP(drive_velocity_kI),drive_velocity_kI);
		required_params_found &= n.getParam(CKSP(drive_velocity_kD),drive_velocity_kD);
		required_params_found &= n.getParam(CKSP(drive_velocity_kF),drive_velocity_kF);
		required_params_found &= n.getParam(CKSP(drive_velocity_iZone),drive_velocity_iZone);
		required_params_found &= n.getParam(CKSP(drive_velocity_maxIAccum),drive_velocity_maxIAccum);
		required_params_found &= n.getParam(CKSP(steering_velocity_kP), steering_velocity_kP);
		required_params_found &= n.getParam(CKSP(steering_velocity_kI), steering_velocity_kI);
		required_params_found &= n.getParam(CKSP(steering_velocity_kD), steering_velocity_kD);
		required_params_found &= n.getParam(CKSP(steering_velocity_kF), steering_velocity_kF);
		required_params_found &= n.getParam(CKSP(steering_velocity_iZone), steering_velocity_iZone);
		required_params_found &= n.getParam(CKSP(steering_velocity_maxIAccum), steering_velocity_maxIAccum);
		required_params_found &= n.getParam(CKSP(steering_closed_loop_ramp), steering_closed_loop_ramp);
		required_params_found &= n.getParam(CKSP(steering_motion_cruise_velocity), steering_motion_cruise_velocity);
		required_params_found &= n.getParam(CKSP(steering_motion_accel), steering_motion_accel);
		required_params_found &= n.getParam(CKSP(steering_motion_s_curve_strength), steering_motion_s_curve_strength);
		required_params_found &= n.getParam(CKSP(open_loop_ramp), open_loop_ramp);
		required_params_found &= n.getParam(CKSP(drive_supply_current_limit), drive_supply_current_limit);
		required_params_found &= n.getParam(CKSP(drive_supply_current_limit_threshold), drive_supply_current_limit_threshold);
		required_params_found &= n.getParam(CKSP(drive_supply_current_limit_threshold_exceeded_time), drive_supply_current_limit_threshold_exceeded_time);
		required_params_found &= n.getParam(CKSP(steering_supply_current_limit), steering_supply_current_limit);
		required_params_found &= n.getParam(CKSP(steering_supply_current_limit_threshold), steering_supply_current_limit_threshold);
		required_params_found &= n.getParam(CKSP(steering_supply_current_limit_threshold_exceeded_time), steering_supply_current_limit_threshold_exceeded_time);
        required_params_found &= n.getParam(CKSP(robot_max_fwd_accel), robot_max_fwd_accel);
        required_params_found &= n.getParam(CKSP(quattro_decel), quattro_decel);
        required_params_found &= n.getParam(CKSP(robot_teleop_max_fwd_vel), robot_teleop_max_fwd_vel);
		int tmp_drive_control_mode = 0;
        required_params_found &= n.getParam(CKSP(drive_control_mode), tmp_drive_control_mode);
		drive_control_mode = (DriveControlMode)tmp_drive_control_mode;


		if (!required_params_found)
		{
			ROS_ERROR("Missing required parameters for node %s. Please check the list and make sure all required parameters are included", ros::this_node::getName().c_str());
			return false;
		}

		return true;
	}
}