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
#include <ck_ros_msgs_node/Arm_Status.h>
#include <swerve_trajectory_node/StartTrajectory.h>

#include <ck_utilities/team254_swerve/SwerveSetpointGenerator.hpp>

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
ck_ros_msgs_node::Arm_Status arm_status;
ck_ros_msgs_node::Swerve_Drivetrain_Auto_Control auto_control;
static bool auto_control_valid;

static ros::Publisher * diagnostics_publisher;
ck_ros_msgs_node::Swerve_Drivetrain_Diagnostics drivetrain_diagnostics;

ck::team254_swerve::SwerveDriveKinematics* swerve_kinematics;
ck::team254_swerve::SwerveSetpointGenerator* swerve_setpoint_gen;
ck::team254_swerve::KinematicLimits swerve_kinematic_limits;

ck::team254_swerve::SwerveSetpoint swerve_setpoint(ck::planners::ChassisSpeeds(), std::vector<ck::team254_swerve::SwerveModuleState>{
    ck::team254_swerve::SwerveModuleState(),
    ck::team254_swerve::SwerveModuleState(),
    ck::team254_swerve::SwerveModuleState(),
    ck::team254_swerve::SwerveModuleState()
});


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
        drivetrain_diagnostics.modules[i].actual_speed_m_s = motor_map[config_params::drive_motor_ids[i]].sensor_velocity * (ck::math::inches_to_meters(config_params::wheel_diameter_inches)* M_PI) / 60.0;
    }
}

void apply_robot_twist(geometry::Twist desired_twist, bool useDeadband=true)
{
    double linear_deadband = 0.2;
    double angular_deadband = 20;

    if (!useDeadband)
    {
        linear_deadband = 0.0;
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
        set_swerve_idle();
    }
}

void apply_robot_twist_team254(geometry::Twist desired_twist)
{
    constexpr double ideal_dt = 0.01;
    static ros::Time prev_time = ros::Time::now();
    double dt = (ros::Time::now() - prev_time).toSec();
    if (dt != 0)
    {
        // TODO: Use this angle to lerp a better accel limit to prevent tipping
        double total_arm_angle = abs(arm_status.arm_base_angle + arm_status.arm_upper_angle);
        total_arm_angle = ck::math::limit(total_arm_angle, 0.0, 150.0);

        ck::planners::ChassisSpeeds desired_speeds(desired_twist.linear.x(), desired_twist.linear.y(), desired_twist.angular.yaw());

        ck::team254_geometry::Pose2d robot_pose_vel(desired_speeds.vxMetersPerSecond * ideal_dt,
                                                    desired_speeds.vyMetersPerSecond * ideal_dt,
                                                    ck::team254_geometry::Rotation2d::fromRadians(desired_speeds.omegaRadiansPerSecond * ideal_dt));
        ck::team254_geometry::Twist2d twist_vel = ck::team254_geometry::Pose2d::log(robot_pose_vel);
        ck::planners::ChassisSpeeds updated_chassis_speeds(twist_vel.dx / ideal_dt, twist_vel.dy / ideal_dt, twist_vel.dtheta / ideal_dt);
        swerve_setpoint = swerve_setpoint_gen->generateSetpoint(swerve_kinematic_limits, swerve_setpoint, updated_chassis_speeds, dt);  //TODO: If something is wrong, try changing to ideal_dt here

        set_swerve_output_team254(swerve_setpoint);
    }
}

void apply_robot_twist_auto(geometry::Twist desired_twist, bool useDeadband=true)
{
    double linear_deadband = 0.2;
    double angular_deadband = 20;

    if (!useDeadband)
    {
        linear_deadband = 0.0;
        angular_deadband = 10;
    }

    if (std::abs(desired_twist.linear.norm()) > linear_deadband ||
        std::abs(desired_twist.angular.yaw()) > ck::math::deg2rad(angular_deadband))
    {
        std::vector<std::pair<geometry::Pose, geometry::Twist>> sdo = calculate_swerve_outputs(desired_twist, wheel_transforms);
        set_swerve_output_auto(sdo);
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
        switch(i)
        {
            case 0:
            {
                pose.orientation.yaw(ck::math::deg2rad(45));
                break;
            }
            case 1:
            {
                pose.orientation.yaw(ck::math::deg2rad(315));
                break;
            }
            case 2:
            {
                pose.orientation.yaw(ck::math::deg2rad(315));
                break;
            }
            default:
            case 3:
            {
                pose.orientation.yaw(ck::math::deg2rad(45));
                break;
            }
        }
        std::pair<geometry::Pose, geometry::Twist> module(pose, empty_twist);
        sdo.push_back(module);
        heading -= 90;
    }
    set_swerve_output(sdo);
    return;
}

void process_swerve_logic()
{
    static int8_t prev_robot_mode = 3;
    static ros::Duration time_in_disabled;
    static ros::Time disabled_start_time = ros::Time::now();
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
            if (auto_control_valid)
            {
                desired_robot_twist = get_twist_from_auto();
                if (auto_control.x_mode)
                {
                    apply_x_mode();
                }
                else
                {
                    switch (config_params::drive_control_mode)
                    {
                        case config_params::DriveControlMode::TEAM254_SETPOINT_GEN:
                        {
                            apply_robot_twist_team254(desired_robot_twist);
                            break;
                        }
                        case config_params::DriveControlMode::SWERVE_VELOCITY_FF:
                        default:
                        {
                            apply_robot_twist_auto(desired_robot_twist, false);
                            break;
                        }
                    };
                    
                }
            }
            else
            {
                kill_motors();
            }
        }
        break;
        case ck_ros_base_msgs_node::Robot_Status::TELEOP:
        {
            set_brake_mode(hmi_signals.drivetrain_brake);
            desired_robot_twist = get_twist_from_HMI();
            if (hmi_signals.drivetrain_xmode)
            {
                apply_x_mode();
            }
            else
            {
                apply_robot_twist_auto(desired_robot_twist);
            }
        }
        break;
        case ck_ros_base_msgs_node::Robot_Status::DISABLED:
        case ck_ros_base_msgs_node::Robot_Status::TEST:
        default:
        {
            if (prev_robot_mode != ck_ros_base_msgs_node::Robot_Status::DISABLED)
            {
                time_in_disabled = ros::Duration(0);
                disabled_start_time = ros::Time::now();
            }
            time_in_disabled = ros::Time::now() - disabled_start_time;

            if (time_in_disabled.toSec() > 15)
            {
                set_brake_mode(false);
            }

            auto_control_valid = false;
            run_once = true;
            set_swerve_idle();
        }
        break;
    }
    prev_robot_mode = robot_status.robot_state;
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
    auto_control_valid = true;
}

void arm_status_callback(const ck_ros_msgs_node::Arm_Status& arm_status_)
{
    arm_status = arm_status_;
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

    swerve_setpoint_gen = new ck::team254_swerve::SwerveSetpointGenerator(*swerve_kinematics);

    static ros::Subscriber motor_status_subscriber = node->subscribe("/MotorStatus", 1, motor_status_callback, ros::TransportHints().tcpNoDelay());
    static ros::Subscriber robot_status_subscriber = node->subscribe("/RobotStatus", 1, robot_status_callback, ros::TransportHints().tcpNoDelay());
    static ros::Subscriber hmi_signals_subscriber = node->subscribe("/HMISignals", 1, hmi_signals_callback, ros::TransportHints().tcpNoDelay());
    static ros::Subscriber auto_signals_subscriber = node->subscribe("/SwerveAutoControl", 1, auto_control_callback, ros::TransportHints().tcpNoDelay());
    static ros::Subscriber arm_status_subscriber = node->subscribe("/ArmStatus", 1, arm_status_callback, ros::TransportHints().tcpNoDelay());
    ros::Publisher diagnostics_publisher_ = node->advertise<ck_ros_msgs_node::Swerve_Drivetrain_Diagnostics>("/SwerveDiagnostics", 10);
    diagnostics_publisher = &diagnostics_publisher_;
    ros::spin();
    return 0;
}
