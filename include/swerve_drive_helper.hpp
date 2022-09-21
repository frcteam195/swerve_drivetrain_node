#pragma once

#include "ck_utilities/CKMath.hpp"
#include "ck_utilities/CKMath_ros.hpp"
#include <cmath> 
#include <vector>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

struct DriveMotorValues
{
    double left;
    double right;
};

class DriveHelper
{
private:
    // These factor determine how fast the wheel traverses the "non linear" sine curve.
    static constexpr double kHighWheelNonLinearity = 0.65;
    static constexpr double kLowWheelNonLinearity = 0.5;

    static constexpr double kHighNegInertiaScalar = 4.0;

    static constexpr double kLowNegInertiaThreshold = 0.65;
    static constexpr double kLowNegInertiaTurnScalar = 3.5;
    static constexpr double kLowNegInertiaCloseScalar = 4.0;
    static constexpr double kLowNegInertiaFarScalar = 5.0;

    static constexpr double kHighSensitivity = 0.95;
    static constexpr double kLowSensitiity = 1.3;

    static constexpr double kQuickStopDeadband = 0.2;
    static constexpr double kQuickStopWeight = 0.1;
    static constexpr double kQuickStopScalar = 5.0;

    double mOldWheel = 0.0;
    double mQuickStopAccumlator = 0.0;
    double mNegInertiaAccumlator = 0.0;
public:

    DriveMotorValues calculateOutput(double throttle, double wheel, bool isQuickTurn, bool isHighGear);
    static double smallest_traversal(double angle, double target_angle);
    static std::vector<std::pair<geometry_msgs::Pose, geometry_msgs::Twist>> calculate_swerve_outputs
        (geometry_msgs::Twist desired_twist,
        std::vector<geometry_msgs::Transform> wheel_transforms,
        double projection_time_s);
};