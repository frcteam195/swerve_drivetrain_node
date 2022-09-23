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
#include "ck_utilities/swerve/SwerveDriveConfig.hpp"

std::vector<std::pair<geometry_msgs::Pose, geometry_msgs::Twist>> calculate_swerve_outputs
    (geometry_msgs::Twist desired_twist,
    ck::swerve::SwerveDriveConfig& wheel_transforms,
    double projection_time_s);
