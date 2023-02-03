#pragma once

#include <cmath>
#include <vector>
#include <ck_utilities/geometry/geometry.hpp>

double smallest_traversal(double angle, double target_angle);

std::vector<std::pair<geometry::Pose, geometry::Twist>> calculate_swerve_outputs
    (geometry::Twist desired_twist,
    std::vector<geometry::Transform> wheel_transforms,
    float projection_time_s = 0.00001);