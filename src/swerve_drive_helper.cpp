#include "swerve_drive_helper.hpp"
#include "config_params.hpp"
#include <ck_utilities/CKMath.hpp>
#include <sstream>
#include <ros/ros.h>
#include <iostream>

// #define PURE_MATH

double smallest_traversal(double angle, double target_angle)
{
    double left = -ck::math::normalize_to_2_pi(angle - target_angle);
    double right = ck::math::normalize_to_2_pi(target_angle - angle);
    if(fabs(left) < fabs(right))
    {
        return left;
    }
    return right;
}

std::ostream& operator<<(std::ostream& os, const std::vector<std::pair<geometry::Pose, geometry::Twist>>& value)
{
    std::stringstream s;
    for (std::vector<std::pair<geometry::Pose, geometry::Twist>>::const_iterator i = value.begin();
         i != value.end();
         i++)
    {
        s << "Pose: " << (*i).first << "Twist: " << (*i).second;
        s << "---" << std::endl;
    }
    os << s.str();
    return os;
}

std::ostream& operator<<(std::ostream& os, const std::vector<geometry::Transform>& value)
{
    std::stringstream s;
    for (std::vector<geometry::Transform>::const_iterator i = value.begin();
         i != value.end();
         i++)
    {
        s << (*i);
        s << "---" << std::endl;
    }
    os << s.str();
    return os;
}

#ifndef PURE_MATH

std::vector<std::pair<geometry::Pose, geometry::Twist>> calculate_swerve_outputs
    (geometry::Twist desired_twist,
    std::vector<geometry::Transform> wheel_transforms,
    float projection_time_s)
{
    std::vector<std::pair<geometry::Pose, geometry::Twist>> results;

    geometry::Pose robot_initial_pose;
    geometry::Pose robot_projected_pose = robot_initial_pose.twist(desired_twist, projection_time_s);

    float largest_speed_demand = 0;

    for(std::vector<geometry::Transform>::iterator i = wheel_transforms.begin();
        i != wheel_transforms.end();
        i++)
    {
        geometry::Pose wheel_initial_pose = robot_initial_pose.transform(*i);
        geometry::Pose wheel_projected_pose = robot_projected_pose.transform(*i);

        geometry::Transform wheel_transformation = wheel_initial_pose.get_Transform(wheel_projected_pose);

        float wheel_end_yaw = ck::math::normalize_to_2_pi(wheel_transformation.get_Rotation_To().yaw());

        float initial_yaw = ck::math::normalize_to_2_pi((*i).angular.yaw());
        float mirrored_initial_yaw = ck::math::normalize_to_2_pi(initial_yaw + M_PI);

        float normal_smallest_traversal = smallest_traversal(initial_yaw, wheel_end_yaw);
        float mirror_smallest_traversal = smallest_traversal(mirrored_initial_yaw, wheel_end_yaw);

        float smallest_overall_traversal = std::fabs(normal_smallest_traversal) < std::fabs(mirror_smallest_traversal) ? normal_smallest_traversal : mirror_smallest_traversal;
        bool flipped = std::fabs(mirror_smallest_traversal) < std::fabs(normal_smallest_traversal);

        geometry::Rotation smallest_traversal_pose;
        smallest_traversal_pose.yaw(ck::math::normalize_to_2_pi(initial_yaw + smallest_overall_traversal));

        std::pair<geometry::Pose, geometry::Twist> wheel_result;
        wheel_result.first.position = wheel_projected_pose.position;
        wheel_result.first.orientation = smallest_traversal_pose;
        wheel_result.second.linear[0] = wheel_transformation.linear.norm() / projection_time_s * (flipped ? -1.0 : 1.0);
        wheel_result.second.angular.yaw(smallest_overall_traversal / projection_time_s);

        if (std::abs(wheel_result.second.linear[0]) > largest_speed_demand)
        {
            largest_speed_demand = std::abs(wheel_result.second.linear[0]);
        }

        results.push_back(wheel_result);
    }

    for (std::vector<std::pair<geometry::Pose, geometry::Twist>>::iterator i = results.begin();
         i != results.end();
         i++)
    {
        if (std::abs(largest_speed_demand) > 0.001 &&
            largest_speed_demand > config_params::robot_max_fwd_vel)
        {
            float ratio = std::abs(config_params::robot_max_fwd_vel / largest_speed_demand);
            (*i).second.linear[0] *= ratio;
        }
    }

    return results;
}
#endif

#ifdef PURE_MATH

// This is adopted from a matlab file from the Gaelhawks - S.Spoldi posted on chiefdelphi at:
// https://www.chiefdelphi.com/t/team-230-gaelhawks-swerve-code-release/420411/3

// % generate commands for velocity and angle for each wheel
// %
// % robot coordinate frame (North, East, Down)
// %    x = forward, xdot = forward velocity
// %    y = right,   ydot = right velocity
// %    z = down, positive rotation is clockwise from above
// %    psidot is rotation rate around z
// %
// % velocity commands (xdot, ydot) are units of choice (ft/sec, in/sec, etc)
// % rotation rate (psidot) needs to be in deg/sec
// %
// %    v1c = velocity command for swerve module 1
// %    t1c = steering (theta) command for sterring module 1
// %    v2c, t2c = module 2, etc.

// function[v1c, t1c, ...
//          v2c, t2c, ...
//          v3c, t3c, ...
//          v4c, t4c] = swerve_cmd(xdot, ydot, psidot);

// % convert psidot to rad/sec
// psidot = psidot*pi/180;

// % wheel locations (x, y pairs, insert real dimensions here)
// %   don't forget to match units to velocity cmd (ft & ft/sec, or in & in/sec)
// %     4  1
// %     3  2
// % p1 = [ 12,  6];
// % p2 = [-12,  6];
// % p3 = [-12, -6];
// % p4 = [ 12, -6];

// cr = [14, 14];
// p1 = [24.75, 24.75] - cr;
// p2 = [ 3.25, 24.75] - cr;
// p3 = [ 3.25,  3.25] - cr;
// p4 = [24.75,  3.25] - cr;

// % vector wheel velocities (v = w X r + vx + vy)

// %        |  0,  0,  w |
// %    v = | px, py,  0 | + i*xdot + j*ydot
// %        |  i,  j,  k |
// %
// %    v = w*(px*j - py*i) + i*xdot + j*ydot
// %
// %    vx = xdot - w*py
// %    vy = ydot + w*px

// v1 = psidot*[-p1(2), p1(1)] + [xdot, ydot];
// v2 = psidot*[-p2(2), p2(1)] + [xdot, ydot];
// v3 = psidot*[-p3(2), p3(1)] + [xdot, ydot];
// v4 = psidot*[-p4(2), p4(1)] + [xdot, ydot];

// % wheel velocity command
// v1c = sqrt(v1(1)^2 + v1(2)^2);
// v2c = sqrt(v2(1)^2 + v2(2)^2);
// v3c = sqrt(v3(1)^2 + v3(2)^2);
// v4c = sqrt(v4(1)^2 + v4(2)^2);

// % wheel angle command (convert to degrees for the radian-impared)
// t1c = 180/pi*atan2(v1(2), v1(1));
// t2c = 180/pi*atan2(v2(2), v2(1));
// t3c = 180/pi*atan2(v3(2), v3(1));
// t4c = 180/pi*atan2(v4(2), v4(1));

std::vector<std::pair<geometry::Pose, geometry::Twist>> calculate_swerve_outputs
    (geometry::Twist desired_twist,
    std::vector<geometry::Transform> wheel_transforms,
    float projection_time_s)
{
    using namespace Eigen;
    (void) projection_time_s;

    // Perform unit conversions to spoldi-space
    double xdot = desired_twist.linear.x();
    double ydot = -desired_twist.linear.y();
    double psidot = -desired_twist.angular.yaw();

    Vector2d p1(wheel_transforms[0].linear.x(), -wheel_transforms[0].linear.y());
    Vector2d p2(wheel_transforms[1].linear.x(), -wheel_transforms[1].linear.y());
    Vector2d p3(wheel_transforms[2].linear.x(), -wheel_transforms[2].linear.y());
    Vector2d p4(wheel_transforms[3].linear.x(), -wheel_transforms[3].linear.y());

    // This ends the transformations to the matlab examples coordinate space

    // v1 = psidot*[-p1(2), p1(1)] + [xdot, ydot];
    Vector2d v1 = psidot * Vector2d(-p1[1], p1[0]) + Vector2d(xdot, ydot);
    Vector2d v2 = psidot * Vector2d(-p2[1], p2[0]) + Vector2d(xdot, ydot);
    Vector2d v3 = psidot * Vector2d(-p3[1], p3[0]) + Vector2d(xdot, ydot);
    Vector2d v4 = psidot * Vector2d(-p4[1], p4[0]) + Vector2d(xdot, ydot);

    // v1c = sqrt(v1(1)^2 + v1(2)^2);
    double v1c = v1.norm();
    double v2c = v2.norm();
    double v3c = v3.norm();
    double v4c = v4.norm();

    // t1c = 180/pi*atan2(v1(2), v1(1)); // except we like radians
    double t1c = std::atan2(v1[1], v1[0]);
    double t2c = std::atan2(v2[1], v2[0]);
    double t3c = std::atan2(v3[1], v3[0]);
    double t4c = std::atan2(v4[1], v4[0]);

    // End Gaelhawks code - lets get these back in our coordinate system
    t1c = ck::math::normalize_to_2_pi(-t1c);
    t2c = ck::math::normalize_to_2_pi(-t2c);
    t3c = ck::math::normalize_to_2_pi(-t3c);
    t4c = ck::math::normalize_to_2_pi(-t4c);

    // And now some code to pack these up the way the rest of our system wants them
    std::vector<std::pair<geometry::Pose, geometry::Twist>> results;

    class result_container
    {
        public:
        result_container(float initial_yaw, float target_yaw, float target_speed)
        {
            this->initial_yaw = initial_yaw;
            this->target_yaw = target_yaw;
            this->target_speed = target_speed;
        }
        float initial_yaw;
        float target_yaw;
        float target_speed;
    };

    std::vector<result_container> wheel_commands;
    wheel_commands.push_back(result_container(wheel_transforms[0].angular.yaw(), t1c, v1c));
    wheel_commands.push_back(result_container(wheel_transforms[0].angular.yaw(), t2c, v2c));
    wheel_commands.push_back(result_container(wheel_transforms[0].angular.yaw(), t3c, v3c));
    wheel_commands.push_back(result_container(wheel_transforms[0].angular.yaw(), t4c, v4c));

    for(const auto &i : wheel_commands)
    {
        float wheel_end_yaw = i.target_yaw;

        float initial_yaw = ck::math::normalize_to_2_pi(i.initial_yaw);
        float mirrored_initial_yaw = ck::math::normalize_to_2_pi(initial_yaw + M_PI);

        float normal_smallest_traversal = smallest_traversal(initial_yaw, wheel_end_yaw);
        float mirror_smallest_traversal = smallest_traversal(mirrored_initial_yaw, wheel_end_yaw);

        float smallest_overall_traversal = std::fabs(normal_smallest_traversal) < std::fabs(mirror_smallest_traversal) ? normal_smallest_traversal : mirror_smallest_traversal;
        bool flipped = std::fabs(mirror_smallest_traversal) < std::fabs(normal_smallest_traversal);

        geometry::Rotation smallest_traversal_pose;
        smallest_traversal_pose.yaw(ck::math::normalize_to_2_pi(initial_yaw + smallest_overall_traversal));

        std::pair<geometry::Pose, geometry::Twist> wheel_result;
        wheel_result.first.orientation = smallest_traversal_pose;
        wheel_result.second.linear[0] = i.target_speed * (flipped ? -1.0 : 1.0);

        results.push_back(wheel_result);
    }

    return results;
}

#endif