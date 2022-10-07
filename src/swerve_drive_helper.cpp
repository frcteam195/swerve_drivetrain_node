#include "swerve_drive_helper.hpp"
#include <ck_utilities/geometry/geometry_ros_helpers.hpp>
#include <sstream>
#include <ros/ros.h>
#include <iostream>

static double smallest_traversal(double angle, double target_angle)
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
    s << "---------------------------------" << std::endl;
    s << "Pose:Twist Vector" << std::endl;
    for (std::vector<std::pair<geometry::Pose, geometry::Twist>>::const_iterator i = value.begin();
         i != value.end();
         i++)
    {
        s << (*i).first << " : " << (*i).second << std::endl;
    }
    s << "---------------------------------" << std::endl;
    os << s.str();
    return os;
}

std::ostream& operator<<(std::ostream& os, const std::vector<geometry::Transform>& value)
{
    std::stringstream s;
    s << "---------------------------------" << std::endl;
    s << "Transform Vector" << std::endl;
    for (std::vector<geometry::Transform>::const_iterator i = value.begin();
         i != value.end();
         i++)
    {
        s << (*i) << std::endl;
    }
    s << "---------------------------------" << std::endl;
    os << s.str();
    return os;
}

std::vector<std::pair<geometry::Pose, geometry::Twist>> calculate_swerve_outputs_internal
    (geometry::Twist desired_twist,
    std::vector<geometry::Transform> wheel_transforms,
    float projection_time_s)
{
    std::stringstream inputs;
    inputs << "calculate_swerve_outputs_internal" << std::endl;
    inputs << "Inputs: " << std::endl;
    inputs << "Desired Twist: " << desired_twist << std::endl;
    inputs << "Wheel Transforms: " << wheel_transforms << std::endl;
    inputs << "Projection Time s:" << projection_time_s << std::endl;
    inputs << std::endl;
    ROS_INFO("%s", inputs.str().c_str());

    std::vector<std::pair<geometry::Pose, geometry::Twist>> results;

    geometry::Pose robot_initial_pose;
    geometry::Pose robot_projected_pose = robot_initial_pose.twist(desired_twist, projection_time_s);

    std::stringstream k1;
    k1 << "robot_initial_pose: " << robot_initial_pose << std::endl;
    ROS_INFO("%s", k1.str().c_str());
    std::stringstream k2;
    k2 << "robot_projected_pose: " << robot_projected_pose << std::endl;
    ROS_INFO("%s", k2.str().c_str());

    float largest_hypot = 0;

    for(std::vector<geometry::Transform>::iterator i = wheel_transforms.begin();
        i != wheel_transforms.end();
        i++)
    {
        geometry::Pose wheel_initial_pose = robot_initial_pose.transform(*i);
        geometry::Pose wheel_projected_pose = robot_projected_pose.transform(*i);
        std::stringstream s0;
        s0 << "I: " << (*i) << std::endl;
        ROS_INFO("%s", s0.str().c_str());

        geometry::Transform wheel_transformation = wheel_initial_pose.get_Transform(wheel_projected_pose);

        std::stringstream s1;
        s1 << "wheel_transformation: " << wheel_transformation << std::endl;
        ROS_INFO("%s", s1.str().c_str());
        std::stringstream s2;
        s2 << "wheel_projected_pose: " << wheel_projected_pose << std::endl;
        ROS_INFO("%s", s2.str().c_str());

        largest_hypot = std::max(wheel_transformation.linear.norm(), largest_hypot);
        float wheel_end_yaw = wheel_transformation.get_Rotation_To().yaw();

        float initial_yaw = ck::math::normalize_to_2_pi((*i).angular.yaw());
        float mirrored_initial_yaw = ck::math::normalize_to_2_pi(initial_yaw + M_PI);

        float normal_smallest_traversal = smallest_traversal(initial_yaw, wheel_end_yaw);
        float mirror_smallest_traversal = smallest_traversal(mirrored_initial_yaw, wheel_end_yaw);

        ROS_INFO("initial_yaw: %f\n", initial_yaw);
        ROS_INFO("wheel_end_yaw: %f\n", wheel_end_yaw);
        ROS_INFO("mirrored_initial_yaw: %f\n", mirrored_initial_yaw);
        ROS_INFO("normal_smallest_traversal: %f\n", normal_smallest_traversal);
        ROS_INFO("mirror_smallest_traversal: %f\n", mirror_smallest_traversal);

        float smallest_overall_traversal = std::fabs(normal_smallest_traversal) < std::fabs(mirror_smallest_traversal) ? normal_smallest_traversal : mirror_smallest_traversal;

        ROS_INFO("Smallest Traversal %f\n", smallest_overall_traversal);
        geometry::Rotation smallest_traversal_pose;
        smallest_traversal_pose.roll(0);
        smallest_traversal_pose.pitch(0);
        smallest_traversal_pose.yaw(initial_yaw + smallest_overall_traversal);

        std::pair<geometry::Pose, geometry::Twist> wheel_result;
        wheel_result.first.position = wheel_projected_pose.position;
        wheel_result.first.orientation = smallest_traversal_pose;
        wheel_result.second.linear = wheel_transformation.linear / projection_time_s;
        wheel_result.second.angular.roll(0);
        wheel_result.second.angular.pitch(0);
        wheel_result.second.angular.yaw(smallest_overall_traversal / projection_time_s);

        results.push_back(wheel_result);
    }

    for(std::vector<std::pair<geometry::Pose, geometry::Twist>>::iterator i = results.begin();
        i != results.end();
        i++)
    {
        double hypot = (*i).first.position.norm();
        if(largest_hypot >= 0.01)
        {
            double ratio = hypot / largest_hypot;
            (*i).first.position *= ratio;
            (*i).second.linear *= ratio;
        }
    }

    std::stringstream outputs;
    outputs << "Outputs:" << std::endl;
    outputs << results << std::endl;
    outputs << "End calculate_swerve_outputs_internal" << std::endl;
    ROS_INFO("%s", outputs.str().c_str());

    return results;
}

std::vector<std::pair<geometry_msgs::Pose, geometry_msgs::Twist>> calculate_swerve_outputs
    (geometry_msgs::Twist desired_twist,
    ck::swerve::SwerveDriveConfig& wheel_transforms,
    double projection_time_s)
{
    geometry::Twist desired_twist_ = geometry::to_twist(desired_twist);

    std::vector<geometry::Transform> wheel_transforms_;
    for(std::vector<ck::swerve::WheelConfig>::iterator i = wheel_transforms.wheels.begin();
        i != wheel_transforms.wheels.end();
        i++)
    {
        wheel_transforms_.push_back(geometry::to_transform((*i).transform));
    }

    std::vector<std::pair<geometry::Pose, geometry::Twist>> raw_results;
    raw_results = calculate_swerve_outputs_internal(desired_twist_, wheel_transforms_, projection_time_s);
    std::vector<std::pair<geometry_msgs::Pose, geometry_msgs::Twist>> results;

    for (std::vector<std::pair<geometry::Pose, geometry::Twist>>::iterator i = raw_results.begin();
         i != raw_results.end();
         i++)
    {
        std::pair<geometry_msgs::Pose, geometry_msgs::Twist> entry =
            std::make_pair<geometry_msgs::Pose, geometry_msgs::Twist>
                (geometry::to_msg((*i).first), geometry::to_msg((*i).second));
        results.push_back(entry);
    }

    return results;
}