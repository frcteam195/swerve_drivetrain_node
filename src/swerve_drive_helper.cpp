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

std::vector<std::pair<geometry::Pose, geometry::Twist>> calculate_swerve_outputs_internal
    (geometry::Twist desired_twist,
    std::vector<geometry::Transform> wheel_transforms,
    float projection_time_s)
{
    std::stringstream log;
    log << std::endl;
    log << "---------------------------------" << std::endl;
    log << "calculate_swerve_outputs_internal" << std::endl;
    log << "----------" << std::endl;
    log << "Inputs: " << std::endl;
    log << "Desired Twist: " << std::endl << desired_twist << std::endl;
    log << "Wheel Transforms: " << std::endl << wheel_transforms;
    log << "Projection Time s:" << projection_time_s << std::endl;
    log << "----------" << std::endl;

    std::vector<std::pair<geometry::Pose, geometry::Twist>> results;

    geometry::Pose robot_initial_pose;
    geometry::Pose robot_projected_pose = robot_initial_pose.twist(desired_twist, projection_time_s);

    log << "robot_initial_pose: " << robot_initial_pose;
    log << "robot_projected_pose: " << robot_projected_pose;

    for(std::vector<geometry::Transform>::iterator i = wheel_transforms.begin();
        i != wheel_transforms.end();
        i++)
    {
        log << "----------" << std::endl;
        geometry::Pose wheel_initial_pose = robot_initial_pose.transform(*i);
        geometry::Pose wheel_projected_pose = robot_projected_pose.transform(*i);
        std::stringstream s0;
        // log << "I: " << (*i);

        geometry::Transform wheel_transformation = wheel_initial_pose.get_Transform(wheel_projected_pose);

        // log << "wheel_transformation: " << wheel_transformation;
        log << "wheel_projected_pose: " << wheel_projected_pose;

        float wheel_end_yaw = wheel_transformation.get_Rotation_To().yaw();

        float initial_yaw = ck::math::normalize_to_2_pi((*i).angular.yaw());
        float mirrored_initial_yaw = ck::math::normalize_to_2_pi(initial_yaw + M_PI);

        float normal_smallest_traversal = smallest_traversal(initial_yaw, wheel_end_yaw);
        float mirror_smallest_traversal = smallest_traversal(mirrored_initial_yaw, wheel_end_yaw);

        // log << "initial_yaw: " << initial_yaw << std::endl;
        // log << "wheel_end_yaw: " << wheel_end_yaw << std::endl;
        // log << "mirrored_initial_yaw: " << mirrored_initial_yaw << std::endl;
        // log << "normal_smallest_traversal: " << normal_smallest_traversal << std::endl;
        // log << "mirror_smallest_traversal: " << mirror_smallest_traversal << std::endl;

        float smallest_overall_traversal = std::fabs(normal_smallest_traversal) < std::fabs(mirror_smallest_traversal) ? normal_smallest_traversal : mirror_smallest_traversal;
        bool flipped = std::fabs(mirror_smallest_traversal) < std::fabs(normal_smallest_traversal);

        // log << "Smallest Traversal" << smallest_overall_traversal << std::endl;
        geometry::Rotation smallest_traversal_pose;
        smallest_traversal_pose.roll(0);
        smallest_traversal_pose.pitch(0);
        smallest_traversal_pose.yaw(initial_yaw + smallest_overall_traversal);

        std::pair<geometry::Pose, geometry::Twist> wheel_result;
        wheel_result.first.position = wheel_projected_pose.position;
        wheel_result.first.orientation = smallest_traversal_pose;
        wheel_result.second.linear.setZero();
        wheel_result.second.linear[0] = wheel_transformation.linear.norm() / projection_time_s * (flipped ? -1.0 : 1.0);
        wheel_result.second.angular.roll(0);
        wheel_result.second.angular.pitch(0);
        wheel_result.second.angular.yaw(smallest_overall_traversal / projection_time_s);

        results.push_back(wheel_result);
    }

    log << "----------" << std::endl;
    log << "Outputs:" << std::endl;
    log << results << std::endl;
    log << "End calculate_swerve_outputs_internal" << std::endl;
    log << "---------------------------------";

    ROS_INFO("%s", log.str().c_str());

    return results;
}

std::ostream& operator<<(std::ostream& os, const geometry_msgs::Quaternion& value)
{
    os << "W: " << value.w << " X: " << value.x << " Y: " << value.y << " Z: " << value.z;
    return os;
}

std::ostream& operator<<(std::ostream& os, const geometry_msgs::Point& value)
{
    os << " X: " << value.x << " Y: " << value.y << " Z: " << value.z;
    return os;
}

std::ostream& operator<<(std::ostream& os, const geometry_msgs::Pose& value)
{
    os << "Orientation: " << value.orientation << std::endl;
    os << "Position: " << value.position << std::endl;
    return os;
}

std::ostream& operator<<(std::ostream& os, const geometry_msgs::Vector3& value)
{
    os << " X: " << value.x << " Y: " << value.y << " Z: " << value.z;
    return os;
}

std::ostream& operator<<(std::ostream& os, const geometry_msgs::Twist& value)
{
    os << "Angular: " << value.angular << std::endl;
    os << "Linear: " << value.linear << std::endl;
    return os;
}

std::ostream& operator<<(std::ostream& os, const std::vector<std::pair<geometry_msgs::Pose, geometry_msgs::Twist>>& value)
{
    std::stringstream s;
    s << "---------------------------------" << std::endl;
    s << "geometry_msgs::Pose, geometry_msgs::Twist Vector" << std::endl;
    for (std::vector<std::pair<geometry_msgs::Pose, geometry_msgs::Twist>>::const_iterator i = value.begin();
         i != value.end();
         i++)
    {
        s << "Pose: " << std::endl << (*i).first << std::endl;
        s << "Twist: " << std::endl << (*i).second << std::endl;
    }
    s << "---------------------------------" << std::endl;
    os << s.str();
    return os;
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
    // std::stringstream s;
    // s << results;
    // ROS_INFO("Results\n%s", s.str().c_str());

    return results;
}