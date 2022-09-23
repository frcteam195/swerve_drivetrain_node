#include "swerve_drive_helper.hpp"
#include <ck_utilities/geometry.hpp>

double DriveHelper::smallest_traversal(double angle, double target_angle)
{
    double left = -ck::math::normalize_to_2_pi(angle - target_angle);
    double right = ck::math::normalize_to_2_pi(target_angle - angle);
    if(fabs(left) < fabs(right))
    {
        return left;
    }
    return right;
}

// std::vector<std::pair<geometry::Pose, geometry::Twist>> calculate_swerve_outputs_internal
//     (geometry::Twist desired_twist,
//     std::vector<geometry::Transform> wheel_transforms,
//     float projection_time_s)
// {
//     std::vector<std::pair<geometry::Pose, geometry::Twist>> results;

//     geometry::Pose robot_initial_pose;
//     geometry::Pose robot_projected_pose = robot_initial_pose.twist(desired_twist, projection_time_s);

//     float largest_hypot = 0;

//     for(std::vector<geometry::Transform>::iterator i = wheel_transforms.begin();
//         i != wheel_transforms.end();
//         i++)
//     {
//         geometry::Pose wheel_initial_pose = robot_initial_pose.transform(*i);
//         geometry::Pose wheel_projected_pose = robot_projected_pose.transform(*i);

//         geometry::Transform wheel_transformation = wheel_initial_pose.get_Transform(wheel_projected_pose);
//         largest_hypot = std::max(wheel_transformation.linear.norm(), largest_hypot);
//         double wheel_end_yaw = wheel_transformation.get_Rotation_To().z();

//         tf2::Quaternion wheel_initial_quaternion;
//         tf2::fromMsg(wheel_initial_pose.orientation, wheel_initial_quaternion);
//         double initial_roll, initial_pitch, initial_yaw;
//         (void) initial_roll;
//         (void) initial_pitch;
//         tf2::Matrix3x3(wheel_initial_quaternion).getRPY(initial_roll, initial_pitch, initial_yaw);

//         initial_yaw = ck::math::normalize_to_2_pi(initial_yaw);
//         double mirrored_initial_yaw = ck::math::normalize_to_2_pi(initial_yaw + M_PI);

//         double normal_smallest_traversal = smallest_traversal(initial_yaw, wheel_end_yaw);
//         double mirror_smallest_traversal = smallest_traversal(mirrored_initial_yaw, wheel_end_yaw);


//         double smallest_overall_traversal = std::fabs(normal_smallest_traversal) < std::fabs(mirror_smallest_traversal) ? normal_smallest_traversal : mirror_smallest_traversal;

//         tf2::Quaternion smallest_traversal_pose;
//         smallest_traversal_pose.setRPY(0, 0, initial_yaw + smallest_overall_traversal);

//         std::pair<geometry_msgs::Pose, geometry_msgs::Twist> wheel_result;
//         wheel_result.first.position = wheel_projected_pose.position;
//         wheel_result.first.orientation = toMsg(smallest_traversal_pose);
//         wheel_result.second.linear.x = wheel_projected_pose.position.x / projection_time_s;
//         wheel_result.second.linear.y = wheel_projected_pose.position.y / projection_time_s;
//         wheel_result.second.linear.z = wheel_projected_pose.position.z / projection_time_s;
//         wheel_result.second.angular.x = 0;
//         wheel_result.second.angular.y = 0;
//         wheel_result.second.angular.z = smallest_overall_traversal / projection_time_s;

//         results.push_back(wheel_result);
//     }

//     for(std::vector<std::pair<geometry_msgs::Pose, geometry_msgs::Twist>>::iterator i = results.begin();
//         i != results.end();
//         i++)
//     {
//         double hypot = ck::math::hypotenuse((*i).first.position.x, (*i).first.position.y);
//         double ratio = hypot / largest_hypot;
//         (*i).first.position.x *= ratio;
//         (*i).first.position.y *= ratio;
//         (*i).second.linear.x *= ratio;
//         (*i).second.linear.y *= ratio;
//     }

//     return results;
// }

std::vector<std::pair<geometry_msgs::Pose, geometry_msgs::Twist>> DriveHelper::calculate_swerve_outputs
    (geometry_msgs::Twist desired_twist,
    ck::swerve::SwerveDriveConfig& wheel_transforms,
    double projection_time_s)
{
    std::vector<std::pair<geometry_msgs::Pose, geometry_msgs::Twist>> results;

    geometry_msgs::Pose robot_initial_pose = ck::math::null_pose();
    geometry_msgs::Pose robot_projected_pose = ck::math::apply_twist(robot_initial_pose, desired_twist, projection_time_s);

    double largest_hypot = 0;

    for(std::vector<ck::swerve::WheelConfig>::iterator i = wheel_transforms.wheels.begin();
        i != wheel_transforms.wheels.end();
        i++)
    {
        geometry_msgs::Pose wheel_initial_pose = ck::math::apply_transform(robot_initial_pose, (*i).transform);
        geometry_msgs::Pose wheel_projected_pose = ck::math::apply_transform(robot_projected_pose, (*i).transform);

        double hypot = ck::math::hypotenuse(wheel_projected_pose.position.x, wheel_projected_pose.position.y);
        largest_hypot = std::fmax(hypot, largest_hypot);

        double wheel_end_yaw = std::asin(hypot / wheel_projected_pose.position.x);

        tf2::Quaternion wheel_initial_quaternion;
        tf2::fromMsg(wheel_initial_pose.orientation, wheel_initial_quaternion);
        double initial_roll, initial_pitch, initial_yaw;
        (void) initial_roll;
        (void) initial_pitch;
        tf2::Matrix3x3(wheel_initial_quaternion).getRPY(initial_roll, initial_pitch, initial_yaw);

        initial_yaw = ck::math::normalize_to_2_pi(initial_yaw);
        double mirrored_initial_yaw = ck::math::normalize_to_2_pi(initial_yaw + M_PI);

        double normal_smallest_traversal = smallest_traversal(initial_yaw, wheel_end_yaw);
        double mirror_smallest_traversal = smallest_traversal(mirrored_initial_yaw, wheel_end_yaw);


        double smallest_overall_traversal = std::fabs(normal_smallest_traversal) < std::fabs(mirror_smallest_traversal) ? normal_smallest_traversal : mirror_smallest_traversal;

        tf2::Quaternion smallest_traversal_pose;
        smallest_traversal_pose.setRPY(0, 0, initial_yaw + smallest_overall_traversal);

        std::pair<geometry_msgs::Pose, geometry_msgs::Twist> wheel_result;
        wheel_result.first.position = wheel_projected_pose.position;
        wheel_result.first.orientation = toMsg(smallest_traversal_pose);
        wheel_result.second.linear.x = wheel_projected_pose.position.x / projection_time_s;
        wheel_result.second.linear.y = wheel_projected_pose.position.y / projection_time_s;
        wheel_result.second.linear.z = wheel_projected_pose.position.z / projection_time_s;
        wheel_result.second.angular.x = 0;
        wheel_result.second.angular.y = 0;
        wheel_result.second.angular.z = smallest_overall_traversal / projection_time_s;

        results.push_back(wheel_result);
    }

    for(std::vector<std::pair<geometry_msgs::Pose, geometry_msgs::Twist>>::iterator i = results.begin();
        i != results.end();
        i++)
    {
        double hypot = ck::math::hypotenuse((*i).first.position.x, (*i).first.position.y);
        double ratio = hypot / largest_hypot;
        (*i).first.position.x *= ratio;
        (*i).first.position.y *= ratio;
        (*i).second.linear.x *= ratio;
        (*i).second.linear.y *= ratio;
    }

    return results;

}