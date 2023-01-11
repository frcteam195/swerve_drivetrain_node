#pragma once

#include <ck_utilities/geometry/geometry.hpp>

void publishOdometryData();
void publish_motor_links();
geometry::Transform get_robot_transform();
void update_drivetrain_diagnostics_position();