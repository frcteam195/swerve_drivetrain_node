#pragma once

#include <ck_utilities/geometry/geometry.hpp>

geometry::Twist get_twist_from_HMI();
geometry::Twist get_twist_from_auto();

static constexpr float TIMESTEP_DELTA_S = 0.01;