#pragma once
#include <vector>
#include <cmath>
#include "geometry_msgs/Point32.h"
#include "util/math.h"

namespace rhoban
{
    using Footprint = std::vector<geometry_msgs::Point32>;

    Footprint transformFootprint(const Footprint &footprint, math::Vector2<float> position, float theta);
} // namespace rhoban
