#pragma once
#include <vector>
#include <cmath>
#include <Eigen/Dense>

#include "geometry_msgs/Point32.h"
// #include "util/math.h"

namespace rhoban
{
    using Footprint = std::vector<geometry_msgs::Point32>;

    Footprint transformFootprint(const Footprint &footprint, Eigen::Vector2f position, float theta);
    std::vector<float> getFootprintBounds(const Footprint &footprint);
    static Eigen::Vector2f getNormal(Eigen::Vector2f start, Eigen::Vector2f end);
    bool isInsideFootprint(const Footprint &footprint, Eigen::Vector2f point);
} // namespace rhoban
