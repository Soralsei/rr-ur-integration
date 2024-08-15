#include <iostream>

#include "rr100_ur_pose_projection/footprint.h"
#include "rr100_ur_pose_projection/util/math.h"

namespace rhoban
{
    Footprint transformFootprint(const Footprint &footprint, math::Vector2<float> position, float theta)
    {
        Footprint transformed {};
        float cosTheta = cos(theta);
        float sinTheta = sin(theta);

        for (auto &point : footprint)
        {
            float newX = position.x + (point.x * cosTheta - point.y * sinTheta);
            float newY = position.y + (point.x * sinTheta + point.y * cosTheta);

            geometry_msgs::Point32 newPoint;

            newPoint.x = newX;
            newPoint.y = newY;

            std::cout << "new X : " << newX << ", new Y : " << newY << std::endl;

            transformed.push_back(newPoint);
        }

        return transformed;
    }
} // namespace rhoban
