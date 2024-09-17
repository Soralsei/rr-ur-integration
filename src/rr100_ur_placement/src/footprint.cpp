#include "rr100_ur_placement/footprint.h"

#include <iostream>
#include <vector>
#include <limits>
#include <cstddef>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace rhoban
{
    Footprint transformFootprint(const Footprint &footprint, Eigen::Vector2f position, float theta)
    {
        Footprint transformed{};
        // float cosTheta = cos(theta);
        // float sinTheta = sin(theta);

        // std::cout << "-------------------------------------------------------------------------------------\n";
        for (auto &point : footprint)
        {
            Eigen::Vector2f nPoint{point.x, point.y};
            Eigen::Rotation2D<float> r{theta};
            Eigen::Translation2f t{position};

            nPoint = r * nPoint;
            nPoint = t * nPoint;

            // float newX = position[0] + (point.x * cosTheta - point.y * sinTheta);
            // float newY = position[1] + (point.x * sinTheta + point.y * cosTheta);

            // std::cout << "Transformed footprint point (manual): (" << newX << ", " << newY << ")" << std::endl;
            // std::cout << "Transformed footprint point (eigen) : (" << nPoint[0] << ", " << nPoint[1] << ")" << std::endl;

            geometry_msgs::Point32 newPoint;

            newPoint.x = nPoint[0];
            newPoint.y = nPoint[1];

            transformed.push_back(newPoint);
        }
        // std::cout << "-------------------------------------------------------------------------------------\n";

        return transformed;
    }

    /**
     * @return an std::vector<int> in the order [xmin, xmax, ymin, ymax]
     */
    std::vector<float> getFootprintBounds(const Footprint &footprint)
    {
        std::vector<float> bounds = std::vector<float>{
            std::numeric_limits<float>::max(),
            std::numeric_limits<float>::lowest(),
            std::numeric_limits<float>::max(),
            std::numeric_limits<float>::lowest()};
            
        for (auto &point : footprint)
        {
            if (point.x <= bounds[0])
                bounds[0] = point.x;
            if (point.x >= bounds[1])
                bounds[1] = point.x;
            if (point.y <= bounds[2])
                bounds[2] = point.y;
            if (point.y >= bounds[3])
                bounds[3] = point.y;
        }

        return bounds;
    }

    bool isInsideFootprint(const Footprint &footprint, Eigen::Vector2f point)
    {
        std::size_t count = footprint.size();

        Eigen::Vector2f start{footprint[count - 1].x, footprint[count - 1].y};
        Eigen::Vector2f end{footprint[0].x, footprint[0].y};
        Eigen::Vector2f normal = getNormal(start, end);

        bool sign = std::signbit(normal.dot(point));

        for (std::size_t i = 1, j = 0; i < count; j = i++)
        {
            Eigen::Vector2f start{footprint[j].x, footprint[j].y};
            Eigen::Vector2f end{footprint[i].x, footprint[i].y};
            Eigen::Vector2f normal = getNormal(start, end);

            bool nextSign = std::signbit(normal.dot(point));
            if (sign != nextSign)
            {
                return false;
            }
            sign = nextSign;
        }

        return true;
    }

    static inline Eigen::Vector2f getNormal(Eigen::Vector2f start, Eigen::Vector2f end)
    {
        Eigen::Vector2f delta = (end - start);
        return Eigen::Vector2f{-delta[1], delta[0]};
    }
} // namespace rhoban
