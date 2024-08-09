#pragma once
#include <vector>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

namespace rhoban
{

typedef std::vector<geometry_msgs::Point> Footprint;

class CollisionChecker
{
private:
    Footprint footprint;
public:
    CollisionChecker(/* args */);
    ~CollisionChecker();
};

CollisionChecker::CollisionChecker(/* args */)
{
}

CollisionChecker::~CollisionChecker()
{
}

} // namespace rhoban


