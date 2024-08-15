#pragma once
#include <vector>
#include <memory>

#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

#include "ros/ros.h"
#include "util/math.h"

namespace rhoban
{
    using MapPtr = nav_msgs::OccupancyGrid::ConstPtr;
    using PoseMsg = geometry_msgs::Pose;

    using Footprint = std::vector<geometry_msgs::Point32>;

    class MapSubscriber;
    class FootprintSubscriber;

    class CollisionChecker
    {
    private:
        Footprint mFootprint;
        std::unique_ptr<FootprintSubscriber> mFootprintSubscriber;
        std::unique_ptr<MapSubscriber> mMapSubscriber;

        MapPtr mMap;

    public:
        CollisionChecker(
            ros::NodeHandle &node,
            const std::string &footprint_topic,
            const std::string &map_topic);
        ~CollisionChecker();

        double scorePose(const PoseMsg &currentPose, const PoseMsg &targetPose);
        bool isCollisionFree(const PoseMsg &currentPose, const PoseMsg &targetPose);

    private:
        double lineCost(const math::Vector2<int> &start, const math::Vector2<int> &end) const;
        double pointCost(const math::Vector2<int> &point) const;
        math::Vector2<int> worldToMap(const float x, const float y) const;

        double footprintCost(const Footprint &footprint) const;
        Footprint getFootprintAt(math::Vector2<float> position, float theta, math::Vector2<float> targetPosition, float targetTheta);
    };

} // namespace rhoban
