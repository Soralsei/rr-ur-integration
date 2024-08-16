#pragma once
#include <vector>
#include <memory>
#include <Eigen/Dense>

#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

#include "ros/ros.h"
// #include "util/math.h"

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
        double lineCost(const Eigen::Vector2i &start, const Eigen::Vector2i &end) const;
        double pointCost(int x, int y) const;
        Eigen::Vector2i worldToMap(const float x, const float y) const;
        Eigen::Vector2f mapToWorld(const int x, const int y) const;

        double footprintCost(const Footprint &footprint) const;
        Footprint getFootprintAt(Eigen::Vector2f position, float theta, Eigen::Vector2f targetPosition, float targetTheta);
    };

} // namespace rhoban
