#pragma once
#include <string>
#include <memory>

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

namespace rhoban
{

    using MapPtr = nav_msgs::OccupancyGrid::ConstPtr;

    class MapSubscriber
    {
    public:
        MapSubscriber(
            ros::NodeHandle &node,
            const std::string &topic_name);

        MapPtr getMap();

    protected:
        // Interfaces used for logging and creating publishers and subscribers
        ros::NodeHandle &mNodeHandle;

        void mapCallback(const MapPtr &msg);

        MapPtr mMap;
        bool isMapReceived{false};
        ros::Subscriber mMapSubscriber;
    };

} // namespace nav2_costmap_2d