#include <iostream>

#include "rr100_ur_pose_projection/map_subscriber.h"

#include "ros/ros.h"
#include "rr100_ur_pose_projection/util/exceptions.h"
#include "rr100_ur_pose_projection/util/format.h"

namespace rhoban
{
    MapSubscriber::MapSubscriber(
        ros::NodeHandle &node,
        const std::string &topic_name) 
        : mNodeHandle {node}
    {
        mMapSubscriber = mNodeHandle.subscribe(topic_name, 1, &MapSubscriber::mapCallback, this);
        ROS_INFO("%s", format("[MapSubscriber] : Subcribed to topic \"%s\"", topic_name.c_str()).c_str());
    }

    MapPtr MapSubscriber::getMap()
    {
        if (!isMapReceived) {
            throw NoMapReceivedException("Trying to get map that's not yet received.");
        }
        return mMap;
    }

    void MapSubscriber::mapCallback(const MapPtr &msg)
    {
        isMapReceived = true;
        mMap = msg;
    }
} // namespace rhoban
