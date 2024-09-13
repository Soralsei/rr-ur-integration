// #include <iostream>
// #include <iomanip>
#include <Eigen/Dense>

#include "rr100_ur_placement/map_subscriber.h"

#include "ros/ros.h"
#include "rr100_ur_placement/util/exceptions.h"
#include "rr100_ur_placement/util/format.h"

namespace rhoban
{
    MapSubscriber::MapSubscriber(
        ros::NodeHandle &node,
        const std::string &topic_name)
        : mNodeHandle{node}
    {
        mMapSubscriber = mNodeHandle.subscribe(topic_name, 1, &MapSubscriber::mapCallback, this);
        ROS_INFO("%s", format("[MapSubscriber] : Subcribed to topic \"%s\"", topic_name.c_str()).c_str());
    }

    MapPtr MapSubscriber::getMap()
    {
        if (!isMapReceived)
        {
            throw NoMapReceivedException("Trying to get map that's not yet received.");
        }
        return mMap;
    }

    void MapSubscriber::mapCallback(const MapPtr &msg)
    {
        isMapReceived = true;
        mMap = msg;

        // int height = mMap->info.height;
        // int width = mMap->info.width;
        // for (size_t i = height / 2 + 40 - 1; i >= height / 2 - 40; i--)
        // {
        //     for (size_t j = width / 2 - 40; j < width / 2 + 40; j++)
        //     {
        //         int cost = mMap->data[i * width + j];
        //         // std::cout << std::setw(2) << format("%d",  / 100);
        //         if (cost == 0) {
        //             std::cout << std::setw(2) << '.';
        //         } else {
        //             std::cout << std::setw(2) << 'o';
        //         }
        //     }
        //     std::cout << '\b' << std::endl;
        // }
    }
} // namespace rhoban
