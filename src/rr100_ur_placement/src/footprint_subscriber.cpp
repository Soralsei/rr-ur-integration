#include <iostream>
#include <cmath>
#include <Eigen/Dense>

#include "rr100_ur_placement/footprint_subscriber.h"
#include "rr100_ur_placement/footprint.h"
#include "rr100_ur_placement/util/exceptions.h"
#include "rr100_ur_placement/util/format.h"

#include "ros/ros.h"

namespace rhoban
{
    FootprintSubscriber::FootprintSubscriber(
        ros::NodeHandle &node,
        const std::string &topic_name) : mNodeHandle(node)
    {
        mFootprintSubscriber = mNodeHandle.subscribe(topic_name, 10, &FootprintSubscriber::footprintCallback, this);
        ROS_INFO("%s", format("[FootprintSubscriber] : Subcribed to topic \"%s\"", topic_name.c_str()).c_str());
    }

    Footprint FootprintSubscriber::getFootprintRaw()
    {
        if (!isFootprintReceived)
        {
            throw NoFootprintReceivedException("Trying to get footprint that's not yet received.");
        }
        std::lock_guard<std::mutex> guard{mFootprintLock};
        return mFootprint->polygon.points;
    }

    Footprint FootprintSubscriber::getFootprintInRobotFrame(Eigen::Vector2f position, float theta)
    {
        Footprint footprint = getFootprintRaw();
        Footprint tmp = transformFootprint(footprint, -position, 0.0);
        footprint = transformFootprint(tmp, Eigen::Vector2f {0,0}, -theta);

        return footprint;
    }

    void FootprintSubscriber::footprintCallback(const FootprintPtr &msg)
    {
        ROS_INFO_STREAM_ONCE(*msg);
        std::lock_guard<std::mutex> guard{mFootprintLock};
        isFootprintReceived = true;
        mFootprint = msg;
    }
} // namespace rhoban
