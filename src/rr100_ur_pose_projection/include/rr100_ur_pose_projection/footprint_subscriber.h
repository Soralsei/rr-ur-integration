#pragma once

#include <string>
#include <memory>
#include <mutex>

#include "ros/ros.h"
#include "geometry_msgs/PolygonStamped.h"
#include "util/math.h"


namespace rhoban
{
    using FootprintPtr = geometry_msgs::PolygonStamped::ConstPtr;
    using Footprint = std::vector<geometry_msgs::Point32>;

    class FootprintSubscriber
    {
    private:
        ros::NodeHandle &mNodeHandle;
        ros::Subscriber mFootprintSubscriber;

        std::mutex mFootprintLock;
        FootprintPtr mFootprint;
        bool isFootprintReceived{false};

        void footprintCallback(const FootprintPtr &msg);
        
    public:
        FootprintSubscriber(
            ros::NodeHandle &node,
            const std::string &topic_name);

        Footprint getFootprintRaw();
        Footprint getFootprintInRobotFrame(math::Vector2<float> position, float theta);

    private:
    };

} // namespace rhoban
