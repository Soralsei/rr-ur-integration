#include "rr100_ur_placement/robot_base_placer.h"
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robot_base_placer");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    rhoban::RobotBasePlacer placer{nh, nh_priv};


    ros::spin();
}