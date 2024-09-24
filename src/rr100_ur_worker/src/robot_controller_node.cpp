#include "rr100_ur_worker/robot_controller.h"
#include "ros/ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robot_controller_node");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    rhoban::RobotController controller{nh, nh_priv};
    ros::spin();

    return 0;
}