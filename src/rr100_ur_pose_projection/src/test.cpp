#include <iostream>
#include <string>
#include <Eigen/Dense>

#include "rr100_ur_pose_projection/collision_checker.h"
#include "rr100_ur_pose_projection/footprint_subscriber.h"
#include "rr100_ur_pose_projection/map_subscriber.h"
#include "rr100_ur_pose_projection/util/exceptions.h"

#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"


namespace rhoban
{
    inline
    void convert(const geometry_msgs::Transform& trans, geometry_msgs::Pose& pose)
    {
        pose.orientation = trans.rotation;
        pose.position.x = trans.translation.x;
        pose.position.y = trans.translation.y;
        pose.position.z = trans.translation.z;
    }

    inline
    void convert(const geometry_msgs::Pose& pose, geometry_msgs::Transform& trans)
      {
        trans.rotation = pose.orientation;
        trans.translation.x = pose.position.x;
        trans.translation.y = pose.position.y;
        trans.translation.z = pose.position.z;
    }

    inline
    void convert(const geometry_msgs::TransformStamped& trans, geometry_msgs::PoseStamped& pose)
    {
        convert(trans.transform, pose.pose);
        pose.header = trans.header;
    }

    inline
    void convert(const geometry_msgs::PoseStamped& pose, geometry_msgs::TransformStamped& trans)
    {
        convert(pose.pose, trans.transform);
        trans.header = pose.header;
    }
}



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_footprint_sub");
    ros::NodeHandle nh;
    tf2_ros::Buffer buffer {ros::Duration{20.0}};
    tf2_ros::TransformListener listener{buffer};

    // rhoban::FootprintSubscriber fsub{nh, "/move_base_rr100/global_costmap/footprint"};
    // rhoban::MapSubscriber msub{nh, "/map"};
    rhoban::CollisionChecker col_checker {nh, "/move_base_rr100/global_costmap/footprint", "/map"};
    // rhoban::CollisionChecker col_checker {nh, "/move_base_rr100/global_costmap/footprint", "/move_base_rr100/global_costmap/costmap"};

    // auto pos = Eigen::Vector2f{5.929, -0.522};
    // float theta = -1.416;

    // auto pose = geometry_msgs::Pose{};
    // pose.position.x = 5.929;
    // pose.position.y = -0.522;
    // pose.orientation.x = 0.0;
    // pose.orientation.y = 0.0;
    // pose.orientation.z = 0.650;
    // pose.orientation.w = -0.760;

    while (ros::ok())
    {
        geometry_msgs::TransformStamped transform;
        try
        {
            transform = buffer.lookupTransform("map", "base_footprint", ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            // ROS_WARN("%s", ex.what());
            continue;
        }
        geometry_msgs::PoseStamped pose, target;
        rhoban::convert(transform, pose);
        target.header = pose.header;
        target.pose = pose.pose;
        target.pose.position.x += 1.0;
        try
        {
            auto ret = col_checker.scorePose(pose.pose, target.pose);
            // auto footprint = fsub.getFootprintInRobotFrame(pos, theta);
            // for (auto &&point : footprint)
            // {
            //     std::cout << point.x << " | " << point.y << std::endl;
            // }

            // // ROS_INFO_STREAM(*footprint);

            // auto map = msub.getMap();
            // ROS_INFO_STREAM(map->info);
            ROS_INFO("Pose collision score : %.2f", ret);

            ros::shutdown();
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("%s", e.what());
        }
        ros::spinOnce();
    }
}