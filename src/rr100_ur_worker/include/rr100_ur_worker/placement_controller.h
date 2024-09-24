#pragma once

#include <ros/ros.h>
#include <memory>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "rr100_ur_placement/GetPlacement.h"
#include "rr100_ur_placement/GetPlacementRequest.h"
#include "rr100_ur_placement/GetPlacementResponse.h"
#include "rr100_ur_worker/optional.hpp"

namespace rhoban
{
    class PlacementController
    {
        friend class PlacementTask;

    private:
        using ControllerClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

        ros::NodeHandle nh;

        ros::ServiceClient placement_client;
        ControllerClient move_base_client;
        std::unique_ptr<tf2_ros::Buffer> tf;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener;

    public:
        PlacementController(
            ros::NodeHandle nh_,
            const std::string &placement_service,
            const std::string &move_base_action);
        ~PlacementController();

    protected:
        /**
         * Gets mobile base placement to be able to reach target
         * @note Assumes the both poses to be in "map" frame
         */
        tl::optional<geometry_msgs::PoseStamped> get_placement(
            const geometry_msgs::PoseStamped &current,
            const geometry_msgs::PoseStamped &target);

        bool go_to(const geometry_msgs::PoseStamped &pose);
    };
} // namespace rhoban
