#include "rr100_ur_worker/placement_controller.h"

namespace rhoban
{
    PlacementController::PlacementController(
        ros::NodeHandle nh_,
        const std::string &placement_service,
        const std::string &move_base_action,
        tf2_ros::Buffer &tf_)
        : nh(nh_), move_base_client(move_base_action, true), tf(tf_)
    {
        placement_client = nh.serviceClient<rr100_ur_placement::GetPlacement>(placement_service);
        bool connected = placement_client.waitForExistence(ros::Duration(10.0));
        if (!connected)
        {
            throw std::runtime_error("Failed to connect to placement service");
        }
        ROS_INFO("Connected to placement service");
        connected = move_base_client.waitForServer(ros::Duration(10.0));
        if (!connected)
        {
            throw std::runtime_error("Failed to connect to move_base action");
        }
        ROS_INFO("Connected to move_base action");
    }

    PlacementController::~PlacementController() {}

    tl::optional<geometry_msgs::PoseStamped> PlacementController::get_placement(
        const geometry_msgs::PoseStamped &current,
        const geometry_msgs::PoseStamped &target)
    {
        assert(current.header.frame_id.compare("map") == 0);
        assert(target.header.frame_id.compare("map") == 0);

        rr100_ur_placement::GetPlacementRequest req;
        rr100_ur_placement::GetPlacementResponse res;
        req.current = current;
        req.target = target;

        auto success = placement_client.call(req, res);
        if (!success)
        {
            return tl::nullopt;
        }

        return tl::make_optional<geometry_msgs::PoseStamped>(res.best);
    }

    bool PlacementController::go_to(const geometry_msgs::PoseStamped &pose)
    {
        geometry_msgs::TransformStamped base_to_map, target_to_map;
        try
        {
            base_to_map = tf.lookupTransform("map", "base_footprint", ros::Time(0));
            target_to_map = tf.lookupTransform("map", pose.header.frame_id, ros::Time(0));
        }
        catch (const std::exception &e)
        {
            ROS_ERROR_STREAM(e.what() << '\n');
            return false;
        }
        // 0 initialized
        geometry_msgs::PoseStamped identity, current, target;
        // Identity quaternion
        identity.pose.orientation.w = 1.0;
        tf2::doTransform(identity, current, base_to_map);
        tf2::doTransform(pose, target, target_to_map);

        auto placement = get_placement(current, target);
        if (placement.has_value())
        {
            auto P = placement.value();
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose = P;
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.header.frame_id = "map";

            auto ret = move_base_client.sendGoalAndWait(goal, ros::Duration(0.0));
            bool succeeded = ret == actionlib::SimpleClientGoalState::SUCCEEDED;
            ROS_WARN_STREAM_COND(!succeeded, "PlacementController : Failed to move robot base to target\n"
                                                 << pose);
            return succeeded;
        }
        ROS_WARN_STREAM("PlacementController : Failed to get placement for target\n"
                        << pose);

        return false;
    }
} // namespace rhoban
