#include "rr100_ur_worker/placement_controller.h"

namespace rhoban
{
    PlacementController::PlacementController(
        ros::NodeHandle nh_,
        const std::string &placement_service,
        const std::string &move_base_action)
        : nh(nh_), move_base_client(move_base_action, true)
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
        tf = std::make_unique<tf2_ros::Buffer>(ros::Duration(100.0));
        tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf);
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
        geometry_msgs::TransformStamped transform;
        try
        {
            transform = tf->lookupTransform("map", "base_footprint", ros::Time(0));
        }
        catch (const std::exception &e)
        {
            ROS_ERROR_STREAM(e.what() << '\n');
            return false;
        }
        // 0 initialized
        geometry_msgs::PoseStamped identity, current;
        // Identity quaternion
        identity.pose.orientation.w = 1.0;
        tf2::doTransform(identity, current, transform);

        auto placement = get_placement(current, pose);
        if (placement.has_value())
        {
            auto P = placement.value();
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose = P;
            goal.target_pose.header.stamp = ros::Time::now();

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
