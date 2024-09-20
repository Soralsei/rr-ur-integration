#include "actionlib/client/simple_action_client.h"
#include "ur5_kinematics/URGoToAction.h"
#include "ur5_kinematics/URGoToGoal.h"
#include "ur5_kinematics/URGoToResult.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/FollowJointTrajectoryGoal.h"
#include "control_msgs/FollowJointTrajectoryResult.h"
#include "trajectory_msgs/JointTrajectory.h"

namespace rhoban
{
    class ArmController
    {
    public:
        enum Result
        {
            SUCCESS,
            IKFAIL,
            CONTROLFAIL
        };

    private:
        using IKClient = actionlib::SimpleActionClient<ur5_kinematics::URGoToAction>;
        using ControlClient = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;

        std::unique_ptr<IKClient> ik_client;
        std::unique_ptr<ControlClient> control_client;

    protected:
        ArmController(const std::string &ik_action, const std::string &control_action, double connection_timeout = 0.0);
        ~ArmController();

        Result go_to(const geometry_msgs::PoseStamped &pose);
        bool get_trajectory(const geometry_msgs::PoseStamped &pose, trajectory_msgs::JointTrajectory &out);
    };

    ArmController::ArmController(const std::string &ik_action, const std::string &control_action, double connection_timeout = 0.0)
    {
        ik_client = std::make_unique<IKClient>(ik_action);
        control_client = std::make_unique<ControlClient>(control_action);

        bool connected = ik_client->waitForServer(ros::Duration(connection_timeout));
        if (!connected)
        {
            throw std::runtime_error("Failed to connect to UR IK action server, make sure it is running");
        }
        ROS_INFO("Connected to IK action server");

        connected = control_client->waitForServer(ros::Duration(connection_timeout));
        if (!connected)
        {
            throw std::runtime_error("Failed to connect to Joint controller action server, make sure it is running");
        }
        ROS_INFO("Connected to Joint controller action server");
    }

    ArmController::~ArmController() {}

    bool ArmController::get_trajectory(const geometry_msgs::PoseStamped &pose, trajectory_msgs::JointTrajectory &out)
    {
        // TODO : implement
        // refer to tests in ur5_kinematics/testing for implementation example
        return true;
    }

    ArmController::Result ArmController::go_to(const geometry_msgs::PoseStamped &pose)
    {
        // TODO : implement
        // First, contact IK server
        // Second, transmit computed joint trajectory to controller (if succeeded)
        // return SUCCESS if everything succeded, IKFAIL if we failed to solve IK,
        // CONTROLFAIL if we failed to follow the trajectory)
        return Result::SUCCESS;
    }

} // namespace rhoban
