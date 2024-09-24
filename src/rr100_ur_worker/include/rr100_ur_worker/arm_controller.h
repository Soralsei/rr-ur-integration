#pragma once

#include "actionlib/client/simple_action_client.h"

#include "ur5_kinematics/URGoToAction.h"
#include "ur5_kinematics/URGoToGoal.h"
#include "ur5_kinematics/URGoToResult.h"

#include "control_msgs/FollowJointTrajectoryAction.h"
#include "control_msgs/FollowJointTrajectoryGoal.h"
#include "control_msgs/FollowJointTrajectoryResult.h"

#include "control_msgs/GripperCommandAction.h"
#include "control_msgs/GripperCommandGoal.h"
#include "control_msgs/GripperCommandResult.h"

#include "trajectory_msgs/JointTrajectory.h"
#include "control_msgs/GripperCommand.h"

#include "rr100_ur_worker/optional.hpp"

namespace rhoban
{
    class ArmController
    {

        friend class ReachingTask;
        friend class GripperTask;

    public:
        const static double GRIPPER_CLOSED;
        const static double GRIPPER_OPEN;
        enum Result
        {
            SUCCESS,
            IKFAIL,
            CONTROLFAIL,
            STALLED
        };

        double get_gripper_effort();
        double get_gripper_position();

        ArmController(
            const std::string &ik_action,
            const std::string &control_action,
            const std::string &gripper_action,
            double connection_timeout = 5.0);
        ~ArmController();

    private:
        using IKClient = actionlib::SimpleActionClient<ur5_kinematics::URGoToAction>;
        using ControlClient = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;
        using GripperClient = actionlib::SimpleActionClient<control_msgs::GripperCommandAction>;

        IKClient ik_client;
        ControlClient control_client;
        GripperClient gripper_client;

        std::uint32_t seq_num = 0;

        double gripper_effort = 0.0;
        double gripper_position = 0.0;

    protected:
        Result go_to(const geometry_msgs::PoseStamped &pose, double duration);

        tl::optional<trajectory_msgs::JointTrajectory> get_trajectory(
            const geometry_msgs::PoseStamped &pose,
            double duration = 0.0);

        Result set_gripper(double value);
    };
} // namespace rhoban
