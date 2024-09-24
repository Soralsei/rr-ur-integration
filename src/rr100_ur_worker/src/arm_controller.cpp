#include "rr100_ur_worker/arm_controller.h"

namespace rhoban
{

    const double ArmController::GRIPPER_CLOSED = 0.025;
    const double ArmController::GRIPPER_OPEN = 0.0;

    ArmController::ArmController(
        const std::string &ik_action,
        const std::string &control_action,
        const std::string &gripper_action,
        double connection_timeout)
        : ik_client(ik_action, true),
          control_client(control_action, true),
          gripper_client(gripper_action, true)
    {
        bool connected = ik_client.waitForServer(ros::Duration(connection_timeout));
        if (!connected)
        {
            throw std::runtime_error(std::string("Failed to connect to UR IK action server ") + ik_action + std::string(", make sure it is running"));
        }
        ROS_INFO("Connected to IK action server");

        connected = control_client.waitForServer(ros::Duration(connection_timeout));
        if (!connected)
        {
            throw std::runtime_error("Failed to connect to Joint controller action server, make sure it is running");
        }
        ROS_INFO("Connected to Joint Trajectory controller action server");

        connected = gripper_client.waitForServer(ros::Duration(connection_timeout));
        if (!connected)
        {
            throw std::runtime_error("Failed to connect to Gripper controller action server, make sure it is running");
        }
        ROS_INFO("Connected to Gripper controller action server");
    }

    ArmController::~ArmController() {}

    tl::optional<trajectory_msgs::JointTrajectory> ArmController::get_trajectory(
        const geometry_msgs::PoseStamped &pose,
        double duration)
    {
        ur5_kinematics::URGoToGoal goal;
        goal.target_pose = pose;
        goal.duration = ros::Duration(duration);
        goal.timeout = 2.0;
        goal.target_pose.header.seq = seq_num;

        ik_client.sendGoalAndWait(goal);
        auto result = ik_client.getResult();

        if (result->state == ur5_kinematics::URGoToResult::SUCCEEDED)
        {
            seq_num++;
            return tl::make_optional<trajectory_msgs::JointTrajectory>(result->trajectory);
        }

        return tl::nullopt;
    }

    ArmController::Result ArmController::go_to(
        const geometry_msgs::PoseStamped &pose,
        double duration = 0.0)
    {
        Result res = Result::SUCCESS;
        auto trajectory = get_trajectory(pose, duration);
        if (trajectory.has_value())
        {
            control_msgs::FollowJointTrajectoryGoal goal;
            goal.trajectory = trajectory.value();
            std::vector<control_msgs::JointTolerance> tolerance;
            for (auto &&name : trajectory.value().joint_names)
            {
                control_msgs::JointTolerance tol;
                tol.name = name;
                tol.position = -1;
                tol.velocity = -1;
                tol.acceleration = -1;
                tolerance.push_back(tol);
            }
            
            // If tolerances are required,
            goal.goal_tolerance = tolerance;
            goal.path_tolerance = tolerance;
            // goal.goal_time_tolerance = ...
            control_client.sendGoalAndWait(goal);
            auto ret = control_client.getResult();
            if (ret->error_code != control_msgs::FollowJointTrajectoryResult::SUCCESSFUL)
            {
                ROS_ERROR("UR arm control failed with code %d, %s...", ret->error_code, ret->error_string.c_str());
                res = Result::CONTROLFAIL;
            }
        }
        else
        {
            ROS_ERROR("UR IK failed...");
            res = Result::IKFAIL;
        }
        return res;
    }

    ArmController::Result ArmController::set_gripper(double value)
    {
        if (value == gripper_effort)
        {
            return Result::SUCCESS;
        }

        control_msgs::GripperCommandGoal goal;
        goal.command.position = value;
        gripper_client.sendGoalAndWait(goal);

        auto ret = gripper_client.getResult();
        gripper_effort = ret->effort;
        gripper_effort = ret->position;

        if (ret->stalled)
        {
            return Result::STALLED;
        }
        else if (!ret->reached_goal)
        {
            return Result::CONTROLFAIL;
        }

        return Result::SUCCESS;
    }

    double ArmController::get_gripper_position()
    {
        return gripper_position;
    }

    double ArmController::get_gripper_effort()
    {
        return gripper_effort;
    }
} // namespace rhoban
