#include "rr100_ur_worker/Tasks/compound_task.h"

namespace rhoban
{

    CompoundTask &CompoundTask::add(TaskPtr task)
    {
        children.push_back(std::move(task));
        return *this;
    }

    CompoundTask::CompoundTask(
        ArmController &arm_controller_,
        PlacementController &placement_controller_,
        priority_t priority_,
        size_t size)
        : Task::Task(priority_, "CompoundTask"),
          arm_controller(arm_controller_),
          placement_controller(placement_controller_) {}

    CompoundTask::~CompoundTask()
    {
        children.clear();
    }

    bool CompoundTask::execute()
    {
        while (!children.empty())
        {
            // ROS_INFO_THROTTLE(1, "Looping...");
            auto task = children.front();
            auto succeeded = task->execute();
            if (!succeeded)
            {
                std::string name = task->get_name();
                ROS_WARN("CompoundTask : Failed to execute %s", name.c_str());
                if (task->should_retry())
                {
                    ROS_INFO("Task should be retried, retrying...");
                }
                else
                {
                    return false;
                }
            }
            children.pop_front();
        }
        return true;
    }

    CompoundTask &CompoundTask::add_reaching(
        ArmController &controller,
        geometry_msgs::PoseStamped target,
        tf2_ros::Buffer &tf,
        int num_retries,
        double duration)
    {
        
        return add(std::make_shared<ReachingTask>(controller, target, duration, num_retries));
    }

    CompoundTask &CompoundTask::add_placement(
        PlacementController &controller,
        geometry_msgs::PoseStamped target,
        int num_retries)
    {
        return add(std::make_shared<PlacementTask>(controller, target, num_retries));
    }

    CompoundTask &CompoundTask::add_gripper(
        ArmController &controller,
        GripperTask::Action action,
        int num_retries,
        double position)
    {
        return add(std::make_shared<GripperTask>(controller, action, position, num_retries));
    }

} // namespace rhoban