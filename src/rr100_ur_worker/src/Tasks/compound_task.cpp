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
        : Task::Task(priority_, "CompoundTask"), arm_controller(arm_controller_), placement_controller(placement_controller_) {}

    CompoundTask::~CompoundTask()
    {
        children.clear();
    }

    bool CompoundTask::execute()
    {
        while (!children.empty())
        {
            auto task = children.front();
            auto res = task->execute();
            if (!res)
            {
                std::string name = task->get_name();
                ROS_WARN("CompoundTask : Failed to execute %s", name.c_str());
                if (!task->is_retrying() && name.compare("ReachingTask") == 0)
                {
                    auto reaching_task = std::dynamic_pointer_cast<ReachingTask>(task);
                    auto placement_task = std::make_shared<PlacementTask>(placement_controller, reaching_task->get_target());
                    
                    ROS_INFO_STREAM("Adding " << placement_task->get_name() << " to front of queue");
                    children.push_front(std::move(placement_task));

                    reaching_task->set_retrying(true);
                }
                else
                {
                    return false;
                }
            }
            else
            {
                children.pop_front();
            }
        }
        return true;
    }

    CompoundTask &CompoundTask::add_reaching(
        ArmController &controller,
        geometry_msgs::PoseStamped target,
        double duration)
    {
        return add(std::make_shared<ReachingTask>(controller, target, duration));
    }

    CompoundTask &CompoundTask::add_placement(
        PlacementController &controller,
        geometry_msgs::PoseStamped target)
    {
        return add(std::make_shared<PlacementTask>(controller, target));
    }

    CompoundTask &CompoundTask::add_gripper(
        ArmController &controller,
        GripperTask::Action action,
        double position)
    {
        return add(std::make_shared<GripperTask>(controller, action, position));
    }

} // namespace rhoban