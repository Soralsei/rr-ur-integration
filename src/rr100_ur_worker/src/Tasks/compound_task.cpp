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
                    task->set_retrying(true);
                    bool is_reaching_task = name.compare("ReachingTask") == 0;
                    if (is_reaching_task)
                    {
                        auto reaching_task = std::dynamic_pointer_cast<ReachingTask>(task);
                        auto placement_task = std::make_shared<PlacementTask>(placement_controller, reaching_task->get_target());

                        ROS_INFO_STREAM("Adding " << placement_task->get_name() << " to front of queue");
                        children.push_front(std::move(placement_task));
                    }
                }
                else
                {
                    return false;
                }
            }
            if (succeeded || !task->is_retrying())
            {
                children.pop_front();
            }
        }
        return true;
    }

    CompoundTask &CompoundTask::add_reaching(
        ArmController &controller,
        geometry_msgs::PoseStamped target,
        tf2_ros::Buffer &tf,
        double duration)
    {
        geometry_msgs::PoseStamped transformed_target;
        geometry_msgs::TransformStamped target_to_map;
        try
        {
            target_to_map = tf.lookupTransform("map", target.header.frame_id, ros::Time(0));
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        tf2::doTransform(target, transformed_target, target_to_map);
        
        return add(std::make_shared<ReachingTask>(controller, transformed_target, duration));
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