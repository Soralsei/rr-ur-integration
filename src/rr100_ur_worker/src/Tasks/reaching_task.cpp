#include "rr100_ur_worker/Tasks/reaching_task.h"

namespace rhoban
{
    ReachingTask::ReachingTask(
        ArmController &controller_,
        geometry_msgs::PoseStamped target_,
        double duration_)
        : Task::Task(priority::Normal, "ReachingTask"),
        controller(controller_),
        target(target_), duration(duration_) {
            retry = true;
        }

    ReachingTask::~ReachingTask() {}

    bool ReachingTask::execute()
    {
        // polling implementation or blocking ?
        // Blocking for now...
        auto ret = controller.go_to(target, duration);
        switch (ret)
        {
        case ArmController::Result::IKFAIL:
            ROS_WARN_STREAM("ReachingTask : IK failed for target\n"
                            << target);
            return false;

        case ArmController::Result::CONTROLFAIL:
            ROS_WARN_STREAM("ReachingTask : Control failed for target\n"
                            << target);
            return false;

        case ArmController::Result::SUCCESS:
            return true;

        default:
            ROS_WARN("ReachingTask : Got non-matching result %d", ret);
            return false;
        }
    }

    geometry_msgs::PoseStamped ReachingTask::get_target()
    {
        return target;
    }
} // namespace rhoban
