#include "rr100_ur_worker/Tasks/gripper_task.h"

namespace rhoban
{
    GripperTask::GripperTask(
        ArmController &controller_,
        Action action_,
        double position_,
        int num_retries_)
        : Task::Task(priority::Normal, "GripperTask", num_retries_),
          controller(controller_),
          action(action_),
          position(position_) {}

    GripperTask::~GripperTask() {}

    bool GripperTask::execute()
    {
        // polling implementation or blocking ?
        // Blocking for now...
        switch (action)
        {
        case Action::Grip:
            position = ArmController::GRIPPER_CLOSED;
            break;
        case Action::Release:
            position = ArmController::GRIPPER_OPEN;
            break;
        case Action::Set:
            position = std::max(ArmController::GRIPPER_OPEN, std::min(position, ArmController::GRIPPER_CLOSED));
            break;
        default:
            ROS_ERROR("GripperTask : Got Unknown Action %d", static_cast<int>(action));
            return false;
        }

        auto res = controller.set_gripper(position);
        switch (res)
        {
        case ArmController::Result::STALLED:
            ROS_WARN("GripperTask : Gripper stalled at position %.3f", controller.get_gripper_position());
            return false;

        case ArmController::Result::CONTROLFAIL:
            ROS_WARN("GripperTask : Gripper control failed at position %.3f", controller.get_gripper_position());
            return false;

        case ArmController::Result::SUCCESS:
            return true;

        default:
            ROS_WARN("GripperTask : Got non-matching result %d", res);
            return false;
        }
    }

} // namespace rhoban
