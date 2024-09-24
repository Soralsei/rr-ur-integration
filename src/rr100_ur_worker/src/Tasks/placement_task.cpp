#include "rr100_ur_worker/Tasks/placement_task.h"

namespace rhoban
{
    PlacementTask::PlacementTask(
        PlacementController &controller_,
        geometry_msgs::PoseStamped target_) : Task::Task(priority::Normal, "PlacementTask"), controller(controller_), target(target_) {}

    PlacementTask::~PlacementTask() {}

    bool PlacementTask::execute()
    {
        // polling implementation or blocking ?
        // Blocking for now...
        return controller.go_to(target);
    }
} // namespace rhoban
