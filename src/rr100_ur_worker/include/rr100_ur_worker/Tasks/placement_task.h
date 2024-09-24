#pragma once
#include "rr100_ur_worker/Tasks/task.h"
#include "rr100_ur_worker/placement_controller.h"
#include "geometry_msgs/PoseStamped.h"

namespace rhoban
{
    class PlacementTask : public Task
    {
        
    private:
        geometry_msgs::PoseStamped target;
        PlacementController &controller;

    public:
        PlacementTask(PlacementController &controller_, geometry_msgs::PoseStamped target_);
        ~PlacementTask();
        virtual bool execute();
    };
} // namespace rhoban
