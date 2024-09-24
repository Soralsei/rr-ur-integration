#pragma once
#include "rr100_ur_worker/Tasks/task.h"
#include "rr100_ur_worker/arm_controller.h"
#include "geometry_msgs/PoseStamped.h"

namespace rhoban
{
    class ReachingTask : public Task
    {
        
    private:
        geometry_msgs::PoseStamped target;
        double duration;

        ArmController &controller;

    public:
        ReachingTask(ArmController &controller_, geometry_msgs::PoseStamped target_, double duration_ = 0.0);
        ~ReachingTask();
        virtual bool execute();
        geometry_msgs::PoseStamped get_target();
    };
} // namespace rhoban
