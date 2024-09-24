#pragma once

#include "rr100_ur_worker/Tasks/task.h"
#include "rr100_ur_worker/arm_controller.h"

namespace rhoban
{
    class GripperTask : public Task
    {

    public:
        enum class Action
        {
            Grip,
            Release,
            Set
        };

    private:
        double value;

        geometry_msgs::PoseStamped target;
        Action action;

        ArmController &controller;

    public:
        GripperTask(ArmController &controller_, Action action_, double value_ = 0.0);
        ~GripperTask();
        virtual bool execute();
    };
} // namespace rhoban
