#pragma once
#include "rr100_ur_worker/Tasks/task.h"
#include "rr100_ur_worker/arm_controller.h"
#include "geometry_msgs/PoseStamped.h"

namespace rhoban
{
    class ReachingTask : Task
    {
    public:
        enum Action
        {
            Goto,
            Grab,
            LetGo
        };

    private:
        geometry_msgs::PoseStamped target;
        Action action;
        double duration;

        ArmController& controller;

    public:
        ReachingTask(geometry_msgs::PoseStamped target_, Action action_);
        ReachingTask(geometry_msgs::PoseStamped target_, double duration_, Action action_);
        ~ReachingTask();
        virtual bool execute();
    };

    ReachingTask::ReachingTask(geometry_msgs::PoseStamped target_, Action action_) : ReachingTask(target_, 0.0, action_) {}

    ReachingTask::ReachingTask(geometry_msgs::PoseStamped target_, double duration_, Action action_)
        : target(target_), duration(duration_), action(action_) {}

    ReachingTask::~ReachingTask() {}

    bool ReachingTask::execute()
    {
        // TODO : implement
        // polling implementation or blocking ?
        return true;
    }

} // namespace rhoban
