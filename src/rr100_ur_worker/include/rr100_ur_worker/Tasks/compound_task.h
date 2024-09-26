#pragma once

#include "rr100_ur_worker/Tasks/task.h"
#include "rr100_ur_worker/Tasks/gripper_task.h"
#include "rr100_ur_worker/Tasks/placement_task.h"
#include "rr100_ur_worker/Tasks/reaching_task.h"
#include "rr100_ur_worker/arm_controller.h"
#include "rr100_ur_worker/placement_controller.h"

// #include <queue>
#include "tf2_ros/buffer.h"
#include <vector>
#include <memory>

namespace rhoban
{
    class CompoundTask : public Task
    {
        using TaskPtr = std::shared_ptr<Task>;

    private:
        std::list<TaskPtr> children;
        ArmController &arm_controller;
        PlacementController &placement_controller;

    public:
        CompoundTask(ArmController &arm_controller, PlacementController &placement_controller, priority_t priority_ = priority::Normal, size_t size = 0);
        ~CompoundTask();

        virtual bool execute() override;

        CompoundTask &add(TaskPtr task);
        CompoundTask &add_reaching(ArmController &controller_, geometry_msgs::PoseStamped target_, tf2_ros::Buffer &tf, double duration_ = 0.0);
        CompoundTask &add_placement(PlacementController &controller_, geometry_msgs::PoseStamped target_);
        CompoundTask &add_gripper(ArmController &controller_, GripperTask::Action action_, double position = 0.0);
    };
} // namespace rhoban
