#pragma once

#include <memory>
#include <vector>
#include "rr100_ur_worker/placement_controller.h"
#include "rr100_ur_worker/arm_controller.h"
#include "rr100_ur_worker/Tasks/task.h"
#include "rr100_ur_worker/Tasks/compound_task.h"
#include "rr100_ur_worker/TaskArray.h"
#include <mutex>

namespace rhoban
{
    class RobotController
    {
    private:
        using TaskPtr = std::shared_ptr<Task>;

        std::unique_ptr<ArmController> arm_controller;
        std::unique_ptr<PlacementController> placement_controller;

        std::list<TaskPtr> task_queue;
        std::mutex task_queue_mutex;

        ros::Subscriber task_sub;

        ros::Timer update_timer;
        ros::NodeHandle nh;

    public:
        RobotController(ros::NodeHandle nh_, ros::NodeHandle nh_priv);
        ~RobotController();

        void add_task(TaskPtr task);
        void update(const ros::TimerEvent &);
        void taskarray_callback(const rr100_ur_worker::TaskArrayConstPtr &task_array);
    };
} // namespace rhoban
