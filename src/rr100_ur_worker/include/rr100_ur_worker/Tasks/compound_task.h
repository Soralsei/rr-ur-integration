#pragma once
#include "rr100_ur_worker/Tasks/task.h"
#include <queue>
#include <vector>
#include <memory>
#include <algorithm>

namespace rhoban
{
    class CompoundTask : Task
    {
        using TaskPtr = std::shared_ptr<Task>;

    private:
        std::queue<TaskPtr> children;

    public:
        CompoundTask(priority_t priority_ = priority::Normal, size_t size = 0);
        ~CompoundTask();

        void add(TaskPtr task);
    };

    void CompoundTask::add(TaskPtr task)
    {
        children.push(std::move(task));
    }

    CompoundTask::CompoundTask(priority_t priority_ = priority::Normal, size_t size = 0) : Task::Task(priority_) {}

    CompoundTask::~CompoundTask()
    {
        while (!children.empty())
        {
            children.pop();
        }
    }
} // namespace rhoban
