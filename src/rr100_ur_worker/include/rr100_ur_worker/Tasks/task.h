#pragma once

#include <string>
#include <cstdint>
#include <cstddef>
#include <limits>

namespace rhoban
{
    using priority_t = std::int_least16_t;

    namespace priority
    {
        constexpr priority_t Highest = std::numeric_limits<std::int16_t>::max();
        constexpr priority_t High = std::numeric_limits<std::int16_t>::max() / 2;
        constexpr priority_t Normal = 0;
        constexpr priority_t Low = std::numeric_limits<std::int16_t>::lowest() / 2;
        constexpr priority_t Lowest = std::numeric_limits<std::int16_t>::lowest();
    } // namespace priority

    class Task
    {
    protected:
        priority_t priority;
        std::string name;

        bool retrying = false;
        bool retry;

    public:
        Task(const priority_t priority_ = priority::Normal, std::string name_ = "") : priority(priority_), name(name_), retry(false) {}
        ~Task() {};
        virtual bool execute() = 0;

        friend bool operator<(const Task &lhs, const Task &rhs)
        {
            return lhs.priority < rhs.priority;
        }

        std::string &get_name()
        {
            return name;
        }

        bool is_retrying()
        {
            return retrying;
        }

        void set_retrying(bool val)
        {
            retrying = val;
        }

        bool should_retry(void)
        {
            return retry && !retrying;
        }
    };
} // namespace rhoban
