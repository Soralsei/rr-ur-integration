#pragma once
#include <string>
#include <stdexcept>

namespace rhoban
{
    class NoFootprintReceivedException : public std::runtime_error
    {
    private:
        /* data */
    public:
        explicit NoFootprintReceivedException(const std::string description) : std::runtime_error(description) {}
    };
    
    class NoMapReceivedException : public std::runtime_error
    {
    private:
        /* data */
    public:
        explicit NoMapReceivedException(const std::string description) : std::runtime_error(description) {}
    };
    
    class IllegalPoseException : public std::runtime_error
    {
    private:
        /* data */
    public:
        explicit IllegalPoseException(const std::string description) : std::runtime_error(description) {}
    };
} // namespace rhoban
