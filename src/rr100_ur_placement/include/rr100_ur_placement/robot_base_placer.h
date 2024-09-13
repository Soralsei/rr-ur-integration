#pragma once

#include <memory>
#include <limits>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include "ros/ros.h"
#include "rr100_ur_placement/GetPlacement.h"

class CollisionChecker;
class WorkspaceDatabase;

namespace rhoban
{
    class RobotBasePlacer
    {
    private:
        const double PI = 3.14159265358979323846;
        // const double MaxScore{std::numeric_limits<double>::max()};
        const double MinScore{0.0};
        const double InvalidScore{std::nan("")};

        ros::ServiceServer placement_service;
        ros::NodeHandle nh;

        std::unique_ptr<WorkspaceDatabase> database;
        std::unique_ptr<CollisionChecker> collision_checker;

        double position_weight = 1.0;
        double rotation_weight = 1.0;

    public:
        RobotBasePlacer(ros::NodeHandle nh_);
        ~RobotBasePlacer();

    private:
        bool determineBestPlacement(rr100_ur_placement::GetPlacementRequest &req, rr100_ur_placement::GetPlacementResponse &res);
        double scorePose(const Eigen::Vector2d& trans, const Eigen::Rotation2Dd& rot);
    };

} // namespace rhoban
