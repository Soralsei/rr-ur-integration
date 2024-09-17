#pragma once

#include <memory>
#include <limits>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>
#include "ros/ros.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include "rr100_ur_placement/GetPlacement.h"

namespace rhoban
{
    class CollisionChecker;
    class WorkspaceDatabase;

    class RobotBasePlacer
    {
    private:
        const double PI = 3.14159265358979323846;
        // const double MaxScore{std::numeric_limits<double>::max()};
        const double MinScore{0.0};
        const double InvalidScore{std::nan("")};

        ros::ServiceServer placement_service;
        ros::Publisher workspace_viz;
        ros::Publisher placement_viz;
        ros::NodeHandle nh;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer;
        std::unique_ptr<tf2_ros::TransformListener> tf;

        std::unique_ptr<WorkspaceDatabase> database;
        std::unique_ptr<CollisionChecker> collision_checker;

        double position_weight = 1.0;
        double rotation_weight = 1.0;
        uint32_t viz_id = 0;

    public:
        RobotBasePlacer(ros::NodeHandle nh_, ros::NodeHandle nh_priv_);
        ~RobotBasePlacer();

    private:
        bool determineBestPlacement(rr100_ur_placement::GetPlacementRequest &req, rr100_ur_placement::GetPlacementResponse &res);
        double scorePose(
            const Eigen::Vector2d &t_current,
            const Eigen::Vector2d &t_candidate,
            const double r_current,
            const double r_candidate);
    };

} // namespace rhoban
