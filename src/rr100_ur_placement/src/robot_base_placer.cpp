#include "rr100_ur_placement/robot_base_placer.h"

#include "rr100_ur_placement/collision_checker.h"
#include "rr100_ur_placement/workspace_database.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen.h>

namespace rhoban
{
    template <typename T>
    std::vector<double> linspace(T start, T end, size_t num)
    {
        std::vector<double> space;
        double d_start = static_cast<double>(start);
        double d_end = static_cast<double>(end);
        double d_num = static_cast<double>(num);

        if (num == 0)
        {
            return space;
        }
        if (num == 1)
        {
            space.push_back(d_start);
            return space;
        }
        if (d_start > d_end)
        {
            std::swap(d_start, d_end);
        }

        double step = (d_end - d_start) / d_num - 1;
        for (size_t i = 0; i < num - 1; i++)
        {
            space.push_back(d_start + step * i);
        }
        space.push_back(d_end);

        return space;
    }

    RobotBasePlacer::RobotBasePlacer(ros::NodeHandle nh_)
        : nh(nh_)
    {
        std::string footprint_topic = nh.param<std::string>("~footprint_topic", "footprint");
        std::string map_topic = nh.param<std::string>("~map_topic", "map");

        position_weight = nh.param<double>("~position_weight", 1.0);
        rotation_weight = nh.param<double>("~rotation_weight", 1.0);

        std::string workspace_config_path;
        if (!nh.getParam("~workspace_config_path", workspace_config_path))
        {
            ROS_FATAL("Fatal error: Missing workspace configuration file");
            exit(-1);
        }
        database = std::make_unique<WorkspaceDatabase>(workspace_config_path);
        collision_checker = std::make_unique<CollisionChecker>(nh, footprint_topic, map_topic);

        placement_service = nh.advertiseService("get_placement", RobotBasePlacer::determineBestPlacement, this);
    }

    // Assume the pose is in the robot's frame
    bool RobotBasePlacer::determineBestPlacement(
        rr100_ur_placement::GetPlacementRequest &req,
        rr100_ur_placement::GetPlacementResponse &res)
    {
        // Score depends on euclidean distance from current robot pose to target poses to score
        double best_score = RobotBasePlacer::InvalidScore;

        Eigen::Affine3d target;
        tf2::fromMsg(req.target.pose, target);
        Eigen::Vector2d T_target{target.translation()};
        Eigen::Rotation2Dd R_target{target.rotation()};

        Layer layer = database->getLayer(req.target.pose.position.z);
        for (auto &&angle : linspace(-PI, PI, 16))
        {
            Eigen::Rotation2Dd r{angle};
            for (auto &&point : layer)
            {
                Eigen::Vector2d T_robot = T_target - (r * point);
                T_robot = T_robot * r;

                double score = scorePose(T_robot, r);

                if (std::isnan(best_score) || score < best_score)
                {
                    tf2::Quaternion q{0.0, 0.0, angle};
                    geometry_msgs::Pose candidate;
                    candidate.position.x = T_robot.x();
                    candidate.position.y = T_robot.y();
                    candidate.orientation.w = q.getW();
                    candidate.orientation.x = q.getX();
                    candidate.orientation.y = q.getY();
                    candidate.orientation.z = q.getZ();

                    bool is_colliding = collision_checker->isCollisionFree(req.current.pose, candidate);
                    if (is_colliding)
                    {
                        best_score = score;
                        res.best.pose.position.x = T_robot.x();
                        res.best.pose.position.y = T_robot.y();
                        res.best.pose.orientation.w = q.getW();
                        res.best.pose.orientation.x = q.getX();
                        res.best.pose.orientation.y = q.getY();
                        res.best.pose.orientation.z = q.getZ();
                    }
                }
            }
        }
        return !std::isnan(best_score);
    }

    double RobotBasePlacer::scorePose(const Eigen::Vector2d &trans, const Eigen::Rotation2Dd &rot)
    {
        return position_weight * trans.norm() + rotation_weight * rot.angle();
    }

} // namespace rhoban
