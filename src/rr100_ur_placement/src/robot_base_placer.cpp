#include "rr100_ur_placement/robot_base_placer.h"

#include "rr100_ur_placement/collision_checker.h"
#include "rr100_ur_placement/workspace_database.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2/utils.h"

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

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

        double step = (d_end - d_start) / (d_num - 1);

        for (size_t i = 0; i < num - 1; i++)
        {
            space.push_back(d_start + step * i);
        }
        space.push_back(d_end);

        return space;
    }

    RobotBasePlacer::RobotBasePlacer(ros::NodeHandle nh_, ros::NodeHandle nh_priv_)
        : nh(nh_)
    {
        std::string footprint_topic = nh_priv_.param<std::string>("footprint_topic", "footprint");
        std::string map_topic = nh_priv_.param<std::string>("map_topic", "map");

        tf_buffer = std::make_shared<tf2_ros::Buffer>();
        tf = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

        position_weight = nh_priv_.param<double>("position_weight", 1.0);
        rotation_weight = nh_priv_.param<double>("rotation_weight", 1.0);

        std::string workspace_config_path;
        if (!nh_priv_.getParam("workspace_config_path", workspace_config_path))
        {
            ROS_FATAL("Fatal error: Missing workspace configuration file");
            exit(-1);
        }
        database = std::make_unique<WorkspaceDatabase>(workspace_config_path);
        collision_checker = std::make_unique<CollisionChecker>(nh, footprint_topic, map_topic);

        placement_viz = nh.advertise<visualization_msgs::Marker>("rr100_placement_viz", 0, false);

        workspace_viz = nh.advertise<visualization_msgs::MarkerArray>("ur_workspace_points", 0, true);
        visualization_msgs::MarkerArray workspace_points;
        auto layers = database->getAllLayers();
        auto heights = database->getAllHeights();
        for (std::size_t i = 0; i < layers.size(); i++)
        {
            auto layer = *layers.at(i);
            for (size_t j = 0; j < layer.size(); j++)
            {
                visualization_msgs::Marker workspace_point;
                workspace_point.header.frame_id = "base_footprint";
                workspace_point.header.stamp = ros::Time();
                workspace_point.ns = "ur_workspace";
                workspace_point.id = i * layer.size() + j;
                workspace_point.frame_locked = true;
                workspace_point.type = visualization_msgs::Marker::SPHERE;
                workspace_point.action = visualization_msgs::Marker::ADD;
                workspace_point.pose.position.x = layer[j](0);
                workspace_point.pose.position.y = layer[j](1);
                workspace_point.pose.position.z = heights[i];
                workspace_point.pose.orientation.x = 0.0;
                workspace_point.pose.orientation.y = 0.0;
                workspace_point.pose.orientation.z = 0.0;
                workspace_point.pose.orientation.w = 1.0;
                workspace_point.scale.x = 0.02;
                workspace_point.scale.y = 0.02;
                workspace_point.scale.z = 0.02;
                workspace_point.color.a = 1.0; // Don't forget to set the alpha!
                workspace_point.color.r = 1.0;
                workspace_point.color.g = 0.65;
                workspace_point.color.b = 0.0;
                workspace_points.markers.push_back(workspace_point);
            }
        }
        workspace_viz.publish(workspace_points);

        placement_service = nh_priv_.advertiseService("get_placement", &RobotBasePlacer::determineBestPlacement, this);
    }

    RobotBasePlacer::~RobotBasePlacer() {}

    // Assume the pose is in the robot's frame
    bool RobotBasePlacer::determineBestPlacement(
        rr100_ur_placement::GetPlacementRequest &req,
        rr100_ur_placement::GetPlacementResponse &res)
    {
        // Score depends on euclidean distance from current robot pose to target poses to score
        double best_score = RobotBasePlacer::InvalidScore;

        geometry_msgs::PoseStamped target_pose, current_pose;
        try
        {
            auto target_to_map = tf_buffer->lookupTransform("map", req.target.header.frame_id, ros::Time(0.0));
            auto current_to_map = tf_buffer->lookupTransform("map", req.current.header.frame_id, ros::Time(0.0));
            tf2::doTransform(req.target, target_pose, target_to_map);
            tf2::doTransform(req.current, current_pose, current_to_map);
        }
        catch (const std::exception &e)
        {
            ROS_ERROR_STREAM("Transform lookup error : " << e.what());
            return false;
        }

        visualization_msgs::Marker marker;
        marker.header.frame_id = req.target.header.frame_id;
        marker.header.stamp = ros::Time();
        marker.ns = "placement_goal";
        marker.id = 0;
        marker.frame_locked = true;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = req.target.pose;
        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.12;
        marker.color.b = 1.0;
        marker.lifetime = ros::Duration(60.0);
        placement_viz.publish(marker);
        // viz_id++;

        double yaw_current = tf2::getYaw(current_pose.pose.orientation);

        Eigen::Vector2d T_current{current_pose.pose.position.x, current_pose.pose.position.y};
        Eigen::Vector2d T_target{target_pose.pose.position.x, target_pose.pose.position.y};

        Layer layer = database->getLayer(target_pose.pose.position.z);

        for (auto &&angle : linspace(-PI, PI, 16))
        {
            Eigen::Rotation2Dd r_candidate{angle};
            for (auto &&point : layer)
            {
                Eigen::Vector2d T_candidate = T_target - (r_candidate * point);
                double score = scorePose(T_current, T_candidate, yaw_current, angle);
                if (std::isnan(best_score) || score < best_score)
                {
                    tf2::Quaternion q{tf2::Vector3{0.0, 0.0, 1.0}, angle};
                    geometry_msgs::PoseStamped candidate;
                    candidate.pose.position.x = T_candidate.x();
                    candidate.pose.position.y = T_candidate.y();
                    candidate.pose.orientation.w = q.getW();
                    candidate.pose.orientation.x = q.getX();
                    candidate.pose.orientation.y = q.getY();
                    candidate.pose.orientation.z = q.getZ();

                    bool not_colliding = collision_checker->isCollisionFree(req.current, candidate);
                    ROS_DEBUG_STREAM("Candidate pose : " << T_candidate
                        << ", theta : " << angle 
                        << ", is collision free : " << not_colliding);

                    if (not_colliding)
                    {
                        best_score = score;
                        res.best.header.frame_id = "map";
                        res.best.pose.position.x = T_candidate.x();
                        res.best.pose.position.y = T_candidate.y();
                        res.best.pose.orientation.w = q.getW();
                        res.best.pose.orientation.x = q.getX();
                        res.best.pose.orientation.y = q.getY();
                        res.best.pose.orientation.z = q.getZ();
                    }
                }
            }
        }
        bool succeeded = !std::isnan(best_score);
        if (succeeded)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "placement_result";
            marker.id = 0;
            marker.frame_locked = true;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = res.best.pose;
            marker.scale.x = 1.0;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.85;
            marker.color.g = 0.85;
            marker.color.b = 0.07;
            marker.lifetime = ros::Duration(60.0);
            placement_viz.publish(marker);
            // viz_id++;
        }
        return succeeded;
    }

    double RobotBasePlacer::scorePose(
        const Eigen::Vector2d &t_current,
        const Eigen::Vector2d &t_candidate,
        const double yaw_current,
        const double yaw_candidate)
    {
        double position_score = position_weight * (t_candidate - t_current).norm();
        double orientation_score = rotation_weight * std::abs(yaw_candidate - yaw_current);
        ROS_INFO("Position score : %.4f | Orientation score : %.4f", position_score, orientation_score);
        return position_score + orientation_score; 
    }

} // namespace rhoban
