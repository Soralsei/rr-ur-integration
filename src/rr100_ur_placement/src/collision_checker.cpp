#include <algorithm>
#include <Eigen/Dense>

#include "rr100_ur_placement/collision_checker.h"
#include "rr100_ur_placement/map_subscriber.h"
#include "rr100_ur_placement/footprint_subscriber.h"
#include "rr100_ur_placement/cost_values.h"
#include "rr100_ur_placement/footprint.h"

#include "rr100_ur_placement/util/exceptions.h"
#include "rr100_ur_placement/util/format.h"
#include "rr100_ur_placement/util/line_iterator.h"

#include "tf2/utils.h"
#include "costmap_2d/cost_values.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"

namespace rhoban
{
    CollisionChecker::CollisionChecker(
        ros::NodeHandle &node,
        const std::string &footprint_topic,
        const std::string &map_topic)
    {
        mFootprintSubscriber = std::make_unique<FootprintSubscriber>(node, footprint_topic);
        mMapSubscriber = std::make_unique<MapSubscriber>(node, map_topic);

        mCollisionViz = node.advertise<visualization_msgs::Marker>("collision_visualisation", 10);
    }

    CollisionChecker::~CollisionChecker() {}

    double CollisionChecker::scorePose(const PoseMsg &currentPose, const PoseMsg &targetPose)
    {
        try
        {
            mMap = mMapSubscriber->getMap();
        }
        catch (const NoMapReceivedException &e)
        {
            throw e;
        }

        const Eigen::Vector2f currentPosition{currentPose.pose.position.x, currentPose.pose.position.y};
        const Eigen::Vector2f targetPosition{targetPose.pose.position.x, targetPose.pose.position.y};

        const float currentTheta = tf2::getYaw(currentPose.pose.orientation);
        const float targetTheta = tf2::getYaw(targetPose.pose.orientation);

        const Footprint footprint = getFootprintAt(currentPosition, currentTheta, targetPosition, targetTheta);

        try
        {
            worldToMap(targetPosition[0], targetPosition[1]);
        }
        catch (const std::exception &e)
        {
            return static_cast<double>(LETHAL_COST);
        }

        return footprintCost(footprint);
    }

    bool CollisionChecker::isCollisionFree(const PoseMsg &currentPose, const PoseMsg &targetPose)
    {
        try
        {
            return scorePose(currentPose, targetPose) != LETHAL_COST;
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("%s", e.what());
        }
        return false;
    }

    Footprint CollisionChecker::getFootprintAt(Eigen::Vector2f position, float theta, Eigen::Vector2f targetPosition, float targetTheta)
    {
        Footprint footprint = mFootprintSubscriber->getFootprintInRobotFrame(position, theta);
        return transformFootprint(footprint, targetPosition, targetTheta);
    }

    Eigen::Vector2i CollisionChecker::worldToMap(const float x, const float y) const
    {
        int width = mMap->info.width;
        int height = mMap->info.height;
        float resolution = mMap->info.resolution;
        auto mapOrigin = mMap->info.origin.position;

        Eigen::Vector2f coords{x, y};
        Eigen::Vector2f origin{mapOrigin.x, mapOrigin.y};

        // ROS_INFO("Converting coordinates %.2f|%.2f to map coordinates...", x, y);

        Eigen::Vector2i cell = ((coords - origin) / resolution).cast<int>();

        if (cell[0] >= width || cell[0] < 0)
        {
            throw IllegalPoseException(format("X coordinate %.2f is outside of map grid", x).c_str());
        }
        if (cell[1] >= height || cell[1] < 0)
        {
            throw IllegalPoseException(format("Y coordinate %.2f is outside of map grid", y).c_str());
        }

        return cell;
    }

    Eigen::Vector2f CollisionChecker::mapToWorld(const int x, const int y) const
    {
        int width = mMap->info.width;
        int height = mMap->info.height;
        float resolution = mMap->info.resolution;
        auto mapOrigin = mMap->info.origin.position;

        Eigen::Vector2f origin{mapOrigin.x, mapOrigin.y};
        Eigen::Vector2f coords{x, y};

        Eigen::Vector2f cell = coords * resolution + origin;

        if (cell[0] >= width || cell[0] < 0)
        {
            throw IllegalPoseException(format("X coordinate %d is outside of map grid", x).c_str());
        }
        if (cell[1] >= height || cell[1] < 0)
        {
            throw IllegalPoseException(format("Y coordinate %d is outside of map grid", y).c_str());
        }

        return cell;
    }

    double CollisionChecker::footprintCost(const Footprint &footprint) const
    {
        double cost = 0.0;
        size_t count = footprint.size();

        for (size_t i = 0, j = count - 1; i < count; j = i++)
        {
            try
            {
                Eigen::Vector2i start = worldToMap(footprint[j].x, footprint[j].y);
                Eigen::Vector2i end = worldToMap(footprint[i].x, footprint[i].y);
                // ROS_INFO("Inspecting line (%d, %d) -> (%d, %d)", start[0], start[1], end[0], end[1]);
                cost = std::max(cost, lineCost(start, end));
                // ROS_INFO("cost : %.2f", cost);
            }
            catch (const std::exception &e)
            {
                ROS_ERROR("%s", e.what());
                return static_cast<double>(LETHAL_COST);
            }

            if (cost == LETHAL_COST)
            {
                return cost;
            }
        }

        auto bounds = getFootprintBounds(footprint);
        Eigen::Vector2i min;
        Eigen::Vector2i max;
        try
        {
            min = worldToMap(bounds[0], bounds[2]);
            max = worldToMap(bounds[1], bounds[3]);
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("%s", e.what());
            return static_cast<double>(LETHAL_COST);
        }
        for (size_t y = min[1]; y < max[1]; y++)
        {
            for (size_t x = min[0]; x < max[0]; x++)
            {
                double pcost = pointCost(x, y);
                if (pcost > 0 && isInsideFootprint(footprint, mapToWorld(x, y)))
                {
                    if (pcost == LETHAL_COST)
                    {
                        return static_cast<double>(LETHAL_COST);
                    }
                    cost = std::max(cost, pcost);
                }
            }
        }

        return cost;
    }

    double CollisionChecker::lineCost(const Eigen::Vector2i &start, const Eigen::Vector2i &end) const
    {
        double cost = 0.0;
        static uint32_t viz_id = 0;
        for (LineIterator line{start, end}; !line.endReached(); line.next())
        {
            int x, y;
            x = line.getX();
            y = line.getY();
            double currentCost = pointCost(x, y);
            cost = std::max(cost, currentCost);
            if (cost == LETHAL_COST)
            {
                visualization_msgs::Marker marker;
                geometry_msgs::Pose pose;
                auto point = mapToWorld(x, y);
                pose.position.x = point.x();
                pose.position.y = point.y();
                marker.header.frame_id = "map";
                marker.header.stamp = ros::Time();
                marker.ns = "collision_markers";
                marker.id = viz_id++;
                marker.frame_locked = true;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose = pose;
                marker.scale.x = 0.2;
                marker.scale.y = 0.2;
                marker.scale.z = 0.2;
                marker.color.a = 1.0;
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                mCollisionViz.publish(marker);
                return static_cast<double>(LETHAL_COST);
            }
        }
        return cost;
    }

    inline double CollisionChecker::pointCost(int x, int y) const
    {
        int cost = mMap->data[y * mMap->info.width + x];
        // ROS_INFO("Inspecting point (%d, %d), cost : %d", x, y, cost);
        return cost;
    }

} // namespace rhoban
