#include <algorithm>

#include "rr100_ur_pose_projection/collision_checker.h"
#include "rr100_ur_pose_projection/map_subscriber.h"
#include "rr100_ur_pose_projection/footprint_subscriber.h"
#include "rr100_ur_pose_projection/cost_values.h"
#include "rr100_ur_pose_projection/footprint.h"

#include "rr100_ur_pose_projection/util/exceptions.h"
#include "rr100_ur_pose_projection/util/format.h"
#include "rr100_ur_pose_projection/util/line_iterator.h"

#include "tf2/utils.h"
#include "costmap_2d/cost_values.h"
#include "ros/ros.h"

namespace rhoban
{
    CollisionChecker::CollisionChecker(
        ros::NodeHandle &node,
        const std::string &footprint_topic,
        const std::string &map_topic)
    {
        mFootprintSubscriber = std::make_unique<FootprintSubscriber>(node, footprint_topic);
        mMapSubscriber = std::make_unique<MapSubscriber>(node, map_topic);
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

        const math::Vector2<float> currentPosition{currentPose.position.x, currentPose.position.y};
        const math::Vector2<float> targetPosition{targetPose.position.x, targetPose.position.y};

        const float currentTheta = tf2::getYaw(currentPose.orientation);
        const float targetTheta = tf2::getYaw(targetPose.orientation);

        const Footprint footprint = getFootprintAt(currentPosition, currentTheta, targetPosition, targetTheta);

        try
        {
            worldToMap(targetPosition.x, targetPosition.y);
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
            return scorePose(currentPose, targetPose) >= 0.0;
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
        return false;
    }

    Footprint CollisionChecker::getFootprintAt(math::Vector2<float> position, float theta, math::Vector2<float> targetPosition, float targetTheta)
    {
        Footprint footprint = mFootprintSubscriber->getFootprintInRobotFrame(position, theta);
        return transformFootprint(footprint, targetPosition, targetTheta);
    }

    math::Vector2<int> CollisionChecker::worldToMap(const float x, const float y) const
    {
        int width = mMap->info.width;
        int height = mMap->info.height;

        float resolution = mMap->info.resolution;

        auto origin = mMap->info.origin.position;

        int cellX = (x - origin.x) / resolution;
        int cellY = (y - origin.y) / resolution;

        if (cellX >= width || cellX < 0)
        {
            throw IllegalPoseException(format("X coordinate %.2f is outside of map grid", x).c_str());
        }
        if (cellY >= height || cellY < 0)
        {
            throw IllegalPoseException(format("Y coordinate %.2f is outside of map grid", y).c_str());
        }

        return math::Vector2<int>{cellX, cellY};
    }

    double CollisionChecker::footprintCost(const Footprint &footprint) const
    {
        double cost = 0.0;
        size_t count = footprint.size();

        for (size_t i = 0, j = count - 1; i < count; j = i++)
        {
            try
            {
                math::Vector2<int> start = worldToMap(footprint[j].x, footprint[j].y);
                math::Vector2<int> end = worldToMap(footprint[i].x, footprint[i].y);
                ROS_INFO("Inspecting line (%d, %d) -> (%d, %d), cost : %.2f", start.x, start.y, end.x, end.y, cost);
                cost = std::max(cost, lineCost(start, end));
            }
            catch (const std::exception &e)
            {
                ROS_ERROR("%s", e.what());
                return static_cast<double>(LETHAL_COST);
            }

            if (cost == LETHAL_COST)
            {
                break;
            }
        }

        return cost;
    }

    double CollisionChecker::lineCost(const math::Vector2<int> &start, const math::Vector2<int> &end) const
    {
        double cost = 0.0;
        for (LineIterator line{start, end}; !line.endReached(); line.next())
        {
            double currentCost = pointCost(line.getCurrent());

            cost = std::max(cost, currentCost);
            if (cost == LETHAL_COST)
            {
                return static_cast<double>(LETHAL_COST);
            }
        }
        return cost;
    }

    inline double CollisionChecker::pointCost(const math::Vector2<int> &point) const
    {
        int cost = mMap->data[point.y * mMap->info.width + point.x];
        ROS_INFO("Inspecting point (%d, %d), cost : %d", point.x, point.y, cost);
        return cost;
    }

} // namespace rhoban
