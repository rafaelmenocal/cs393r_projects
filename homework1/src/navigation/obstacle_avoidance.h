#ifndef __SRC_NAVIGATION_OBSTACLE_AVOIDANCE__
#define __SRC_NAVIGATION_OBSTACLE_AVOIDANCE__

#include <vector>

#include "eigen3/Eigen/Dense"
#include "amrl_msgs/VisualizationMsg.h"


namespace obstacle_avoidance {

    struct shortest_dist {
        Eigen::Vector2f furthest_point;
        float_t free_path_length;
        float_t average_distance;
    };

    // Loop through all points in point cloud and determine which points the car will
    // collide with for given arc.
    std::vector<Eigen::Vector2f> FindCollisionPoints(
        const std::vector<Eigen::Vector2f>& point_cloud,
        float_t turning_radius,
        float_t width,
        float_t length,
        float_t wheelbase);

    // Loop through all the collision points and return the distance with which has the
    // smallest arc length from car to point. Pass this new distance to the TOC function
    // to find change in velocity.
    shortest_dist FindShortestDistance(
        const std::vector<Eigen::Vector2f>& point_cloud,
        float_t turning_radius,
        float_t width,
        float_t length,
        float_t wheelbase,
        amrl_msgs::VisualizationMsg& msg);

    inline float_t GetDistance(float_t x0, float_t y0, float_t x1, float_t y1) {
        return sqrt(pow(x1 - x0, 2.0) + pow(y1 - y0, 2.0));
    };
} // namespace obstacle_avoidance

#endif