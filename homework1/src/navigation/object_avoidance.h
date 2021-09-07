#ifndef __SRC_NAVIGATION_OBJECT_AVOIDANCE__
#define __SRC_NAVIGATION_OBJECT_AVOIDANCE__

#include <vector>

#include "eigen3/Eigen/Dense"


namespace object_avoidance {

    // Loop through all points in point cloud and determine
    // which points the car will collide with for given arc.
    std::vector<Eigen::Vector2f> FindCollisionPoints(
        std::vector<Eigen::Vector2f> point_cloud,
        float_t turning_radius,
        float_t width,
        float_t length,
        float_t wheelbase);

    // Loop through all the collision points and return the distance with
    // which has the smallest arc length from car to point. Pass this new
    // distance to the TOC function to find change in velocity.
    float_t FindShortestDistance(
        std::vector<Eigen::Vector2f> point_cloud, float_t turning_radius);
    
}



#endif