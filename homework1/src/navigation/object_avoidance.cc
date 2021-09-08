/*
 * Basic code for object detection / avoidance.
 * NOTE: This code current only works for positive curvature.
 */

#include <cmath>
#include <math.h>

#include "object_avoidance.h"


namespace object_avoidance {

    std::vector<Eigen::Vector2f> FindCollisionPoints(
        std::vector<Eigen::Vector2f> point_cloud, float_t turning_radius,
        float_t width, float_t length, float_t wheelbase)
    {
        // Calculate the bounding radii which define the swept area the robot will move
        // through along this arc.
        float_t inner_radius = turning_radius - (width / 2);
        float_t outter_radius = GetDistance(-(width / 2), -wheelbase,
                                            turning_radius, (length - wheelbase) / 2.0);

        std::vector<Eigen::Vector2f> collision_points;
        for (const auto& point : point_cloud) {
            float_t distance = GetDistance(0, turning_radius, point[0], point[1]);
            if (inner_radius <= distance && distance <= outter_radius) {
                collision_points.push_back(point);
            }
        }
        return collision_points;
    };

    float_t FindShortestDistance(
        std::vector<Eigen::Vector2f> point_cloud,
        float_t turning_radius,
        float_t width,
        float_t length,
        float_t wheelbase)
    {
        float_t inner_radius = turning_radius - (width / 2);
        float_t middle_radius = GetDistance((width / 2), -(length - wheelbase) / 2,
                                            turning_radius, wheelbase);
        float_t outter_radius = GetDistance(-(width / 2), -wheelbase, turning_radius + (width / 2),
                                            (length - wheelbase) / 2.0);

        for (const auto& point : point_cloud) {
            float_t distance = GetDistance(0, turning_radius, point[0], point[1]);
            // collision along inside part of the car
            if (inner_radius <= distance && distance < middle_radius) {
                float_t x = GetDistance(0, (0.5 * width), distance, turning_radius);
                return 2 * distance * asin(
                    object_avoidance::GetDistance(x, width / 2, point[0], point[1]) / (2 * turning_radius));
            // collision along front of the car
            } else if (middle_radius <= distance && distance < outter_radius) {
                float_t y = -(GetDistance(0, -(width - wheelbase) / 2, distance, wheelbase) - turning_radius);
                return 2 * distance * asin(
                    object_avoidance::GetDistance(wheelbase + (width - wheelbase) / 2, y, point[0], point[1]) / (2 * turning_radius));
            }
        }

        return 0.0;
    };
    
}