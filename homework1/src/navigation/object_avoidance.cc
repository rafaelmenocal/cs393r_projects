#include <cmath>
#include <iostream>
#include <math.h>

#include "object_avoidance.h"


namespace object_avoidance {

    std::vector<Eigen::Vector2f> FindCollisionPoints(
        std::vector<Eigen::Vector2f> point_cloud,
        float_t turning_radius,
        float_t width,
        float_t length,
        float_t wheelbase)
    {
        float_t inner_radius = turning_radius - (width / 2);
        float_t outter_radius = sqrt(
           pow(turning_radius + (width / 2), 2.0) + pow((length - wheelbase) / 2.0 + wheelbase, 2)
        );

        std::vector<Eigen::Vector2f> collision_points;
        for (const auto& point : point_cloud) {
            float_t distance = sqrt(
                pow(point[0], 2.0) + pow(point[1] - turning_radius, 2));
            if (inner_radius <= distance && distance <= outter_radius) {
                collision_points.push_back(point);
            }
        }
        return collision_points;
        
    };

    // Take in a collection of points and find which point the car will collide
    // with first. Along a given arc, the car will collide with the point either
    // on the side facing the center or turning or the front of the car.
    float_t FindShortestDistance(
        std::vector<Eigen::Vector2f> point_cloud,
        float_t turning_radius,
        float_t width,
        float_t length,
        float_t wheelbase)
    {
        float_t inner_radius = turning_radius - (width / 2);
        float_t middle_radius = sqrt(
                pow(turning_radius - (width / 2), 2.0) +
                pow(wheelbase + (length - wheelbase) / 2, 2));
        float_t outter_radius = sqrt(
                pow(turning_radius + (width / 2), 2.0) + 
                pow((length - wheelbase) / 2.0 + wheelbase, 2)
        );
        for (const auto& point : point_cloud) {
            float_t distance = sqrt(
                pow(point[0], 2.0) + pow(point[1] - turning_radius, 2));
            // collision along inside part of the car
            if (inner_radius <= distance && distance < middle_radius) {
                float_t x = sqrt(pow(distance, 2) - pow(turning_radius - (0.5 * width), 2));
                return 2 * distance * asin(
                    object_avoidance::GetDistance(x, width / 2, point[0], point[1]) / (2 * turning_radius));
            // collision along front of the car
            } else if (middle_radius <= distance && distance < outter_radius) {
                float_t y = -(sqrt(pow(distance, 2) - pow(wheelbase + (width - wheelbase) / 2, 2)) - turning_radius);
                return 2 * distance * asin(
                    object_avoidance::GetDistance(wheelbase + (width - wheelbase) / 2, y, point[0], point[1]) / (2 * turning_radius));
            }
        }

        return 0.0;
    };
    
}