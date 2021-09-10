/*
 * Basic code for obstacle detection / avoidance.
 * NOTE: This code current only works for positive curvature.
 */

#include <cmath>
#include <math.h>
#include <iostream>

#include "obstacle_avoidance.h"
#include "visualization/visualization.h"


namespace obstacle_avoidance {

    std::vector<Eigen::Vector2f> FindCollisionPoints(
        const std::vector<Eigen::Vector2f>& point_cloud, float_t turning_radius,
        float_t width, float_t length, float_t wheelbase)
    {
        float_t sign = -1 ? (turning_radius < 0) : 1;
        // Calculate the bounding radii which define the swept area the robot will move
        // through along this arc. This function works for both negative and positive
        // turning radii.
        float_t inner_radius = abs(turning_radius - sign * (width / 2));
        float_t outter_radius = GetDistance(sign * (width / 2), 0,
                                            turning_radius, (length + wheelbase / 2.0));

        std::vector<Eigen::Vector2f> collision_points;
        for (const auto& point : point_cloud) {
            float_t distance = GetDistance(0, turning_radius, point[0], point[1]);
            if (inner_radius <= distance && distance <= outter_radius) {
                collision_points.push_back(point);
            }
        }
        return collision_points;
    };

    shortest_dist FindShortestDistance(
        const std::vector<Eigen::Vector2f>& point_cloud,
        float_t turning_radius,
        float_t width,
        float_t length,
        float_t wheelbase,
        amrl_msgs::VisualizationMsg& msg)
    {   
        float_t inner_radius = abs(turning_radius) - (width / 2);

        float_t middle_radius = sqrt(
            pow(abs(turning_radius) - (width / 2), 2.0) + 
            pow(wheelbase + (length - wheelbase) / 2, 2.0));
    
        float_t outter_radius = sqrt(
            pow(abs(turning_radius) + (width / 2), 2.0) +
            pow(wheelbase + (length - wheelbase) / 2, 2.0));

        float_t shortest_distance = 10.0;
        Eigen::Vector2f furthest_point;

        for (const auto& point : point_cloud) {
            float_t distance = GetDistance(0, abs(turning_radius), point[0], point[1]);
            // collision along inside part of the car
            if (inner_radius <= distance && distance < middle_radius) {
                float_t x = sqrt(pow(distance, 2) - pow(turning_radius - (0.5 * width), 2));
                // Return the arclength between the collision point on the car and the obstacle.
                float_t arc_length = abs(2 * distance * asin(
                    GetDistance(x, width / 2, point[0], point[1]) / (2 * turning_radius)));

                if (arc_length <= shortest_distance) {
                    shortest_distance = arc_length;
                    furthest_point = point;
                }

            // collision along front of the car
            } else if (middle_radius <= distance && distance < outter_radius) {
                float_t y = -(GetDistance(0, -(width - wheelbase) / 2, distance, wheelbase) - turning_radius);
                // Return the arclength between the collision point on the car and the obstacle.
                float_t arc_length = abs(2 * distance * asin(
                    GetDistance(wheelbase + (width - wheelbase) / 2, y, point[0], point[1]) / (2 * turning_radius)));
                if (arc_length <= shortest_distance) {
                    shortest_distance = arc_length;
                    furthest_point = point;
                }
            }
        }
        return shortest_dist{furthest_point, shortest_distance};
    };

}