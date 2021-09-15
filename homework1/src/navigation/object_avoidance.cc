/*
 * Basic code for object detection / avoidance.
 * NOTE: This code current only works for positive curvature.
 */

#include "object_avoidance.h"

#include <cmath>
#include <math.h>

#include "ros/ros.h"
#include "visualization/visualization.h"

namespace object_avoidance {

    /*
    * 
    * @param car_specs: the struct with information about the robot's dimensions
    * @param num_paths: the number of paths to consider for planning
    * @param min_turn_radius: the minimum radius this car can turn (TODO(alex): could be in CarSpecs?)
    */
    ObjectAvoidance::ObjectAvoidance(CarSpecs car_specs,
                                     int32_t num_paths,
                                     float_t min_turn_radius) : car_specs_(car_specs) {
        // Initialize all the empty paths.
        paths_ = std::make_shared<std::vector<PathOption>>(num_paths);
        // Loop through and create intial paths options.
        int middle_index = (num_paths - 1) / 2;
        float curvature_minimum = -1 / min_turn_radius;
        float curvature_increment = -curvature_minimum / middle_index;
        float curvature_current = curvature_minimum;
        for (int i = 0; i < num_paths; i++) {
            if (i == middle_index) {
                paths_->at(i).curvature = 0.0;
            } else {
                paths_->at(i).curvature = curvature_current;
            }
            curvature_current += curvature_increment;
        }
    };

    /*
    * Function to be called by clients whenever there is a new point cloud to use to update
    * the paths.
    * 
    * @param cloud: the point cloud reading
    */
    void ObjectAvoidance::UpdatePaths(const std::vector<Eigen::Vector2f>& cloud) {
        // Loop over all the candidate paths and update their stats based on the current
        // point cloud.
        for (auto& path : *paths_) {
            // Find the longest distance that can be traversed along this curve before
            // collusion with a point
            path.free_path_length = FindMinPathLength(cloud, path.curvature);
            // Find the turn magnitude, greater value means more straight.
            path.turn_magnitude = abs((paths_->at(paths_->size() - 1).curvature - abs(path.curvature)));
            // Calculate a path's final score.
            path.score = score_max_distance_weight_ * path.free_path_length + score_min_turn_weight * 10.0 * path.turn_magnitude;
            
        }
    };

    /*
    * Function to find the longest traversable distance before collision along a given path
    * 
    * @param cloud: the point cloud reading
    * @param curvature: the path's curvature
    * 
    * @return the longest path along the given curve before any collision
    */
    float_t ObjectAvoidance::FindMinPathLength(
        const std::vector<Eigen::Vector2f>& cloud, float_t curvature) {

        // maximum distance reading by laser
        float min_path_length = 10.0;
        float path_length;
        for (const auto& point: cloud) {
            // To avoid division by zero, send zero curvature path to special linear
            // calculator.
            if (curvature == 0) {
                path_length = FindStraightPathLength(point); 
            } else {
                path_length = FindCurvePathLength(point, curvature);
            }
            // Update the longest path possible if a shorter one has been calculated.
            if (path_length < min_path_length) {
                min_path_length = path_length;
            }
        }
        return min_path_length;
    }

    /*
    * Find the longest traversable distance before collision along a straight path
    * 
    * @param point: the point to check for collision with
    * 
    * @return the longest path until collision with the point. Returns 10.0 if no
    * collision with the point
    */ 
    float_t ObjectAvoidance::FindStraightPathLength(const Eigen::Vector2f& point) {
        // First check if the y position is within the swept volume of the robot
        if (abs(point[1]) <= car_specs_.total_side) {
            // TODO(alex): Why would this be negative?
            return std::max(point[0] - car_specs_.total_front, (float_t)0.0);
        }  else {
            return 10.0;
        }
    }

    /*
    * Find the longest traversable distance before collision along a curved path
    * 
    * @param point: the point to check for collision with
    * @param curvature: the curvature of the path
    * 
    * @return the longest path until collision with the point. Returns 10.0 if no
    * collision with the point
    */ 
    float_t ObjectAvoidance::FindCurvePathLength(
        const Eigen::Vector2f& point, float_t curvature) {
        
        // If we are turning left and the point under consideration is below the bottom
        // of the car, we won't hit the point until we circle all the way back around.
        if (curvature > 0 && point[1] < -car_specs_.total_side) {
            return 10.0;
        }
        // If we are turning right and the point under consideration is above the top part
        // of the car, we won't hit the point until we circle all the way back around.
        else if (curvature < 0 && point[1] > car_specs_.total_side) {
            return 10.0;
        }
        float_t turning_radius = 1.0 / curvature;

        float_t inner_radius = abs(turning_radius) - (car_specs_.car_width / 2) - car_specs_.car_safety_margin_side;

        float_t middle_radius = sqrt(
            pow(abs(turning_radius) - (car_specs_.car_width / 2) - car_specs_.car_safety_margin_side, 2.0) + 
            pow(-car_specs_.rear_axle_offset + (car_specs_.car_length / 2.0) + car_specs_.car_safety_margin_front, 2.0));

        float_t outter_radius = sqrt(
            pow(abs(turning_radius) + car_specs_.total_side, 2.0) +
            pow(car_specs_.total_front, 2.0));


        float_t shortest_distance = 10.0;
        Eigen::Vector2f furthest_point;

        // Find the distance from the center of turning circle to point
        float_t dist_to_point = GetDistance(0, turning_radius, point[0], point[1]);

        float_t dist;
        // collision along inside part of the car
        if (inner_radius <= dist_to_point && dist_to_point < middle_radius) {
            // Find the x-coordinate of the point of collision along the car. We know the
            // y-position of this point. This works for positive and negative curvature.
            float_t x = sqrt(
                pow(dist_to_point, 2) - pow(abs(turning_radius) - car_specs_.total_side, 2));
        
            // Return the arclength between the collision point on the car and the obstacle.
            if (turning_radius < 0) {
                dist = GetDistance(
                x,
                -((0.5 * car_specs_.car_width) + car_specs_.car_safety_margin_side),
                point[0],
                point[1]) / (2 * abs(dist_to_point));
            }
            else {
                dist = GetDistance(
                x,
                (0.5 * car_specs_.car_width - car_specs_.car_safety_margin_side),
                point[0],
                point[1]) / (2 * abs(dist_to_point));
            }
            float_t arc_length = abs(2 * abs(dist_to_point) * asin(dist));

            if (arc_length <= shortest_distance) {
                shortest_distance = arc_length;
                furthest_point = point;
            }
        // collision along front of the car
        } else if (middle_radius <= dist_to_point && dist_to_point < outter_radius) {
            float_t y;
            if (turning_radius < 0) {
                y = sqrt(pow(dist_to_point, 2.0) - pow(car_specs_.total_front, 2.0)) - abs(turning_radius);
            } else {
                y = turning_radius - sqrt(pow(dist_to_point, 2.0) - pow(car_specs_.total_front, 2.0));
            }
            // Calculate the distance from the point of collision along the car to the
            // current position of the car.
            float_t dist_from_collision_to_point = GetDistance(car_specs_.total_front, y, point[0], point[1]);
            float_t angle = acos(1 - pow(dist_from_collision_to_point, 2.0) / (2 * (pow(dist_to_point, 2.0))));
            float_t arc_length = dist_to_point * angle;

            if (arc_length <= shortest_distance) {
                shortest_distance = arc_length;
                furthest_point = point;
            }
        }
        return shortest_distance;
    }

    /*
    * Find the highest scoring path
    * 
    * @return curvature of the highest scoring path
    */
    float_t ObjectAvoidance::GetHighestScorePath() {
        return std::max_element(
            paths_->begin(),
            paths_->end(),
            [](const PathOption& lhs, const PathOption& rhs){
                return lhs.score < rhs.score;}
            )->curvature;
    }

    float_t ObjectAvoidance::FindCurvePathLengthv2(const Eigen::Vector2f& point, float_t curvature) {
        float r = 1 / abs(curvature);
        Eigen::Vector2f turn_point = Eigen::Vector2f(0.0, r * int(curvature / abs(curvature)));

        // if (point.x() < 0.0) {
        //     return 10.0; // don't consider points behind car
        // }

        float r_min = r - car_specs_.car_width/2.0 - car_specs_.car_safety_margin_side;
        float r_mid = sqrt(pow(r_min, 2) + pow(-car_specs_.rear_axle_offset + (car_specs_.car_length / 2.0) + car_specs_.car_safety_margin_front, 2)); 
        float r_max = sqrt(pow(r + (car_specs_.car_width / 2.0) + car_specs_.car_safety_margin_side, 2) + pow(-car_specs_.rear_axle_offset + (car_specs_.car_length / 2.0) + car_specs_.car_safety_margin_front, 2)); 
        float r_obs = GetDistance(turn_point, point);

        float Beta;
        float alpha;
        if ((r_min <= r_obs) && (r_obs < r_mid)){ // point will hit the left side of car
            Beta = acos(r - (car_specs_.car_width / 2.0) - car_specs_.car_safety_margin_side / r_obs);
        } else if ((r_mid <= r_obs) && (r_obs <= r_max)) {  // point will hit front of car
            Beta = asin((- car_specs_.rear_axle_offset + (car_specs_.car_length / 2.0) + car_specs_.car_safety_margin_front)/ r_obs);
        } else{ // else point doesn't hit car
            return 10.0;
        }
        float dist = GetDistance(point, Eigen::Vector2f(0.0, 0.0));
        alpha =  acos((pow(dist, 2) - pow(r, 2) - pow(r_obs, 2))/(2 * r * r_obs)) - Beta;
        return alpha * r; // alpha in radians to path length
    }
};
