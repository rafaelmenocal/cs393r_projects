/*
 * Basic code for object detection / avoidance.
 * NOTE: This code current only works for positive curvature.
 */

#include "object_avoidance.h"

#include <cmath>
#include <math.h>


namespace object_avoidance {


    ObjectAvoidance::ObjectAvoidance(CarSpecs car_specs,
                                     int32_t num_paths,
                                     float_t min_turn_radius) : car_specs_(car_specs) {
        
        paths_ = std::make_shared<std::vector<PathOption>>(num_paths);
        // Create intial paths options.
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

    // returns the minimum path length to all points in a point cloud on a given curve
    float_t ObjectAvoidance::FindMinPathLength(const std::vector<Eigen::Vector2f>& cloud, float_t curvature){
        float min_path_length = 10.0; // maximum distance reading by laser
        float path_length;
        for (const auto& point: cloud){
            if (curvature == 0){
            // avoid divide by zero math
            path_length = FindStraightPathLength(point); 
            } else {
            path_length = FindCurvePathLength(point, curvature);
            }
            if (path_length < min_path_length) {
            min_path_length = path_length;
            }
        }
        return min_path_length;
    }

    // Given a single point and vehicle dimensions, return straight path length  
    float_t ObjectAvoidance::FindStraightPathLength(const Eigen::Vector2f& point) {
        if (abs(point[1]) <= car_specs_.car_length/2.0 + car_specs_.car_safety_margin_side) {
            return point[0] - (-car_specs_.rear_axle_offset + car_specs_.car_length/2.0 + car_specs_.car_safety_margin_front);
        }  else {
            return 10.0;
        }
    }

    // Given a single point, vehicle dimensions, and a curvature, return path length 
    float_t ObjectAvoidance::FindCurvePathLength(const Eigen::Vector2f& point, float curvature) {

        if (curvature > 0 && point[1] < -((car_specs_.car_width / 2.0) + car_specs_.car_safety_margin_side)) {
            return 10.0;
        }
        else if (curvature < 0 && point[1] > ((car_specs_.car_width / 2.0) + car_specs_.car_safety_margin_side)) {
            return 10.0;
        }

        float_t turning_radius = 1 / curvature;

        float_t inner_radius = abs(turning_radius) - (car_specs_.car_width / 2) - car_specs_.car_safety_margin_side;

        float_t middle_radius = sqrt(
            pow(abs(turning_radius) - (car_specs_.car_width / 2) - car_specs_.car_safety_margin_side, 2.0) + 
            pow(car_specs_.rear_axle_offset + car_specs_.car_length + car_specs_.car_safety_margin_front, 2.0));

        float_t outter_radius = sqrt(
            pow(abs(turning_radius) + (car_specs_.car_width / 2.0) + car_specs_.car_safety_margin_side, 2.0) +
            pow(car_specs_.rear_axle_offset + car_specs_.car_length + car_specs_.car_safety_margin_front, 2.0));


        float_t shortest_distance = 10.0;
        Eigen::Vector2f furthest_point;

        float_t distance = GetDistance(0, turning_radius, point[0], point[1]);

        float_t dist;
        // collision along inside part of the car
        if (inner_radius <= distance && distance < middle_radius) {
            float_t x = sqrt(
                pow(distance, 2) -
                pow(abs(turning_radius) - (0.5 * car_specs_.car_width - car_specs_.car_safety_margin_side), 2));
        
            // Return the arclength between the collision point on the car and the obstacle.
            if (turning_radius < 0) {
                dist = GetDistance(
                x,
                -((0.5 * car_specs_.car_width) + car_specs_.car_safety_margin_side),
                point[0],
                point[1]) / (2 * abs(distance));
            }
            else {
                dist = GetDistance(
                x,
                (0.5 * car_specs_.car_width - car_specs_.car_safety_margin_side),
                point[0],
                point[1]) / (2 * abs(distance));
            }
            float_t arc_length = abs(2 * abs(distance) * asin(dist));

            if (arc_length <= shortest_distance) {
                shortest_distance = arc_length;
                furthest_point = point;
            }
        // collision along front of the car
        } else if (middle_radius <= distance && distance < outter_radius) {
            float_t y;
            if (turning_radius < 0) {
                y = sqrt(pow(distance, 2.0) -
                    pow(car_specs_.rear_axle_offset + car_specs_.car_length + car_specs_.car_safety_margin_front, 2.0)
                ) - abs(turning_radius);
            } else {
                y = turning_radius - sqrt(pow(distance, 2.0) -
                    pow(car_specs_.rear_axle_offset + car_specs_.car_length + car_specs_.car_safety_margin_front, 2.0)
                );
            }
            dist = GetDistance(
                car_specs_.rear_axle_offset + car_specs_.car_length + car_specs_.car_safety_margin_front,
                y,
                point[0],
                point[1]);

            float_t arc_length = abs(2 * distance * asin(dist / (2 * distance)));
            float_t angle = acos(1 - pow(dist, 2.0) / (2 * (pow(distance, 2.0))));
                
            
            arc_length = distance * angle;
            if (arc_length <= shortest_distance) {
                shortest_distance = arc_length;
                furthest_point = point;
            }
        }
        return shortest_distance;
    }

    float_t ObjectAvoidance::GetHighestScorePath() {
        return std::max_element(
            paths_->begin(),
            paths_->end(),
            [](const PathOption& lhs, const PathOption& rhs){
                return lhs.score < rhs.score;}
            )->curvature;
    }
    // std::vector<Eigen::Vector2f> FindCollisionPoints(
    //     std::vector<Eigen::Vector2f> point_cloud, float_t turning_radius,
    //     float_t width, float_t length, float_t wheelbase)
    // {
    //     // Calculate the bounding radii which define the swept area the robot will move
    //     // through along this arc.
    //     float_t inner_radius = turning_radius - (width / 2);
    //     float_t outter_radius = GetDistance(-(width / 2), -wheelbase,
    //                                         turning_radius, (length - wheelbase) / 2.0);

    //     std::vector<Eigen::Vector2f> collision_points;
    //     for (const auto& point : point_cloud) {
    //         float_t distance = GetDistance(0, turning_radius, point[0], point[1]);
    //         if (inner_radius <= distance && distance <= outter_radius) {
    //             collision_points.push_back(point);
    //         }
    //     }
    //     return collision_points;
    // };

    // float_t FindShortestDistance(
    //     std::vector<Eigen::Vector2f> point_cloud,
    //     float_t turning_radius,
    //     float_t width,
    //     float_t length,
    //     float_t wheelbase)
    // {
    //     float_t inner_radius = turning_radius - (width / 2);
    //     float_t middle_radius = GetDistance((width / 2), -(length - wheelbase) / 2,
    //                                         turning_radius, wheelbase);
    //     float_t outter_radius = GetDistance(-(width / 2), -wheelbase, turning_radius + (width / 2),
    //                                         (length - wheelbase) / 2.0);

    //     float_t shortest_distance = 10000000;

    //     for (const auto& point : point_cloud) {
    //         float_t distance = GetDistance(0, turning_radius, point[0], point[1]);
    //         // collision along inside part of the car
    //         if (inner_radius <= distance && distance < middle_radius) {
    //             float_t x = GetDistance(0, (0.5 * width), distance, turning_radius);
    //             // Return the arclength between the collision point on the car and the obstacle.
    //             float_t arc_length = 2 * distance * asin(
    //                 object_avoidance::GetDistance(x, width / 2, point[0], point[1]) / (2 * turning_radius));
    //             if (arc_length < shortest_distance) {
    //                 shortest_distance = arc_length;
    //             }

    //         // collision along front of the car
    //         } else if (middle_radius <= distance && distance < outter_radius) {
    //             float_t y = -(GetDistance(0, -(width - wheelbase) / 2, distance, wheelbase) - turning_radius);
    //             // Return the arclength between the collision point on the car and the obstacle.
    //             float_t arc_length = 2 * distance * asin(
    //                 object_avoidance::GetDistance(wheelbase + (width - wheelbase) / 2, y, point[0], point[1]) / (2 * turning_radius));
    //             if (arc_length < shortest_distance) {
    //                 shortest_distance = arc_length;
    //             }
    //         }
    //     }
    //     return shortest_distance;
    // };



    // float_t ObjectAvoidance::FindBestCurvaturePath(const std::vector<Eigen::Vector2f>& cloud, const float min_turn_radius, const int num_paths,
    //                             const float car_width_, const float car_length_, const float rear_axle_offset_,
    //                             const float car_safety_margin_front_, const float car_safety_margin_side_,
    //                             const float score_max_distance_weight, const float score_min_turn_weight){
    // float start_path_curvature = - 1 / min_turn_radius;
    // float end_path_curvature = 1 / min_turn_radius;
    // float curvature_inc = (end_path_curvature - start_path_curvature) / num_paths;
    
    // // first non-zero MinPathLength will initialize best_curvature
    // // if they're all zero, just plan to go straight, or J-turn ?
    // float best_curvature = 0.0; 
    // float best_score_distance = 0.0;
    // float best_score_turn_magnitude = 0.0;
    // float best_score = 0.0;
    // float distance = 0.0;
    // float turn_magnitude = 0.0;
    // float score = 0.0;
    
    // for (float c = start_path_curvature; c <= end_path_curvature; c+=curvature_inc){
    //     float curve = floor(c*10000 + 0.5) / 10000;
    //     // if (curve == 0) {
    //     //   ROS_INFO("curve = 0 ===================");
    //     //   continue;
    //     // }

    //     // visualization::DrawPathOption(curve, 10, 0, local_viz_msg_);
    //     // returns worse case path length 
    //     distance = FindMinPathLength(cloud, curve);
        
    //     turn_magnitude = (end_path_curvature - abs(curve)); // greater value means more straight
        
    //     // score comprised of weighted sum of path distance and curvature magnitude (favors going straight)
    //     // we should probably normalize "distance" (0 - 10) and "turn_magnitude" (0 - 1.02)
    //     // for right now, I just multipled that later by 10.0 to account for that
    //     // score = score_max_distance_weight * pow(distance, 3.0) + score_min_turn_weight * turn_magnitude;
    //     score = score_max_distance_weight * distance + score_min_turn_weight * 10.0 * turn_magnitude;
        
    //     if (score > best_score) {
    //     // if the best_score_distance is less than 3.0 meters? robot should probably turn around
    //     best_score_distance = distance; // max distance of the best score seen so far (debugging)
    //     best_score_turn_magnitude = turn_magnitude;
    //     best_score = score;
    //     // curvature corresponding to best score so far
    //     best_curvature = curve;
    //     // best_curvature = floor(c*10000 + 0.5) / 10000; //round to nearest ten-thousandth
    //     }
    // }
    // visualization::DrawPathOption(best_curvature, 10, 0, local_viz_msg_);
    // float_t turning_radius = 1 / best_curvature;
    // float_t inner_radius = abs(turning_radius) - (car_width_ / 2) - car_safety_margin_side_;
    // float_t middle_radius = sqrt(
    //     pow(abs(turning_radius) - (car_width_ / 2) - car_safety_margin_side_, 2.0) + 
    //     pow(rear_axle_offset_ + car_length_ + car_safety_margin_front_, 2.0));
    // float_t outter_radius = sqrt(
    //     pow(abs(turning_radius) + (car_width_ / 2.0) + car_safety_margin_side_, 2.0) +
    //     pow(rear_axle_offset_ + car_length_ + car_safety_margin_front_, 2.0));
    
    // visualization::DrawArc(
    //     Vector2f(0, 1 / best_curvature), inner_radius,
    //     -6, 1, 0x000000, local_viz_msg_);
    // visualization::DrawArc(
    //     Vector2f(0, 1 / best_curvature), outter_radius,
    //     -6, 1, 0x000000, local_viz_msg_);
    // visualization::DrawArc(
    //     Vector2f(0, 1 / best_curvature), middle_radius,
    //     -6, 1, 0x000000, local_viz_msg_);
    // visualization::DrawArc(
    //     Vector2f(0, 1 / best_curvature), 1 / best_curvature,
    //     -6, 1, 0x000000, local_viz_msg_);

    // ROS_INFO("best_score_distance = %f", best_score_distance);
    // ROS_INFO("best_score_turn_magnitude = %f", best_score_turn_magnitude);
    // ROS_INFO("best_score = %f", best_score);
    // ROS_INFO("best curvature = %f", best_curvature);
    // ROS_INFO("----------------------");
    // return best_curvature;
    // }


// // Given a single point, vehicle dimensions, and a curvature, return path length 
// float FindCurvePathLengthv2(const Vector2f& point, float curvature,
//                           const float car_width_, const float car_length_, const float rear_axle_offset_,
//                           const float car_safety_margin_front_, const float car_safety_margin_side_) {
//   float r = -1/curvature;
//   float r_min = r - car_width_/2.0 - car_safety_margin_side_;
//   float r_mid = sqrt(pow(r - car_width_/2.0 - car_safety_margin_side_, 2) + pow(-rear_axle_offset_ + car_length_/2.0 + car_safety_margin_front_, 2));
//   float r_max = sqrt(pow(r + car_width_/2.0 + car_safety_margin_side_, 2) + pow(-rear_axle_offset_ + car_length_/2.0 + car_safety_margin_front_, 2));
//   float r_obs = sqrt(pow(point.x(), 2) + pow(point.y() + r, 2)); // is r + or -

//   if ((r_min <= r_obs) && (r_obs < r_mid)){
    
//   }

//   return 10.0;
// }

}