/*
 * Basic code for object detection / avoidance.
 * NOTE: This code current only works for positive curvature.
 */

#include "object_avoidance.h"

#include <cmath>
#include <math.h>
#include "glog/logging.h"
#include "ros/ros.h"


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
            path.collision_point = FindMinPathLength(cloud, &path);
            path.free_path_length = std::min(path.collision_point.free_path_length, float_t(5.0));
            // Now for this path we have the point the robot will collide with before
            // moving r * pi/2 along the arc. We can now calculate the clearance for 
            // this path. Since we know the angle from the car to the collision point,
            // any other point which has a smaller angle from the car to the point
            // would be passed by the car before collision. 
            FindPathClearance(&path);
            //path.free_path_lengthv2 = FindMinPathLengthv2(cloud, path.curvature);            
            // Find the turn magnitude, greater value means more straight.
            path.turn_magnitude = abs((paths_->at(paths_->size() - 1).curvature - abs(path.curvature)));
            // Calculate a path's final score.
            path.score = (
                score_max_distance_weight_ * path.free_path_length
                + score_min_turn_weight * path.turn_magnitude
                + score_clearanse * path.clearance);
        }
    };

    /*
    * Find the clearance for each path. This is the average path length of its 2
    * neighbors
    */
    void ObjectAvoidance::FindPathClearance(PathOption* path) {
        // Intialize a vector to store all the clearances from the points which are ahead
        // of the robot
        std::vector<float_t> clearances;
        // For 0 curvature, the calculations are a bit different. Consider all the points
        // up until the collision point.
        if (path->curvature == 0.0) {
            for (auto point : path->points) {
                // Only consider the points which the robot will pass before colliding.
                if (point.point.x() < path->free_path_length) {
                    clearances.push_back(abs(point.point.y()) - car_specs_.total_side);
                }
            }
        }
        else {
            Eigen::Vector2f p1(0, 1 / path->curvature);
            ROS_INFO("c=%f. collision %f %f", path->curvature, path->collision_point.angle, path->collision_point.free_path_length);
            for (auto point : path->points) {
                // If the point along the baseline and on the circle drawn by p to point p is
                // has an angle larger than the angle to the collision point, the car will not
                // pass this point before collision.
                if (point.angle < path->collision_point.angle) {
                    auto point_radius = GetDistance(p1, point.point);

                    if (point_radius < abs(path->inner_radius)) {
                        clearances.push_back(path->inner_radius - point_radius);
                    }
                    else {
                        clearances.push_back(point_radius - path->outer_radius);
                    }
                }
            }
        }
        // Find the smallest clearance.
        if (clearances.size() != 0) {
            path->clearance = *std::min_element(clearances.begin(), clearances.end());
        } else {
            path->clearance = 1.0;
        }
        path->clearance = std::min(path->clearance, float_t(1.0));
    }

    /*
    * Function to find the longest traversable distance before collision along a given path
    * 
    * @param cloud: the point cloud reading
    * @param curvature: the path's curvature
    * 
    * @return the longest path along the given curve before any collision
    */
    CollisionPoint ObjectAvoidance::FindMinPathLength(
        const std::vector<Eigen::Vector2f>& cloud, PathOption* path) {

        // Since this is a new cloud, clear out all the previous points.
        path->points.clear();
        // maximum distance reading by laser
        float_t min_straight_path_length = 5.0;
        float_t min_path_length = abs(float_t(3.1415) / path->curvature);
        CollisionPoint collision_point;
        CollisionPoint closest_point;
        for (const auto& point: cloud) {
            // To avoid division by zero, send zero curvature path to special linear
            // calculator.
            if (path->curvature == 0) {
                collision_point = FindStraightPathLength(point, path); 
                // Update the longest path possible if a shorter one has been calculated.
                if (collision_point.free_path_length <= min_straight_path_length) {
                    closest_point = collision_point;
                    min_straight_path_length = closest_point.free_path_length;
                }
            } else {
                collision_point = FindCurvePathLength(point, path);
                // Update the longest path possible if a shorter one has been calculated.
                if (collision_point.free_path_length <= min_path_length) {
                    closest_point = collision_point;
                    min_path_length = collision_point.free_path_length;
                }
            }
        }
        
        return closest_point;
    }

    // // implements updated FindCurvePathLengthv2
    // float_t ObjectAvoidance::FindMinPathLengthv2(
    //     const std::vector<Eigen::Vector2f>& cloud, float_t curvature) {

    //     // maximum distance reading by laser
    //     float min_path_length = 10.0;
    //     float path_length;
    //     for (const auto& point: cloud) {
    //         // To avoid division by zero, send zero curvature path to special linear
    //         // calculator.
    //         if (curvature == 0) {
    //             path_length = FindStraightPathLength(point); 
    //         } else {
    //             path_length = FindCurvePathLengthv2(point, curvature);
    //         }
    //         // Update the longest path possible if a shorter one has been calculated.
    //         if (path_length < min_path_length) {
    //             min_path_length = path_length;
    //         }
    //     }
    //     return min_path_length;
    // }

    /*
    * Find the longest traversable distance before collision along a straight path
    * 
    * @param point: the point to check for collision with
    * 
    * @return the longest path until collision with the point. Returns 10.0 if no
    * collision with the point
    */ 
    CollisionPoint ObjectAvoidance::FindStraightPathLength(
        const Eigen::Vector2f& point, PathOption* path) {
        
        
        // Only consider points that are ahead of the robot's base link
        if (point.x() > 0) {
            // Add this point since it is ahead of the car. We don't really care about
            // the arclength/angle here because for the straight path, clearance is
            // determined by straight lines.
            path->points.push_back(
                CollisionPoint(point[0] - car_specs_.total_front, float_t(0), point));
            
            // Now check for collision. This will happen if the point is within the swept
            // volume of the car.
            if (abs(point[1]) <= car_specs_.total_side) {
                // The car will eventually collide, so find the distance from the front
                // of the car to the point.
                return CollisionPoint(
                    std::max(point[0] - car_specs_.total_front, (float_t)0.0), 0.0, point);
            }  else {
                // No collision ever.
                return CollisionPoint(float_t(5.0), float_t(0.0), point);
            }
        }
        // Don't worry about colliding with points which are behind the car.
        return CollisionPoint(float_t(5.0), float_t(0.0), point);
    }

    /*
    * Find the longest traversable distance before collision along a curved path
    * 
    * @param point: the point to check for collision with
    * @param curvature: the path struct
    * 
    * @return the longest path until collision with the point. Returns 10.0 if no
    * collision with the point
    */ 
    CollisionPoint ObjectAvoidance::FindCurvePathLength(
        const Eigen::Vector2f& point, PathOption* path) {
    
        // Set up some resued values.
        float_t pi = 3.1415;
        float_t max_arc_length = abs(pi / path->curvature);
    
        // If the point is behind the car then the smallest possible angle along a given
        // is pi/2. Note it could actually be larger, but this is sufficient for planning
        if (point.x() < 0) {
            return CollisionPoint(max_arc_length, pi, point);
        }
        
        float_t turning_radius = 1.0 / path->curvature;
        // Find the distance from the center of turning circle to point under consideration.
        float_t dist_to_point = GetDistance(0, turning_radius, point[0], point[1]);

        // Find the point which lies at x=0 along the arc drawn out by this point and
        // the center of turning. The origin in this coordinate system is the center of the
        // turning circle.
        float_t sign = (turning_radius < 0.0) ? float_t(1.0) : float_t(-1.0);
        auto point_along_arc = Eigen::Vector2f(0.0, sign * abs(dist_to_point));

        // NOTE: point is in the reference of the robot, we need to translate it to a
        // coordinate system centered at the turning center.
        auto point_translated = Eigen::Vector2f(point.x(), point.y() - turning_radius);

        // We have two points along the arc drawn by the collision point. Find the angle
        // between these points. The angle will determine whether the robot would actually
        // pass this point before hitting the collision point.
        float_t l2_distance = GetDistance(point_along_arc, point_translated);
        float_t angle = acos(
            1 - pow(l2_distance, 2.0) / (2 * (pow(dist_to_point, 2.0))));

        if (path->curvature < 0 && point.y() > car_specs_.total_side) {
            path->points.push_back(
                CollisionPoint(max_arc_length, pi, point));
        } else if (path->curvature > 0 && point.y() < -car_specs_.total_side) {
            path->points.push_back(
                    CollisionPoint(max_arc_length, pi, point));
        } else {
            path->points.push_back(
                CollisionPoint(dist_to_point * angle, angle, point));
        }

        // If we are turning left and the point under consideration is below the bottom
        // of the car, we won't hit the point until we circle all the way back around.
        if (path->curvature > 0 && point[1] < -car_specs_.total_side) {
            return CollisionPoint(max_arc_length, pi, point);
        }
        // If we are turning right and the point under consideration is above the top part
        // of the car, we won't hit the point until we circle all the way back around.
        else if (path->curvature < 0 && point[1] > car_specs_.total_side) {
            return CollisionPoint(max_arc_length, pi, point);
        }

        // Find the smallest radius of the car's swept volume. This point is along the inside
        // part of the car along the wheelbase.
        float_t inner_radius = abs(turning_radius) - car_specs_.total_side;
        path->inner_radius = inner_radius;
        // Find the radius drawn out by the inner most edge of the front part of the robot.
        // This determines if a point will collide with the front or the inner side of the
        // car.
        float_t middle_radius = sqrt(
            pow(abs(turning_radius) - car_specs_.total_side, 2.0) + 
            pow(car_specs_.total_front, 2.0));
        // Find the radius drawn out by the outter most edge of the front part of the robot.
        // This determines if a point will collide with the front of the car.
        float_t outter_radius = sqrt(
            pow(abs(turning_radius) + car_specs_.total_side, 2.0) +
            pow(car_specs_.total_front, 2.0));
        
        path->outer_radius = outter_radius;
    
        float_t shortest_distance = max_arc_length;
        float_t angle_with_collision_point = pi;
        Eigen::Vector2f furthest_point;

        float_t collision_to_point;
        // collision along inside part of the car
        if (inner_radius <= dist_to_point && dist_to_point < middle_radius) {
            // Find the x-coordinate of the point of collision along the car. We know the
            // y-position of this point. This works for positive and negative curvature.
            float_t x = sqrt(
                pow(dist_to_point, 2) - pow(abs(turning_radius) - car_specs_.total_side, 2));
        
            // Find the L2 distance from the point of collision to where the obstacle
            // point is right now. This will be used for arclength calculations.
            if (turning_radius < 0) {
                // If we are turning right, the collision along the inside of the car
                // will have a negative y component.
                collision_to_point = GetDistance(
                    x, -car_specs_.total_side, point[0], point[1]);
            }
            else {
                // If turning left, the collision along the inside wall will have a
                // positive y component.
                collision_to_point = GetDistance(
                    x, car_specs_.total_side, point[0], point[1]);
            }
            // Arch length is equal to r*theta where
            // theta := arccos(1 - (collision_to_point^2) / (2 * dist_to_point^2))
            float_t arc_length = dist_to_point * 
                acos(1 - (pow(collision_to_point, 2.0) / (2 * pow(dist_to_point, 2.0))));
            // Keep track of whether this point gives a smaller possible distance to travel.
            if (arc_length <= shortest_distance) {
                shortest_distance = arc_length;
                furthest_point = point;
            }
        // collision along front of the car
        } else if (middle_radius <= dist_to_point && dist_to_point < outter_radius) {
            float_t y;

            if (turning_radius < 0) {
                y = sqrt(
                    pow(dist_to_point, 2.0) - pow(car_specs_.total_front, 2.0)) - abs(turning_radius);
            } else {
                y = turning_radius - sqrt(pow(dist_to_point, 2.0) - pow(car_specs_.total_front, 2.0));
            }
            // Calculate the distance from the point of collision along the car to the
            // current position of the car.
            float_t dist_from_collision_to_point = GetDistance(
                car_specs_.total_front, y, point[0], point[1]);
            float_t angle = acos(
                1 - pow(dist_from_collision_to_point, 2.0) / (2 * (pow(dist_to_point, 2.0))));
            float_t arc_length = dist_to_point * angle;

            if (arc_length <= shortest_distance) {
                shortest_distance = arc_length;
                furthest_point = point;
                angle_with_collision_point = angle;
            }
        }

        return CollisionPoint(shortest_distance, angle_with_collision_point, point);
    }

    /*
    * Find the highest scoring path
    * 
    * @return curvature of the highest scoring path
    */
    PathOption ObjectAvoidance::GetHighestScorePath() {
        return *std::max_element(
            paths_->begin(),
            paths_->end(),
            [](const PathOption& lhs, const PathOption& rhs){
                return lhs.score < rhs.score;});
    }

    float_t ObjectAvoidance::FindCurvePathLengthv2(const Eigen::Vector2f& point, float_t curvature) {
        
        float r = 1 / abs(curvature);
        Eigen::Vector2f turn_point = Eigen::Vector2f(0.0, r * int(curvature / abs(curvature)));
        
        float r_min = r - (car_specs_.car_width / 2.0) - car_specs_.car_safety_margin_side;
        float r_mid = sqrt(pow(r_min, 2) + pow(-car_specs_.rear_axle_offset + (car_specs_.car_length / 2.0) + car_specs_.car_safety_margin_front, 2)); 
        float r_max = sqrt(pow(r + (car_specs_.car_width / 2.0) + car_specs_.car_safety_margin_side, 2) + pow(-car_specs_.rear_axle_offset + (car_specs_.car_length / 2.0) + car_specs_.car_safety_margin_front, 2)); 
        float r_obs = GetDistance(turn_point, point);
        
        float Beta;
        float alpha;
        if ((r_min <= r_obs) && (r_obs < r_mid)){ // point will hit the side of car
            Beta = acos(r - (car_specs_.car_width / 2.0) - car_specs_.car_safety_margin_side / r_obs);
        } else if ((r_mid <= r_obs) && (r_obs <= r_max)) {  // point will hit front of car
            Beta = asin((- car_specs_.rear_axle_offset + (car_specs_.car_length / 2.0) + car_specs_.car_safety_margin_front)/ r_obs);
        } else{ // else point doesn't hit car
            return std::min(float(10.0), float((r * M_PI/2)));
        }
        float dist = GetDistance(point, Eigen::Vector2f(0.0, 0.0));
        alpha =  acos((pow(r, 2) + pow(r_obs, 2) - pow(dist, 2))/(2 * r * r_obs)) - Beta;

        return abs(alpha * r); //std::min(float(abs(alpha * r)), float((r * M_PI/2))); // alpha in radians to path length
    }
};
