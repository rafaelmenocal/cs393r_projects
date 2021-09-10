#include <iostream>

#include "amrl_msgs/VisualizationMsg.h"

#include "path_planner.h"
#include "visualization/visualization.h"

#include "obstacle_avoidance.h"


namespace path_planner {

    PathPlanner::PathPlanner(Eigen::Vector2f goal_pos,
                                 float_t vehicle_width,
                                 float_t vehicle_length,
                                 float_t vehicle_wheelbase,
                                 int32_t num_c_paths) :
            goal_pos_(goal_pos),
            vehicle_width_(vehicle_width),
            vehicle_length_(vehicle_length),
            vehicle_wheelbase_(vehicle_wheelbase),
            num_c_paths_(num_c_paths)
    {
        for (int i = 0; i < num_c_paths_; i++) {
            curvatures_.push_back(-1 + ( 2 *float(i) / num_c_paths));
        }
    };

    float_t PathPlanner::FindBestPath(
        const std::vector<Eigen::Vector2f>& point_cloud,
        amrl_msgs::VisualizationMsg& msg) {

        EvaluteCurvatures(point_cloud, msg);
        return ConsiderBestPath();
    }

    void PathPlanner::EvaluteCurvatures(
        const std::vector<Eigen::Vector2f>& point_cloud,
        amrl_msgs::VisualizationMsg& msg) {
        // Clear out the old candidate paths
        candidate_paths_.clear();
        // Loop over each potential path/curvature
        for (const auto& curvature : curvatures_) {
            visualization::DrawPathOption(curvature, 1.0, 0.0, msg);
            // Find all the point in the cloud which would collide with this path which
            // 10m.
            const auto& points = obstacle_avoidance::FindCollisionPoints(
                point_cloud, 1 / curvature, vehicle_width_,
                vehicle_length_, 2 * vehicle_length_ / 3);

            // Get the furthest distance the car could travel along this path without hitting
            // an obstacle.
            const auto furthest_point = obstacle_avoidance::FindShortestDistance(
                points, 1 / curvature, vehicle_width_,
                vehicle_length_, 2 * vehicle_length_ / 3);

            candidate_paths_.push_back(candidate_path{curvature, furthest_point});
        }
    }

    float_t PathPlanner::ConsiderBestPath() {
        float_t best_score = 0.0;
        float_t best_curvature = 0.0;

        for (const auto& path : candidate_paths_) {
            float_t score = sqrt(pow(10 - path.furthest_point[0], 2) + pow(path.furthest_point[1], 2));
            if (score > best_score) {
                best_score = score;
                best_curvature = path.curvature;
            }
        }
        return best_curvature;
    }

}// namespace path_planner
