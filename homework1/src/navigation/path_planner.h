#ifndef __SRC_NAVIGATION_PATH_PLANNER__
#define __SRC_NAVIGATION_PATH_PLANNER__

#include <vector>

#include "eigen3/Eigen/Dense"

#include "amrl_msgs/VisualizationMsg.h"


namespace path_planner {

    // Keep track of the characteristics of each potential path so they
    // can all be scored.
    struct candidate_path {
        float_t curvature;
        Eigen::Vector2f furthest_point;
        float_t free_path_length;
        float_t average_distance;
    };

    class PathPlanner {

        private:
            Eigen::Vector2f goal_pos_;
            float_t vehicle_width_;
            float_t vehicle_length_;
            float_t vehicle_wheelbase_;
            int32_t num_c_paths_;
            std::vector<float_t> curvatures_;
            std::vector<candidate_path> candidate_paths_;

            void EvaluteCurvatures(
                const std::vector<Eigen::Vector2f>& point_cloud,
                amrl_msgs::VisualizationMsg& msg);
            float_t ConsiderBestPath();

        public:
            explicit PathPlanner(Eigen::Vector2f goal_pos,
                                 float_t vehicle_width,
                                 float_t vehicle_length,
                                 float_t vehicle_wheelbase,
                                 int32_t num_c_paths);

            // Return the curvature of the best scoring path
            float_t FindBestPath(
                const std::vector<Eigen::Vector2f>& point_cloud,
                amrl_msgs::VisualizationMsg& msg);


    };
} // namespace path_planner

#endif