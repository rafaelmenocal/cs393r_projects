#include <cmath>

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

        for (const auto& point : point_cloud) {
            
        }
        
    };


    float_t FindShortestDistance(
        std::vector<Eigen::Vector2f> point_cloud, float_t turning_radius)
    {
        
    };
    
}