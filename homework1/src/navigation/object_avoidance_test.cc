#include <iostream>

#include "gtest/gtest.h"

#include "object_avoidance.h"


TEST(FindCollisionPoints, FirstTest) {
    std::vector<Eigen::Vector2f> point_cloud;
    const auto p = Eigen::Vector2f{10, 10};
    point_cloud.push_back(p);

    const auto& points = object_avoidance::FindCollisionPoints(
        point_cloud,
        1.0,
        1.0,
        1.0,
        0.5);
    
    EXPECT_EQ(points.size(), 1);
}


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    std::cout << "RUNNING TESTS ..." << std::endl;
    int ret{RUN_ALL_TESTS()};
    if (!ret)
        std::cout << "<<<SUCCESS>>>" << std::endl;
    else
        std::cout << "FAILED" << std::endl;
    return 0;
}