#include <iostream>

#include "gtest/gtest.h"

#include "object_avoidance.h"


TEST(FindCollisionPoints, FirstTest) {
    std::vector<Eigen::Vector2f> point_cloud;
    point_cloud.push_back(Eigen::Vector2f{10, 10});
    
    EXPECT_EQ(1, 2);
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