// main.cpp
#include "relative_pose_finder/relative_pose_finder.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RelativePoseFinder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
