#include "scan_matcher/scan_matcher.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ScanMatcher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
