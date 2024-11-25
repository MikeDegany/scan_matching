// relative_pose_finder.hpp
#ifndef RELATIVE_POSE_FINDER_HPP
#define RELATIVE_POSE_FINDER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "relative_pose_finder/scan_matcher.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

class RelativePoseFinder : public rclcpp::Node
{
public:
    RelativePoseFinder();

private:
    void robot1_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void robot2_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void match_scans();

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr robot1_scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr robot2_scan_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr transformed_scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_scan_pub_;

    
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan1_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan2_;
    
    ScanMatcher scan_matcher_;
    bool scan1_received_;
    bool scan2_received_;
};

#endif