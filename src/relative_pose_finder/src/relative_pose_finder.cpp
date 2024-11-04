// relative_pose_finder.cpp
#include "relative_pose_finder/relative_pose_finder.hpp"
#include <tf2/LinearMath/Quaternion.h>

RelativePoseFinder::RelativePoseFinder()
: Node("relative_pose_finder"), scan_matcher_(50, 0.001)
{
    robot1_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/JK3/sensors/lidar2d_0/scan", 10,
        std::bind(&RelativePoseFinder::robot1_scan_callback, this, std::placeholders::_1));

    robot2_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/JK5/sensors/lidar2d_0/scan", 10,
        std::bind(&RelativePoseFinder::robot2_scan_callback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
    scan1_received_ = false;
    scan2_received_ = false;
}

void RelativePoseFinder::robot1_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    latest_scan1_ = msg;
    scan1_received_ = true;
    if (scan2_received_) {
        match_scans();
    }
}

void RelativePoseFinder::robot2_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    latest_scan2_ = msg;
    scan2_received_ = true;
    if (scan1_received_) {
        match_scans();
    }
}

void RelativePoseFinder::match_scans()
{
    auto result = scan_matcher_.match(latest_scan1_, latest_scan2_);
    
    if (result.fitness > 0.7) {  // Arbitrary threshold, adjust as needed
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "JK3/base_link";
        transform.child_frame_id = "JK5/base_link";
        
        transform.transform.translation.x = result.x;
        transform.transform.translation.y = result.y;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, result.theta);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        
        tf_broadcaster_->sendTransform(transform);
        
        RCLCPP_INFO(this->get_logger(),
            "Found relative pose: x=%.2f, y=%.2f, theta=%.2f, fitness=%.2f",
            result.x, result.y, result.theta, result.fitness);
    } else {
        RCLCPP_WARN(this->get_logger(),
            "Low matching fitness: %.2f", result.fitness);
    }
}