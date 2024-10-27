// include/scan_matcher/scan_matcher.hpp
#ifndef SCAN_MATCHER_HPP
#define SCAN_MATCHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <karto_sdk/Mapper.h>

class ScanMatcher : public rclcpp::Node
{
public:
    ScanMatcher();
    ~ScanMatcher();

private:
    // Parameters
    std::string scan1_topic_;
    std::string scan2_topic_;
    std::string fixed_frame_;
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan1_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan2_sub_;
    
    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    
    // TF broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Latest scans
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan1_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan2_;
    
    // Karto objects
    karto::LaserRangeFinder* laser_;
    karto::ScanMatcher* matcher_;
    
    // Callbacks
    void scan1_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void scan2_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    // Processing methods
    void process_scans();
    visualization_msgs::msg::MarkerArray create_scan_markers(
        const sensor_msgs::msg::LaserScan::SharedPtr scan,
        const std::array<float, 3>& color,
        int id_offset);
        
    // Karto helper methods
    karto::LocalizedRangeScan* create_karto_scan(
        const sensor_msgs::msg::LaserScan::SharedPtr& scan,
        const karto::Pose2& pose);
    void publish_transform(
        const karto::Pose2& pose,
        const std::string& parent_frame,
        const std::string& child_frame);
};

#endif
