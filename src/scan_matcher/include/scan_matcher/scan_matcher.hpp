#ifndef SCAN_MATCHER_HPP
#define SCAN_MATCHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class ScanMatcher : public rclcpp::Node
{
public:
    ScanMatcher();

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
    
    // Callbacks
    void scan1_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void scan2_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    // Processing methods
    void process_scans();
    visualization_msgs::msg::MarkerArray create_scan_markers(
        const sensor_msgs::msg::LaserScan::SharedPtr scan,
        const std::array<float, 3>& color,
        int id_offset);

    // Parameter initialization
    void initialize_parameters();
};

#endif
