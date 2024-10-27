#include "scan_matcher/scan_matcher.hpp"

ScanMatcher::ScanMatcher()
: Node("scan_matcher")
{
    // Initialize parameters
    initialize_parameters();
    
    // Create subscribers
    scan1_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan1_topic_, 10,
        std::bind(&ScanMatcher::scan1_callback, this, std::placeholders::_1));
        
    scan2_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan2_topic_, 10,
        std::bind(&ScanMatcher::scan2_callback, this, std::placeholders::_1));
        
    // Create publisher
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "scan_visualization", 10);
        
    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    RCLCPP_INFO(this->get_logger(), "Scan matcher initialized");
    RCLCPP_INFO(this->get_logger(), "Listening to scan1: %s", scan1_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Listening to scan2: %s", scan2_topic_.c_str());
}

void ScanMatcher::initialize_parameters()
{
    // Declare parameters with default values
    this->declare_parameter("scan1_topic", "/JK3/sensors/lidar2d_0/scan");
    this->declare_parameter("scan2_topic", "/JK5/sensors/lidar2d_0/scan");
    this->declare_parameter("fixed_frame", "map");
    
    // Get parameters
    scan1_topic_ = this->get_parameter("scan1_topic").as_string();
    scan2_topic_ = this->get_parameter("scan2_topic").as_string();
    fixed_frame_ = this->get_parameter("fixed_frame").as_string();
}

void ScanMatcher::scan1_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    latest_scan1_ = msg;
    process_scans();
}

void ScanMatcher::scan2_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    latest_scan2_ = msg;
    process_scans();
}

void ScanMatcher::process_scans()
{
    if (!latest_scan1_ || !latest_scan2_) {
        return;
    }
    
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Create markers for first scan (red)
    auto markers1 = create_scan_markers(latest_scan1_, {1.0, 0.0, 0.0}, 0);
    
    // Create markers for second scan (blue)
    auto markers2 = create_scan_markers(latest_scan2_, {0.0, 0.0, 1.0}, 10000);
    
    // Combine markers
    marker_array.markers.insert(
        marker_array.markers.end(),
        markers1.markers.begin(),
        markers1.markers.end());
    marker_array.markers.insert(
        marker_array.markers.end(),
        markers2.markers.begin(),
        markers2.markers.end());
    
    // Publish markers
    marker_pub_->publish(marker_array);
}

visualization_msgs::msg::MarkerArray ScanMatcher::create_scan_markers(
    const sensor_msgs::msg::LaserScan::SharedPtr scan,
    const std::array<float, 3>& color,
    int id_offset)
{
    visualization_msgs::msg::MarkerArray marker_array;
    
    float angle = scan->angle_min;
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        // Skip invalid measurements
        if (!std::isfinite(scan->ranges[i])) {
            angle += scan->angle_increment;
            continue;
        }
        
        visualization_msgs::msg::Marker marker;
        marker.header = scan->header;
        marker.ns = "scan_points";
        marker.id = id_offset + i;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Calculate point position
        marker.pose.position.x = scan->ranges[i] * cos(angle);
        marker.pose.position.y = scan->ranges[i] * sin(angle);
        marker.pose.position.z = 0.0;
        
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        
        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        marker.color.a = 1.0;
        
        marker_array.markers.push_back(marker);
        
        angle += scan->angle_increment;
    }
    
    return marker_array;
}
