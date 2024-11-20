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
    
    transformed_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("transformed_scan", 10);



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

// sensor_msgs::msg::LaserScan publishTransformedScan(const sensor_msgs::msg::LaserScan::SharedPtr& scan, ScanMatcher::MatchResult result, rclcpp::Time& stamp){
//     ScanMatcher ScanMatcher_instance(50, 0.001);
//     sensor_msgs::msg::LaserScan transformed_scan = *scan;
//     transformed_scan.header.frame_id = "lidar2d_0_laser";
//     transformed_scan.header.stamp = stamp;

//     // Create transformation matrix
//     Eigen::Matrix3d transform = Eigen::Matrix3d::Identity();
//     transform(0, 0) = std::cos(result.theta);
//     transform(0, 1) = -std::sin(result.theta);
//     transform(1, 0) = std::sin(result.theta);
//     transform(1, 1) = std::cos(result.theta);
//     transform(0, 2) = result.x;
//     transform(1, 2) = result.y;
    
//     std::vector<Point2D> points = ScanMatcher_instance.convertScanToPoints(scan);

//     ScanMatcher_instance.transformPoints(points, transform);


//     for (size_t i = 0; i < transformed_scan.ranges.size(); ++i) {
//         transformed_scan.ranges[i] = std::hypot(points[i].x, points[i].y);
    
//     }
//     return transformed_scan;
// }


sensor_msgs::msg::LaserScan publishTransformedScan(const sensor_msgs::msg::LaserScan::SharedPtr& scan, ScanMatcher::MatchResult result, rclcpp::Time& stamp){
    ScanMatcher ScanMatcher_instance(50, 0.001);
    sensor_msgs::msg::LaserScan transformed_scan = *scan;
    transformed_scan.header.frame_id = "lidar2d_0_laser";
    transformed_scan.header.stamp = stamp;

    // Precompute the cosine and sine of the rotation angle
    double cos_theta = std::cos(result.theta);
    double sin_theta = std::sin(result.theta);

    // Loop through the ranges and apply the transformation
    for (size_t i = 0; i < transformed_scan.ranges.size(); ++i)
    {
        double range = transformed_scan.ranges[i];
        if (range < transformed_scan.range_min || range > transformed_scan.range_max)
        {
            // Skip invalid range readings
            transformed_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
            continue;
        }

        // Calculate the original Cartesian coordinates
        double angle = transformed_scan.angle_min + i * transformed_scan.angle_increment;
        double x = range * std::cos(angle);
        double y = range * std::sin(angle);

        // Apply the transformation
        double transformed_x = cos_theta * x - sin_theta * y + result.x;
        double transformed_y = sin_theta * x + cos_theta * y + result.y;

        // Convert back to polar coordinates
        double transformed_range = std::sqrt(transformed_x * transformed_x + transformed_y * transformed_y);
        double transformed_angle = std::atan2(transformed_y, transformed_x);

        // Check if the transformed angle is within the scan's angular limits
        int transformed_index = static_cast<int>((transformed_angle - transformed_scan.angle_min) / transformed_scan.angle_increment);
        if (transformed_index >= 0 && static_cast<size_t>(transformed_index) < transformed_scan.ranges.size())
        {
            transformed_scan.ranges[transformed_index] = static_cast<float>(transformed_range);
        }
        else
        {
            // If the transformed angle is out of range, ignore it
            transformed_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
        }
    }

    return transformed_scan;
}


// sensor_msgs::msg::LaserScan publishCorrespondedPoints(const sensor_msgs::msg::LaserScan::SharedPtr& scan1, const sensor_msgs::msg::LaserScan::SharedPtr& scan2, rclcpp::Time& stamp){
//     ScanMatcher ScanMatcher_instance(50, 0.001);
//     sensor_msgs::msg::LaserScan scan1_p = *scan1;
//     sensor_msgs::msg::LaserScan scan2_p = *scan2;

//     scan1_p.header.frame_id = "tf_laser";
//     scan1_p.header.stamp = stamp;
    
//     std::vector<Point2D> points1 = ScanMatcher_instance.convertScanToPoints(scan1);
//     std::vector<Point2D> points2 = ScanMatcher_instance.convertScanToPoints(scan2);

//     std::vector<Point2D> corresponding_points;

//     for (const auto& p1 : points1) {
//         corresponding_points.push_back(ScanMatcher_instance.findNearestPoint(p1, points2));
//     }

//     for (size_t i = 0; i < scan1_p.ranges.size(); ++i) {
//         scan1_p.ranges[i] = std::hypot(corresponding_points[i].x, corresponding_points[i].y);
    
//     }
//     return scan1_p;
// }


void RelativePoseFinder::match_scans()
{
    auto result = scan_matcher_.match(latest_scan1_, latest_scan2_);
    
    if (result.fitness) {  // Arbitrary threshold, adjust as needed
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "lidar2d_0_laser";
        transform.child_frame_id = "tf_laser";
        
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

        auto stamp = this->now();
        sensor_msgs::msg::LaserScan transformed_scan = publishTransformedScan(latest_scan2_, result, stamp);
        // sensor_msgs::msg::LaserScan transformed_scan = publishCorrespondedPoints(latest_scan1_, latest_scan2_, stamp);

        transformed_scan_pub_->publish(transformed_scan);
    } else {
        RCLCPP_WARN(this->get_logger(),
            "Low matching fitness: %.2f", result.fitness);
    }
}