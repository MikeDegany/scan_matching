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
    
    // transformed_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("transformed_scan", 10);
    transformed_scan_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_scan", 10);



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


sensor_msgs::msg::PointCloud2 publishTransformedScan(const sensor_msgs::msg::LaserScan::SharedPtr& scan, ScanMatcher::MatchResult result, rclcpp::Node* node){
    ScanMatcher ScanMatcher_instance(50, 0.001);
    sensor_msgs::msg::LaserScan transformed_scan = *scan;
    // transformed_scan.header.frame_id = "lidar2d_0_laser";
    // transformed_scan.header.stamp = stamp;

    // Create transformation matrix
    Eigen::Matrix3d transform = Eigen::Matrix3d::Identity();
    transform(0, 0) = std::cos(result.theta);
    transform(0, 1) = -std::sin(result.theta);
    transform(1, 0) = std::sin(result.theta);
    transform(1, 1) = std::cos(result.theta);
    transform(0, 2) = result.x;
    transform(1, 2) = result.y;
    
    std::vector<Point2D> points = ScanMatcher_instance.convertScanToPoints(scan);
    ScanMatcher_instance.transformPoints(points, transform);

    RCLCPP_INFO(node->get_logger(), "Publishing transformed PointCloud2 with %zu points", points.size());

    // Create PointCloud2 message
    sensor_msgs::msg::PointCloud2 pointcloud;
    pointcloud.header.frame_id = "lidar2d_0_laser";
    pointcloud.header.stamp = node->now();

    // Set metadata
    pointcloud.height = 1;  // Unordered point cloud
    pointcloud.width = points.size();
    pointcloud.is_dense = true;
    pointcloud.is_bigendian = false;

    // Define fields (x, y, z)
    sensor_msgs::msg::PointField field_x, field_y, field_z;
    field_x.name = "x";
    field_x.offset = 0;
    field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_x.count = 1;

    field_y.name = "y";
    field_y.offset = 4;
    field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_y.count = 1;

    field_z.name = "z";
    field_z.offset = 8;
    field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_z.count = 1;

    pointcloud.fields = {field_x, field_y, field_z};
    pointcloud.point_step = 12;             // Each point has 3 fields (x, y, z), each 4 bytes
    pointcloud.row_step = pointcloud.width * pointcloud.point_step;

    // Allocate space for point data
    pointcloud.data.resize(pointcloud.row_step * pointcloud.height);

    // Fill point data
    sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud, "z");

    for (const auto& point : points) {
        *iter_x = point.x;  // x-coordinate
        *iter_y = point.y;  // y-coordinate
        *iter_z = 0.0f;     // z-coordinate (LiDAR is 2D, so z=0)
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }


    // for (size_t i = 0; i < transformed_scan.ranges.size(); ++i) {
    //     transformed_scan.ranges[i] = std::hypot(points[i].x, points[i].y);
    
    // }
    return pointcloud;
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
    
    if (result.fitness > 0.7) {  // Arbitrary threshold, adjust as needed
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

        // sensor_msgs::msg::LaserScan transformed_scan = publishTransformedScan(latest_scan1_, result, stamp);
        // sensor_msgs::msg::LaserScan transformed_scan = publishCorrespondedPoints(latest_scan1_, latest_scan2_, stamp);
        sensor_msgs::msg::PointCloud2 transformed_scan = publishTransformedScan(latest_scan1_, result, this);

        transformed_scan_pub_->publish(transformed_scan);
    } else {
        RCLCPP_WARN(this->get_logger(),
            "Low matching fitness: %.2f", result.fitness);
    }
}