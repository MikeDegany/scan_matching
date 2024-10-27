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
    
    // Initialize Karto objects
    laser_ = karto::LaserRangeFinder::CreateLaserRangeFinder(
        karto::LaserRangeFinder_Custom, "laser");
    laser_->SetMinimumRange(0.1);
    laser_->SetMaximumRange(30.0);
    laser_->SetMinimumAngle(-M_PI);
    laser_->SetMaximumAngle(M_PI);
    laser_->SetAngularResolution(0.25 * M_PI / 180.0);
    
    matcher_ = new karto::ScanMatcher();

    RCLCPP_INFO(this->get_logger(), "Scan matcher initialized");
    RCLCPP_INFO(this->get_logger(), "Listening to scan1: %s", scan1_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Listening to scan2: %s", scan2_topic_.c_str());
}

ScanMatcher::~ScanMatcher()
{
    delete matcher_;
    delete laser_;
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

    // Create Karto scans
    karto::Pose2 initial_pose(0.0, 0.0, 0.0);
    auto karto_scan1 = create_karto_scan(latest_scan1_, initial_pose);
    
    // Try different initial poses for scan2
    double best_score = -1.0;
    karto::Pose2 best_pose;
    karto::Matrix3 covariance;
    
    // Grid search for best match
    for (double x = -5.0; x <= 5.0; x += 0.5) {
        for (double y = -5.0; y <= 5.0; y += 0.5) {
            for (double theta = -M_PI; theta <= M_PI; theta += M_PI/8) {
                karto::Pose2 test_pose(x, y, theta);
                auto karto_scan2 = create_karto_scan(latest_scan2_, test_pose);
                
                karto::Pose2 mean = test_pose;
                
                double score = matcher_->MatchScan(
                    karto_scan2,
                    karto_scan1,
                    mean,
                    covariance);
                
                if (score > best_score) {
                    best_score = score;
                    best_pose = mean;
                }
                
                delete karto_scan2;
            }
        }
    }
    
    delete karto_scan1;
    
    if (best_score > 0.0) {
        RCLCPP_INFO(this->get_logger(), 
            "Found match with score: %f at pose: (%f, %f, %f)",
            best_score, best_pose.GetX(), best_pose.GetY(), best_pose.GetHeading());
        publish_transform(best_pose, latest_scan1_->header.frame_id, 
                        latest_scan2_->header.frame_id);
    }
    


    // Visulaize scans 
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


karto::LocalizedRangeScan* ScanMatcher::create_karto_scan(
    const sensor_msgs::msg::LaserScan::SharedPtr& scan,
    const karto::Pose2& pose)
{
    std::vector<kt_double> readings;
    
    for (const auto& range : scan->ranges) {
        if (std::isfinite(range)) {
            readings.push_back(static_cast<kt_double>(range));
        } else {
            readings.push_back(laser_->GetMaximumRange());
        }
    }
    
    auto karto_scan = new karto::LocalizedRangeScan(laser_->GetName(), readings);
    karto_scan->SetOdometricPose(pose);
    karto_scan->SetCorrectedPose(pose);
    
    return karto_scan;
}

void ScanMatcher::publish_transform(
    const karto::Pose2& pose,
    const std::string& parent_frame,
    const std::string& child_frame)
{
    geometry_msgs::msg::TransformStamped transform_msg;
    
    transform_msg.header.stamp = this->now();
    transform_msg.header.frame_id = parent_frame;
    transform_msg.child_frame_id = child_frame;
    
    transform_msg.transform.translation.x = pose.GetX();
    transform_msg.transform.translation.y = pose.GetY();
    transform_msg.transform.translation.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, pose.GetHeading());
    
    transform_msg.transform.rotation.x = q.x();
    transform_msg.transform.rotation.y = q.y();
    transform_msg.transform.rotation.z = q.z();
    transform_msg.transform.rotation.w = q.w();
    
    tf_broadcaster_->sendTransform(transform_msg);
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
