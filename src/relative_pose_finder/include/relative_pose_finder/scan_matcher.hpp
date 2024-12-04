// include/scan_matcher.hpp
#ifndef SCAN_MATCHER_HPP
#define SCAN_MATCHER_HPP

#include <vector>
#include <Eigen/Dense>
#include <sensor_msgs/msg/laser_scan.hpp>


class ScanMatcher {
public:
    struct MatchResult {
        double x;        // x translation
        double y;        // y translation
        double theta;    // rotation
        double fitness;  // matching fitness score
    };

    ScanMatcher(int max_iterations = 50, double tolerance = 0.001);
    
    MatchResult match(const sensor_msgs::msg::LaserScan::SharedPtr& scan1,
                     const sensor_msgs::msg::LaserScan::SharedPtr& scan2);

// private:
    std::vector<Eigen::Vector2d> convertScanToPoints(const sensor_msgs::msg::LaserScan::SharedPtr& scan);
    Eigen::Vector2d findNearestPoint(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& points);
    Eigen::Matrix3d computeTransform(const std::vector<Eigen::Vector2d>& points1,
                                   const std::vector<Eigen::Vector2d>& points2);
    double computeFitness(const std::vector<Eigen::Vector2d>& points1,
                         const std::vector<Eigen::Vector2d>& points2);
    void transformPoints(std::vector<Eigen::Vector2d>& points, const Eigen::Matrix3d& transform);
    
    int max_iterations_;
    double tolerance_;
};

#endif