// include/scan_matcher.hpp
#ifndef SCAN_MATCHER_HPP
#define SCAN_MATCHER_HPP

#include <vector>
#include <Eigen/Dense>
#include <sensor_msgs/msg/laser_scan.hpp>

struct Point2D {
    double x;
    double y;
    
    Point2D(double x_ = 0, double y_ = 0) : x(x_), y(y_) {}
};

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

private:
    std::vector<Point2D> convertScanToPoints(const sensor_msgs::msg::LaserScan::SharedPtr& scan);
    Point2D findNearestPoint(const Point2D& point, const std::vector<Point2D>& points);
    Eigen::Matrix3d computeTransform(const std::vector<Point2D>& points1,
                                   const std::vector<Point2D>& points2);
    double computeFitness(const std::vector<Point2D>& points1,
                         const std::vector<Point2D>& points2);
    void transformPoints(std::vector<Point2D>& points, const Eigen::Matrix3d& transform);
    
    int max_iterations_;
    double tolerance_;
};

#endif