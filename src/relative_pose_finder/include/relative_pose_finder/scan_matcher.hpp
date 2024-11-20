// include/scan_matcher.hpp
#ifndef SCAN_MATCHER_HPP
#define SCAN_MATCHER_HPP

#include <vector>
#include <Eigen/Dense>
#include <sensor_msgs/msg/laser_scan.hpp>

struct PolarPoint {
    double range;
    double angle;
    PolarPoint(double r = 0, double a = 0) : range(r), angle(a) {}
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
    std::vector<PolarPoint> extractPolarPoints(const sensor_msgs::msg::LaserScan::SharedPtr& scan);
    Eigen::Matrix3d computeTransform(const std::vector<PolarPoint>& scan1,
                                     const std::vector<PolarPoint>& scan2);
    double computeFitness(const std::vector<PolarPoint>& scan1,
                          const std::vector<PolarPoint>& scan2);

    int max_iterations_;
    double tolerance_;
};

#endif
