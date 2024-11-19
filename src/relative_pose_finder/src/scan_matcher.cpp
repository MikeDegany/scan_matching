// src/scan_matcher.cpp
#include "relative_pose_finder/scan_matcher.hpp"
#include <cmath>
#include <limits>
#include <iostream>

ScanMatcher::ScanMatcher(int max_iterations, double tolerance)
    : max_iterations_(max_iterations), tolerance_(tolerance)
{}

std::vector<Point2D> ScanMatcher::convertScanToPoints(
    const sensor_msgs::msg::LaserScan::SharedPtr& scan)
{
    std::vector<Point2D> points;
    double angle = scan->angle_min;
    
    for (const auto& range : scan->ranges) {
        if (std::isfinite(range) && range >= scan->range_min && range <= scan->range_max) {
            double x = range * std::cos(angle);
            double y = range * std::sin(angle);
            points.emplace_back(x, y);
        }
        angle += scan->angle_increment;
    }
    
    return points;
}

Point2D ScanMatcher::findNearestPoint(const Point2D& point,
                                    const std::vector<Point2D>& points)
{
    double min_dist = std::numeric_limits<double>::max();
    Point2D nearest;
    
    for (const auto& p : points) {
        double dist = std::hypot(point.x - p.x, point.y - p.y);
        if (dist < min_dist) {
            min_dist = dist;
            nearest = p;
        }
    }
    
    return nearest;
}

Eigen::Matrix3d ScanMatcher::computeTransform(const std::vector<Point2D>& points1,
                                            const std::vector<Point2D>& points2)
{
    // Compute centroids
    Point2D centroid1{0, 0}, centroid2{0, 0};
    for (size_t i = 0; i < points1.size(); ++i) {
        centroid1.x += points1[i].x;
        centroid1.y += points1[i].y;
        centroid2.x += points2[i].x;
        centroid2.y += points2[i].y;
    }
    centroid1.x /= points1.size();
    centroid1.y /= points1.size();
    centroid2.x /= points2.size();
    centroid2.y /= points2.size();
    
    // Compute covariance matrix
    double sxx = 0, sxy = 0, syx = 0, syy = 0;
    for (size_t i = 0; i < points1.size(); ++i) {
        double dx1 = points1[i].x - centroid1.x;
        double dy1 = points1[i].y - centroid1.y;
        double dx2 = points2[i].x - centroid2.x;
        double dy2 = points2[i].y - centroid2.y;
        
        sxx += dx1 * dx2;
        sxy += dx1 * dy2;
        syx += dy1 * dx2;
        syy += dy1 * dy2;
    }
    
    // Compute rotation
    double theta = std::atan2(sxy - syx, sxx + syy);
    
    // Create transformation matrix
    Eigen::Matrix3d transform = Eigen::Matrix3d::Identity();
    transform(0, 0) = std::cos(theta);
    transform(0, 1) = -std::sin(theta);
    transform(1, 0) = std::sin(theta);
    transform(1, 1) = std::cos(theta);
    transform(0, 2) = centroid2.x - (centroid1.x * std::cos(theta) - centroid1.y * std::sin(theta));
    transform(1, 2) = centroid2.y - (centroid1.x * std::sin(theta) + centroid1.y * std::cos(theta));
    
    return transform;
}

void ScanMatcher::transformPoints(std::vector<Point2D>& points,
                                const Eigen::Matrix3d& transform)
{
    for (auto& point : points) {
        double x = point.x;
        double y = point.y;
        point.x = transform(0, 0) * x + transform(0, 1) * y + transform(0, 2);
        point.y = transform(1, 0) * x + transform(1, 1) * y + transform(1, 2);
    }
}

double ScanMatcher::computeFitness(const std::vector<Point2D>& points1,
                                 const std::vector<Point2D>& points2)
{
    double total_error = 0;
    for (const auto& p1 : points1) {
        Point2D nearest = findNearestPoint(p1, points2);
        total_error += std::hypot(p1.x - nearest.x, p1.y - nearest.y);
    }
    return total_error / points1.size();
}

ScanMatcher::MatchResult ScanMatcher::match(
    const sensor_msgs::msg::LaserScan::SharedPtr& scan1,
    const sensor_msgs::msg::LaserScan::SharedPtr& scan2)
{
    // Convert scans to point clouds
    auto points1 = convertScanToPoints(scan1);
    auto points2 = convertScanToPoints(scan2);
    
    // Initial transform
    Eigen::Matrix3d current_transform = Eigen::Matrix3d::Identity();
    double prev_error = std::numeric_limits<double>::max();
    
    // ICP iterations
    for (int iter = 0; iter < max_iterations_; ++iter) {
        // Find corresponding points
        std::vector<Point2D> corresponding_points;
        for (const auto& p1 : points1) {
            corresponding_points.push_back(findNearestPoint(p1, points2));
        }
        
        // Compute transform
        Eigen::Matrix3d transform = computeTransform(points1, corresponding_points);
        current_transform = transform * current_transform;
        
        // Apply transform to points
        transformPoints(points1, transform);
        
        // Check convergence
        double error = computeFitness(points1, points2);
        if (std::abs(error - prev_error) < tolerance_) {
            break;
        }
        prev_error = error;
    }
    // Extract final transformation parameters
    MatchResult result;
    result.x = current_transform(0, 2);
    result.y = current_transform(1, 2);
    result.theta = std::atan2(current_transform(1, 0), current_transform(0, 0));
    result.fitness = 1.0 / (1.0 + prev_error);  // Convert error to fitness score
    
    return result;
}