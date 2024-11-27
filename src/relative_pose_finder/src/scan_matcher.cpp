// src/scan_matcher.cpp
#include "relative_pose_finder/scan_matcher.hpp"
#include <cmath>
#include <limits>
#include <iostream>
#include <fstream>
#include <vector>
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

// Eigen::Matrix3d ScanMatcher::computeTransform(const std::vector<Point2D>& points1,
//                                             const std::vector<Point2D>& points2)
// {
//     // Compute centroids
//     Point2D centroid1{0, 0}, centroid2{0, 0};
//     for (size_t i = 0; i < points1.size(); ++i) {
//         centroid1.x += points1[i].x;
//         centroid1.y += points1[i].y;
//         centroid2.x += points2[i].x;
//         centroid2.y += points2[i].y;
//     }
//     centroid1.x /= points1.size();
//     centroid1.y /= points1.size();
//     centroid2.x /= points2.size();
//     centroid2.y /= points2.size();
    
//     // Compute covariance matrix
//     double sxx = 0, sxy = 0, syx = 0, syy = 0;
//     for (size_t i = 0; i < points1.size(); ++i) {
//         double dx1 = points1[i].x - centroid1.x;
//         double dy1 = points1[i].y - centroid1.y;
//         double dx2 = points2[i].x - centroid2.x;
//         double dy2 = points2[i].y - centroid2.y;
        
//         sxx += dx1 * dx2;
//         sxy += dx1 * dy2;
//         syx += dy1 * dx2;
//         syy += dy1 * dy2;
//     }
    
//     // Compute rotation
//     double theta = std::atan2(sxy - syx, sxx + syy);
    
//     // Create transformation matrix
//     Eigen::Matrix3d transform = Eigen::Matrix3d::Identity();
//     transform(0, 0) = std::cos(theta);
//     transform(0, 1) = -std::sin(theta);
//     transform(1, 0) = std::sin(theta);
//     transform(1, 1) = std::cos(theta);
//     transform(0, 2) = centroid2.x - (centroid1.x * std::cos(theta) - centroid1.y * std::sin(theta));
//     transform(1, 2) = centroid2.y - (centroid1.x * std::sin(theta) + centroid1.y * std::cos(theta));
    
//     return transform;
// }

// Eigen::Matrix3d computeTransformation(const std::vector<Eigen::Vector2d>& scan1,
//                                       const std::vector<Eigen::Vector2d>& scan2) {
//     // Initialization
//     const int max_iterations = 50;
//     const double convergence_threshold = 1e-3;

//     Eigen::Matrix3d T = Eigen::Matrix3d::Identity();  // Initial guess: Identity transformation

//     for (int iter = 0; iter < max_iterations; ++iter) {
//         // Step 1: Find correspondences (nearest points)
//         std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> correspondences;
//         for (const auto& p : scan2) {
//             Eigen::Vector2d transformed_p = (T.block<2, 2>(0, 0) * p) + T.block<2, 1>(0, 2);
//             // std::cout << "T: " << T << std::endl;
//             double min_distance = std::numeric_limits<double>::max();
//             Eigen::Vector2d closest_point;

//             for (const auto& q : scan1) {
//                 double distance = (q - transformed_p).squaredNorm();
//                 if (distance < min_distance) {
//                     min_distance = distance;
//                     closest_point = q;
//                 }
//             }
//             correspondences.emplace_back(transformed_p, closest_point);
//         }

//         // Step 2: Compute the point-to-line error for correspondences
//         Eigen::MatrixXd A(2 * correspondences.size(), 3);
//         Eigen::VectorXd b(2 * correspondences.size());
//         int row = 0;

//         for (const auto& [p2, p1] : correspondences) {
//             // Line representation: normal vector (nx, ny)
//             Eigen::Vector2d line_direction = (p1 - p2).normalized();
//             Eigen::Vector2d normal(-line_direction.y(), line_direction.x());

//             // Fill A and b for weighted least-squares
//             A.row(row) << normal.x(), normal.y(), 1.0;
//             b(row) = normal.dot(p1 - p2);// / normal.norm();
//             row++;
//         }

//         // Solve for delta transformation
//         Eigen::Vector3d delta = A.colPivHouseholderQr().solve(b);
//         std::cout << "delta: " << delta << std::endl;
//         // Update transformation matrix T
//         Eigen::Matrix3d delta_T = Eigen::Matrix3d::Identity();
//         delta_T(0, 0) = std::cos(delta(2));
//         delta_T(0, 1) = -std::sin(delta(2));
//         delta_T(1, 0) = std::sin(delta(2));
//         delta_T(1, 1) = std::cos(delta(2));
//         delta_T(0, 2) = delta(0);
//         delta_T(1, 2) = delta(1);

//         T = delta_T * T;
//         std::cout << "T: " << T << std::endl;

//         // Check for convergence
//         if (delta.norm() < convergence_threshold) {
//             break;
//         }
//     }

//     return T;
// }


Eigen::Matrix3d computeTransformation(const std::vector<Point2D>& scan1,
                                      const std::vector<Point2D>& scan2) {
    // Initialization
    const int max_iterations = 50;
    const double convergence_threshold = 1;

    Eigen::Matrix3d T = Eigen::Matrix3d::Identity();  // Initial guess: Identity transformation

    for (int iter = 0; iter < max_iterations; ++iter) {
        // Step 1: Find correspondences (nearest points)
        std::vector<std::pair<Point2D, Point2D>> correspondences;
        for (const auto& p : scan2) {
            // T.block<2, 2>(0, 0) : The top-left 2×2 submatrix of T,
            // T.block<2, 1>(0, 2) : The top-right 2×1 submatrix of T
            Eigen::Vector2d transformed_p = (T.block<2, 2>(0, 0) * Eigen::Vector2d(p.x, p.y)) + T.block<2, 1>(0, 2);
            // std::cout << "T: " << T << std::endl;
            double min_distance = std::numeric_limits<double>::max();
            Point2D closest_point;

            for (const auto& q : scan1) {
                Eigen::Vector2d q_vec(q.x, q.y);
                double distance = (q_vec - transformed_p).squaredNorm();
                if (distance < min_distance) {
                    min_distance = distance;
                    closest_point = q;
                }
            }
            correspondences.emplace_back(Point2D(transformed_p.x(), transformed_p.y()), closest_point);
        }

        // Step 2: Compute the point-to-line error for correspondences
        Eigen::MatrixXd A(2 * correspondences.size(), 3);
        Eigen::VectorXd b(2 * correspondences.size());
        int row = 0;

        for (const auto& [p2, p1] : correspondences) {

            Eigen::Vector2d p2_vec(p2.x, p2.y);
            Eigen::Vector2d p1_vec(p1.x, p1.y);
            // Line representation: normal vector (nx, ny)
            Eigen::Vector2d line_direction = (p1_vec - p2_vec).normalized();
            Eigen::Vector2d normal(-line_direction.y(), line_direction.x());

            // Fill A and b for weighted least-squares
            A.row(row) << normal.x(), normal.y(), 1.0;
            b(row) = normal.dot(p1_vec - p2_vec);// / normal.norm();
            row++;
        }

        // Solve for delta transformation
        Eigen::Vector3d delta = A.colPivHouseholderQr().solve(b);
        std::cout << "delta: " << delta << std::endl;
        // Update transformation matrix T
        Eigen::Matrix3d delta_T = Eigen::Matrix3d::Identity();
        delta_T(0, 0) = std::cos(delta(2));
        delta_T(0, 1) = -std::sin(delta(2));
        delta_T(1, 0) = std::sin(delta(2));
        delta_T(1, 1) = std::cos(delta(2));
        delta_T(0, 2) = delta(0);
        delta_T(1, 2) = delta(1);

        T = delta_T * T;
        std::cout << "T: " << T << std::endl;

        // Check for convergence
        if (delta.norm() < convergence_threshold) {
            break;
        }
    }

    return T;
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

void savePointsToFile(const std::vector<Point2D>& points, const std::string& filename) {
    std::ofstream file(filename);
    if (!file) {
        std::cerr << "Unable to open file: " << filename << std::endl;
        return;
    }

    for (const auto& point : points) {
        file << point.x << "," << point.y << "\n";
    }
    file.close();
}


ScanMatcher::MatchResult ScanMatcher::match(
    const sensor_msgs::msg::LaserScan::SharedPtr& scan1,
    const sensor_msgs::msg::LaserScan::SharedPtr& scan2)
{
    // Convert scans to point clouds
    auto points1 = convertScanToPoints(scan1);
    auto points2 = convertScanToPoints(scan2);
    
    savePointsToFile(points1, "points1.csv");
    savePointsToFile(points2, "points2.csv");

    // Initial transform
    Eigen::Matrix3d current_transform = Eigen::Matrix3d::Identity();
    double prev_error = std::numeric_limits<double>::max();
    
    // ICP iterations
    // for (int iter = 0; iter < max_iterations_; ++iter) {
    //     // Find corresponding points
    //     std::vector<Point2D> corresponding_points;
    //     for (const auto& p1 : points1) {
    //         corresponding_points.push_back(findNearestPoint(p1, points2));
    //     }
        
    //     // Compute transform
    //     Eigen::Matrix3d transform = computeTransform(points1, corresponding_points);
    //     current_transform = transform * current_transform;
        
    //     // Apply transform to points
    //     transformPoints(points1, transform);
        
    //     // Check convergence
    //     double error = computeFitness(points1, points2);
    //     if (std::abs(error - prev_error) < tolerance_) {
    //         break;
    //     }
    //     prev_error = error;
    // }
    // Extract final transformation parameters

        // Convert to std::vector<Eigen::Vector2d>
    std::vector<Eigen::Vector2d> eigen_points1;
    for (const auto& point1 : points1) {
        eigen_points1.emplace_back(point1.x, point1.y);
    }
        // Convert to std::vector<Eigen::Vector2d>
    std::vector<Eigen::Vector2d> eigen_points2;
    for (const auto& point2 : points2) {
        eigen_points2.emplace_back(point2.x, point2.y);
    }
    // current_transform = computeTransformation(eigen_points1, eigen_points2);
    current_transform = computeTransformation(points1, points2);
    MatchResult result;
    result.x = current_transform(0, 2);
    result.y = current_transform(1, 2);
    result.theta = std::atan2(current_transform(1, 0), current_transform(0, 0));
    result.fitness = 1.0 / (1.0 + prev_error);  // Convert error to fitness score
    // std::cout << "Fitness: " << result.fitness << std::endl;
    // std::cout << "x: " << result.x << " y: " << result.y << " theta: " << result.theta << std::endl;
    // std::cout << "current_transform: " << std::endl << current_transform << std::endl;
    // std::cout << "points1: " << points1.size() << " points2: " << points2.size() << std::endl;
    // std::cout << "eigen_points1: " << eigen_points1.size() << " eigen_points2: " << eigen_points2.size() << std::endl;
    return result;
}