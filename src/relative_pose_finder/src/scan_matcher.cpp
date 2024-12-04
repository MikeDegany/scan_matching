    // src/scan_matcher.cpp
#include "relative_pose_finder/scan_matcher.hpp"
#include <cmath>
#include <limits>
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include "relative_pose_finder/matplotlibcpp.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include <chrono>

ScanMatcher::ScanMatcher(int max_iterations, double tolerance)
    : max_iterations_(max_iterations), tolerance_(tolerance)
{}
namespace plt = matplotlibcpp;


void animateResults(
    const std::vector<std::vector<Eigen::Vector2d>>& P_values,
    const std::vector<Eigen::Vector2d>& Q,
    const std::vector<std::vector<std::pair<int, int>>>& corresp_values,
    const cv::Point2d& xlim, const cv::Point2d& ylim) {
    
    // Create a blank image for plotting
    int width = 1000, height = 1000;
    cv::Mat canvas(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    
    // Coordinate transformation to fit points into the canvas
    auto transformToCanvas = [&](const Eigen::Vector2d& point) -> cv::Point {
        double x = (point.x() - xlim.x) / (xlim.y - xlim.x) * width;
        double y = height - (point.y() - ylim.x) / (ylim.y - ylim.x) * height; // Flip Y-axis
        return cv::Point(static_cast<int>(x), static_cast<int>(y));
    };
    
    // Draw Q points once
    for (const auto& point : Q) {
        cv::circle(canvas, transformToCanvas(point), 3, cv::Scalar(0, 0, 255), -1);
    }

    // Animation loop
    for (size_t frame = 0; frame < P_values.size(); ++frame) {
        // Copy the blank canvas
        cv::Mat frameCanvas = canvas.clone();

        // Draw P points for the current frame
        for (const auto& point : P_values[frame]) {
            cv::circle(frameCanvas, transformToCanvas(point), 3, cv::Scalar(51, 102, 153), -1);
        }

        // Draw correspondences for the current frame
        for (const auto& [i, j] : corresp_values[frame]) {
            cv::line(frameCanvas, transformToCanvas(P_values[frame][i]), transformToCanvas(Q[j]),
                     cv::Scalar(128, 128, 128), 1);
        }

        // Show the frame
        cv::imshow("Animation", frameCanvas);

        // Delay to control animation speed
        int delay_ms = 500; // Adjust frame interval here
        if (cv::waitKey(delay_ms) == 27) { // Press 'ESC' to exit
            break;
        }
    }

    cv::destroyAllWindows();
}



void plotPcl(const std::vector<Eigen::Vector2d>& Points,
              const std::string& label, std::string color = "#336699",
              double markersize = 2.0) {
    // Clear the previous plot
    plt::clf();
    // Separate x and y coordinates of Points
    std::vector<double> x, y;
    for (const auto& point : Points) {
        x.push_back(point.x());
        y.push_back(point.y());
    }


    // Plot the first dataset
    if (!x.empty() && !y.empty()) {
        plt::scatter(x, y, markersize, {{"color", color}, {"label", label}});
    }

    // Configure the plot
    plt::legend();
    plt::axis("equal");
    // Update the plot without blocking
    plt::show();
    plt::pause(0.001);
}

std::vector<std::pair<int, int>> get_correspondence_indices(const std::vector<Eigen::Vector2d>& P, const std::vector<Eigen::Vector2d>& Q) {
    int p_size = P.size();
    int q_size = Q.size();
    std::vector<std::pair<int, int>> correspondences;

    for (int i = 0; i < p_size; ++i) {
        Eigen::Vector2d p_point = P[i];  // Get the i-th point in P
        double min_dist = std::numeric_limits<double>::max();
        int chosen_idx = -1;

        for (int j = 0; j < q_size; ++j) {
            Eigen::Vector2d q_point = Q[j];  // Get the j-th point in Q
            double dist = (q_point - p_point).norm();  // Euclidean distance

            if (dist < min_dist) {
                min_dist = dist;
                chosen_idx = j;
            }
        }
        correspondences.push_back({i, chosen_idx});
    }

    return correspondences;
}

//Derivative of Rotation matrix 
Eigen::Matrix2d dR(double theta) {
    Eigen::Matrix2d result;
    result << -std::sin(theta), -std::cos(theta),
               std::cos(theta), -std::sin(theta);
    return result;
}

Eigen::Matrix2d R(double theta) {
    Eigen::Matrix2d result;
    result << std::cos(theta), -std::sin(theta),
              std::sin(theta),  std::cos(theta);
    return result;
}

// Compute the Jacobian matrix
Eigen::MatrixXd jacobian(const Eigen::Vector3d& x, const Eigen::Vector2d& p_point) {
    double theta = x[2];
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(2, 3);

    // Set the top-left 2x2 block to identity
    J.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity();

    // Compute dR(theta) * p_point and assign it to the last column of J
    J.block<2, 1>(0, 2) = dR(theta) * p_point;

    return J;
}

// Compute the error vector
Eigen::Vector2d error(const Eigen::Vector3d& x, const Eigen::Vector2d& p_point, const Eigen::Vector2d& q_point) {
    Eigen::Matrix2d rotation = R(x[2]); // Compute rotation matrix
    Eigen::Vector2d translation = x.head<2>(); // Extract translation (x[0], x[1])

    // Compute the predicted point
    Eigen::Vector2d prediction = rotation * p_point + translation;

    // Return the error (difference between prediction and q_point)
    return prediction - q_point;
}

// Compute the normals for a given point cloud
void compute_normals(
    const std::vector<Eigen::Vector2d>& points, 
    std::vector<Eigen::Vector2d>& normals, 
    std::vector<Eigen::Vector2d>& normals_at_points, 
    int step = 1) 
{
    // Initialize normals with a default value
    normals.push_back(Eigen::Vector2d(0, 0));  // Placeholder for the first point

    // Loop over points and compute normals
    for (size_t i = step; i < points.size() - step; ++i) {
        const Eigen::Vector2d& prev_point = points[i - step];  // Previous point
        const Eigen::Vector2d& next_point = points[i + step];  // Next point
        const Eigen::Vector2d& curr_point = points[i];  // Current point

        // Compute the normal direction (perpendicular to the direction between previous and next point)
        Eigen::Vector2d delta = next_point - prev_point;
        Eigen::Vector2d normal(-delta[1], delta[0]);  // Perpendicular direction

        // Normalize the normal vector
        normal.normalize();

        // Store the normal and the position of the normal
        normals.push_back(normal);
        normals_at_points.push_back(normal + curr_point);
    }

    // Add a placeholder for the last point
    normals.push_back(Eigen::Vector2d(0, 0));  // Placeholder for the last point
}



void prepare_system_normals(
    const Eigen::Vector3d& x,  // Transformation vector (translation + rotation)
    const std::vector<Eigen::Vector2d>& P,  // Source point cloud P
    const std::vector<Eigen::Vector2d>& Q,  // Target point cloud Q
    const std::vector<std::pair<int, int>>& correspondences,  // Correspondences between P and Q
    const std::vector<Eigen::Vector2d>& Q_normals,  // Normals at points in Q
    Eigen::Matrix3d& H,  // The information matrix
    Eigen::Vector3d& g,  // The gradient vector
    double& chi  // The chi-squared error
) {
    // Initialize H, g, and chi
    H.setZero();
    g.setZero();
    chi = 0;

    // Loop through the correspondences and compute Jacobian and error
    for (const auto& correspondence : correspondences) {
        int i = correspondence.first;  // Index of point in P
        int j = correspondence.second;  // Index of point in Q

        const Eigen::Vector2d& p_point = P[i];  // Get the i-th point in P
        const Eigen::Vector2d& q_point = Q[j];  // Get the j-th point in Q
        const Eigen::Vector2d& normal = Q_normals[j];  // Normal at the j-th point in Q

        // Compute the error between predicted and actual points
        // double e =0; 
        double e = normal.dot(error(x, p_point, q_point));  // Error vector
        // Compute the Jacobian for the transformation (rotation + translation)
        // Eigen::Matrix<double, 1, 3> J = normal.dot(jacobian(x, p_point));  // Jacobian
        Eigen::Matrix<double, 1, 3> J = normal.transpose() * jacobian(x, p_point);  // Jacobian
        // Eigen::MatrixXd J = normal.transpose() * jacobian(x, p_point);  // Jacobian
        // Eigen::MatrixXd J;
        // J.setZero(1, 3);

        // Update H, g, and chi
        H += J.transpose() * J;
        g += J.transpose() * e;
        chi += e * e;
    }
}

Eigen::Vector3d icp_normal(
    const std::vector<Eigen::Vector2d>& P,  // Source point cloud
    const std::vector<Eigen::Vector2d>& Q,  // Target point cloud
    const std::vector<Eigen::Vector2d>& Q_normals,  // Normals for points in Q
    int iterations,  // Number of ICP iterations
    std::vector<std::vector<Eigen::Vector2d>>& P_values,  // Updated point clouds over iterations
    std::vector<double>& chi_values,  // List of chi-squared error values
    std::vector<std::vector<std::pair<int, int>>>& corresp_values  // List of correspondences
) {
    // Initial guess for the transformation (translation and rotation)
    Eigen::Vector3d x = Eigen::Vector3d::Zero();  // Initialize to zero
    Eigen::Matrix3d H;
    Eigen::Vector3d g;
    double chi;

    // P-values initialization
    P_values.push_back(P);

    for (int i = 0; i < iterations; ++i) {
        // Step 1: Get correspondences between point clouds P and Q
        std::vector<std::pair<int, int>> correspondences = get_correspondence_indices(P, Q);
        corresp_values.push_back(correspondences);

        // Step 2: Prepare system of equations based on normals
        prepare_system_normals(x, P, Q, correspondences, Q_normals, H, g, chi);

        // Step 3: Solve the system using least squares to find the update
        Eigen::Vector3d dx = H.ldlt().solve(-g);
        x += dx;

        // Normalize angle
        x[2] = std::atan2(std::sin(x[2]), std::cos(x[2]));
        //print x
        // std::cout << "x: " << x.transpose() << std::endl;
        // Step 4: Update the point cloud P based on the current transformation
        // Eigen::Matrix2Xd P_transformed = Eigen::Matrix2Xd(P.size(), P[0].size());
        Eigen::Matrix2Xd P_transformed = Eigen::Matrix2Xd(P[0].size(), P.size());
        for (size_t j = 0; j < P.size(); ++j) {
            Eigen::Vector2d p_point = P[j];
            Eigen::Matrix2d rotation_matrix;
            rotation_matrix << std::cos(x[2]), -std::sin(x[2]),
                               std::sin(x[2]), std::cos(x[2]);

            P_transformed.col(j) = rotation_matrix * p_point + x.head(2);  // Apply transformation
        }

        // Save the transformed P
        std::vector<Eigen::Vector2d> P_transformed_vec;
        for (int k = 0; k < P_transformed.cols(); ++k) {
            P_transformed_vec.push_back(P_transformed.col(k));
        }
        P_values.push_back(P_transformed_vec);

        // Save chi-squared error
        chi_values.push_back(chi);
    }
    return x;
}

void ScanMatcher::transformPoints(std::vector<Eigen::Vector2d>& points,
                                const Eigen::Matrix3d& transform)
{
    for (auto& point : points) {
        double x = point[0];
        double y = point[1];
        point[0] = transform(0, 0) * x + transform(0, 1) * y + transform(0, 2);
        point[1] = transform(1, 0) * x + transform(1, 1) * y + transform(1, 2);
    }
}

Eigen::Vector2d ScanMatcher::findNearestPoint(const Eigen::Vector2d& point,
                                    const std::vector<Eigen::Vector2d>& points)
{
    double min_dist = std::numeric_limits<double>::max();
    Eigen::Vector2d nearest;
    
    for (const auto& p : points) {
        double dist = std::hypot(point[0] - p[0], point[1] - p[1]);
        if (dist < min_dist) {
            min_dist = dist;
            nearest = p;
        }
    }
    return nearest;
}
double ScanMatcher::computeFitness(const std::vector<Eigen::Vector2d>& points1,
                                 const std::vector<Eigen::Vector2d>& points2)
{
    double total_error = 0;
    for (const auto& p1 : points1) {
        Eigen::Vector2d nearest = findNearestPoint(p1, points2);
        total_error += std::hypot(p1[0] - nearest[0], p1[1] - nearest[1]);
    }
    return total_error / points1.size();
}

std::vector<Eigen::Vector2d> ScanMatcher::convertScanToPoints(
    const sensor_msgs::msg::LaserScan::SharedPtr& scan)
{
    std::vector<Eigen::Vector2d> points;
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


ScanMatcher::MatchResult ScanMatcher::match(
    const sensor_msgs::msg::LaserScan::SharedPtr& scan1,
    const sensor_msgs::msg::LaserScan::SharedPtr& scan2)
{

    // // savePointsToFile(points1, "points1_transformed.csv");
    // // savePointsToFile(points2, "points2_transformed.csv");
    
    // // Initial transform
    // Eigen::Matrix3d current_transform = Eigen::Matrix3d::Identity();
    // double prev_error = std::numeric_limits<double>::max();
    
    // // ICP iterations
    // for (int iter = 0; iter < max_iterations_; ++iter) {
    //     // Find corresponding points
    //     std::vector<Eigen::Vector2d> corresponding_points;
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
    // // Extract final transformation parameters
    // MatchResult result;
    // result.x = current_transform(0, 2);
    // result.y = current_transform(1, 2);
    // result.theta = std::atan2(current_transform(1, 0), current_transform(0, 0));
    // result.fitness = 1.0 / (1.0 + prev_error);  // Convert error to fitness score
    
    // return result;


        // Convert scans to point clouds
    auto points1 = convertScanToPoints(scan1);
    auto points2 = convertScanToPoints(scan2);

    plotPcl(points1, "Cloud 1", "blue");
    plotPcl(points2, "Cloud 2", "red");
    std::vector<std::pair<int, int>> correspondences;

    correspondences = get_correspondence_indices(points1, points2);
    // Print correspondences
    // for (const auto& correspondence : correspondences) {
    //     std::cout << "P[" << correspondence.first << "] -> Q[" << correspondence.second << "]\n";
    // }


    //Compute normals for points1
    std::vector<Eigen::Vector2d> normals2;
    std::vector<Eigen::Vector2d> normals_at_points2;
    compute_normals(points2, normals2, normals_at_points2, 1);
    // //print normals1 and normals_at_points1
    // for (size_t i = 0; i < normals1.size(); ++i) {
    //     std::cout << "Normal " << i << ": " << normals1[i].transpose() << std::endl;
    //     std::cout << "Normal at point " << i << ": " << normals_at_points1[i].transpose() << std::endl;
    // }
    //compute icp_normal
    std::vector<std::vector<Eigen::Vector2d>> P_values;
    std::vector<double> chi_values;
    std::vector<std::vector<std::pair<int, int>>> corresp_values;
    // icp_normal(points1, points2, normals2, max_iterations_, P_values, chi_values, corresp_values);
    Eigen::Vector3d x = icp_normal(points1, points2, normals2, 20, P_values, chi_values, corresp_values);


    // Get the last point cloud from P_values
    const std::vector<Eigen::Vector2d>& last_point_cloud = P_values.back();

    // Print the points in the last point cloud
    // std::cout << "Last Point Cloud:" << std::endl;
    // for (const auto& point : last_point_cloud) {
    //     std::cout << point.transpose() << std::endl;  // Printing each point (as a row vector)
    // }
    double error = computeFitness(last_point_cloud, points2);
    std::cout << "Error: " << error << std::endl;
    // Extract final transformation parameters
    MatchResult result;
    result.x = x[0]; //current_transform(0, 2);
    result.y = x[1];//current_transform(1, 2);
    result.theta = x[2]; //std::atan2(current_transform(1, 0), current_transform(0, 0));
    result.fitness = 1.0 / (1.0 + error);  // Convert error to fitness score

    // X and Y limits
    cv::Point2d xlim(0, 5);
    cv::Point2d ylim(0, 5);

    // animateResults(P_values, points2, corresp_values, xlim, ylim);
    return result;
}