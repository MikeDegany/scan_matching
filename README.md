# Scan Matching Tutorial

A ROS2 package for 2D laser scan matching between two robots.

## Description

This package provides functionality to:
- Subscribe to laser scans from two different robots
- Visualize the scans in RViz
- (Future) Perform scan matching to find relative poses
- (Future) Publish transforms between robots

## Prerequisites

- ROS2 Humble
- Ubuntu 22.04
- C++17 or later

## Installation

```bash
# Create a workspace
mkdir -p ~/scan_matching_ws/src
cd ~/scan_matching_ws/src

# Clone the repository
git clone https://github.com/YOUR_USERNAME/scan_matching_tutorial.git

# Build the package
cd ..
colcon build
