#pragma once

#include <Eigen/Core>
#include <string>
#include <random>
#include <cmath>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

Eigen::Vector3f compute_delta(
    const nav_msgs::msg::Odometry& previous,
    const nav_msgs::msg::Odometry& current
);

double extract_yaw(const geometry_msgs::msg::Quaternion& orientation);

double uniform_zero_one();

double normalize_angle(double radians);