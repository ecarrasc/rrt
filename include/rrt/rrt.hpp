#pragma once

#include <Eigen/Core>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>


#include <rclcpp/rclcpp.hpp>

#include <rrt/distance_grid_map.hpp>

class RRT
{
public:
    struct Parameters
    {
        int iteration_count = 10000;
        double expansion_distance = 0.5;
        double step_size = 0.01;
        double goal_bias = 0.15;
    };

public:
    explicit RRT(rclcpp::Node::SharedPtr node);

private:
    void compute_path();

    Eigen::Vector2d sample_configuration() const;

    Eigen::Vector2d extend_towards(
        Eigen::Vector2d const& from,
        Eigen::Vector2d const& to
    ) const;

    void map_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    void start_pose_cb(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg
    );

    void goal_pose_cb(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg
    );

private:
    rclcpp::Node::SharedPtr node_;

    DistanceGridMap m_grid_map;
    Eigen::Vector3f m_start_pose;
    Eigen::Vector3f m_goal_pose;
    Parameters m_params;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_map_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_start_pose_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_goal_pose_sub;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_path_pub;
    //rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;
};