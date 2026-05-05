#include <rrt/utils.hpp>

Eigen::Vector3f compute_delta(
    const nav_msgs::msg::Odometry& previous,
    const nav_msgs::msg::Odometry& current
)
{
    Eigen::Vector3f previous_pose(
        previous.pose.pose.position.x,
        previous.pose.pose.position.y,
        extract_yaw(previous.pose.pose.orientation)
    );

    Eigen::Vector3f current_pose(
        current.pose.pose.position.x,
        current.pose.pose.position.y,
        extract_yaw(current.pose.pose.orientation)
    );

    Eigen::Vector3f delta = current_pose - previous_pose;
    delta[2] = normalize_angle(delta[2]);

    return delta;
}


double extract_yaw(const geometry_msgs::msg::Quaternion& q)
{
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);

    return normalize_angle(std::atan2(siny_cosp, cosy_cosp));
}

double uniform_zero_one()
{
    static std::mt19937 gen(std::random_device{}());
    static std::uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(gen);
}

double normalize_angle(double radians)
{
    if (std::isnan(radians))
        return 0.0;

    radians = std::fmod(radians + M_PI, 2 * M_PI);

    if (radians < 0.0)
        radians += 2 * M_PI;

    radians -= M_PI;

    return radians;
}