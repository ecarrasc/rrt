#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/rclcpp.hpp>


#include <rrt/rrt.hpp>
#include <rrt/graph.hpp>
#include <rrt/utils.hpp>

#include <functional>

RRT::RRT(rclcpp::Node::SharedPtr node)
    : node_(node)
    , m_start_pose(0.0, 0.0, 0.0)
    , m_goal_pose(0.0, 0.0, 0.0)
{
    // Declare parameters (ROS 2 requires this)
    node_->declare_parameter("map_topic", "/map");
    node_->declare_parameter("initial_pose_topic", "/initialpose");
    node_->declare_parameter("goal_pose_topic", "/goal_pose");

    node_->declare_parameter("iteration_count", 10000);
    node_->declare_parameter("expansion_distance", 0.5);
    node_->declare_parameter("step_size", 0.01);
    node_->declare_parameter("goal_bias", 0.15);

    // Get parameters
    auto map_topic = node_->get_parameter("map_topic").as_string();
    auto start_topic = node_->get_parameter("initial_pose_topic").as_string();
    auto goal_topic = node_->get_parameter("goal_pose_topic").as_string();

    //m_params.iteration_count = node_->get_parameter("iteration_count").as_int();
    m_params.iteration_count = static_cast<int>(node_->get_parameter("iteration_count").as_int());
    m_params.expansion_distance = node_->get_parameter("expansion_distance").as_double();
    m_params.step_size = node_->get_parameter("step_size").as_double();
    m_params.goal_bias = node_->get_parameter("goal_bias").as_double();

    // Subscriptions
    //auto qos = rclcpp::QoS(10).reliable();
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
               .reliable()
               .transient_local();

    m_map_sub = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic, qos,
        std::bind(&RRT::map_cb, this, std::placeholders::_1)
    );

    m_start_pose_sub = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        start_topic, 10,
        std::bind(&RRT::start_pose_cb, this, std::placeholders::_1)
    );

    m_goal_pose_sub = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic, 10,
        std::bind(&RRT::goal_pose_cb, this, std::placeholders::_1)
    );

    // Publisher
    m_path_pub = node_->create_publisher<visualization_msgs::msg::Marker>("rrt_path", 10);
    //m_path_start_stop = node_->create_publisher<visualization_msgs::msg::Marker>("rrt_start_stop", 10);

    RCLCPP_INFO(node_->get_logger(), "RRT initialized successfully");
}

void RRT::compute_path()
{
    rclcpp::Rate rate(5);

    visualization_msgs::msg::Marker msg;

    msg.header.frame_id = "map";
    msg.ns = "rrt";
    msg.action = visualization_msgs::msg::Marker::ADD;

    msg.pose.orientation.w = 1.0;
    msg.id = 1;
    msg.type = visualization_msgs::msg::Marker::LINE_LIST;
    msg.scale.x = 0.02;

    msg.color.r = 0.0;
    msg.color.g = 0.0;
    msg.color.b = 1.0;
    msg.color.a = 1.0;

    Graph g;
    g.add_vertex({m_start_pose[0], m_start_pose[1]});

    bool has_solution = false;
    Eigen::Vector2d q_goal(m_goal_pose[0], m_goal_pose[1]);

    for (int i = 0; i < m_params.iteration_count; ++i)
    {
        if (has_solution){
            RCLCPP_INFO(node_->get_logger(), "Solution found after %d steps.", i);
            break;
        }

        // Sample random free coordinate from the map
        auto q_rand = sample_configuration();

        // Find nearest vertex in the graph
        auto idx_near = g.nearest_vertex(q_rand);
        auto q_near = g[idx_near];

        // Find position along the direction between the nearest vertex and the
        // random location that is free and not further than the specified
        // distance away.
        auto q_new = extend_towards(q_near, q_rand);

        if ((q_new - q_near).norm() < 0.01)
        {
            RCLCPP_WARN(node_->get_logger(), "Expansion blocked");
            continue;
        }

        // Ignore invalid expansions
        if ((q_new - q_near).norm() < 0.01)
            continue;

        // Add new vertex to the graph and add an edge to the nearest vertex
        auto idx_new = g.add_vertex(q_new);
        g.add_edge(idx_near, idx_new);

        // Check if we found a path to the goal and stop if we did
        if ((q_new - q_goal).norm() < 0.1)
            has_solution = true;

        // Visualize graph
        geometry_msgs::msg::Point p1, p2;

        p1.x = q_near[0];
        p1.y = q_near[1];

        p2.x = q_new[0];
        p2.y = q_new[1];

        msg.points.push_back(p1);
        msg.points.push_back(p2);

        RCLCPP_INFO(node_->get_logger(), "Iteration %d: nodes = %zu", i, g.size());

        msg.header.stamp = node_->get_clock()->now();

        m_path_pub->publish(msg);

        //rate.sleep();
    }    
}

Eigen::Vector2d RRT::sample_configuration() const
{
    /*
     * Idea: Sample a random freespace point in the map with a percentage of
     *       those random points being the goal location.
     *
     * - DistanceGridMap class has a method random_free_coordinate, be aware
     *   that this returns Eigen::Vector2f but you need Eigen::Vector2d
     * - utils class has a function uniform_zero_one() which returns a random
     *   number between 0 and 1
     * - the percentage of goal point vs. random point is stored in the
     *   m_params.goal_bias variable
     * - return the final coordinate as an Eigen::Vector2d variable
     */

    // Use float for map
    Eigen::Vector2f free_coord = m_grid_map.random_free_coordinate();

    // Use double for rrt planner
    Eigen::Vector2d coord = free_coord.cast<double>();

    if (uniform_zero_one() < m_params.goal_bias)
    {
        coord = m_goal_pose.head<2>().cast<double>();
    }

    return coord;
}

Eigen::Vector2d RRT::extend_towards(
    Eigen::Vector2d const& from,
    Eigen::Vector2d const& to
) const
{
    /*
     * Idea (simple): 
     *      Find a point along the line between from and to that is that is the
     *      expansion distance away. Then check that the line between from and
     *      that point is free of collisions by checking the map's occupancy
     *      every step size incrments.
     *
     * Idea (advanced):
     *      Find a point not further than the expansion distance from the from
     *      point towards the to point. However, in contrast to the simple idea
     *      try to expand towards the selected point and if you encounter an
     *      obstacle return the last "good" point instead of failing.
     *
     * - DistanceGridMap class can convert world coordinates to cell
     *   coordinates with the world_to_cell method
     * - DistanceGridMap class can return map occupancy values for cell indices
     *   using the occupancy method
     * - A cell that is unknown has an occupancy of 0.5, so you likely want to
     *   be pessimistic, i,e, occupancy values less than 0.5 should be treated
     *   as occupied
     * - the maximum expansion distance is stored in
     *   m_params.expansion_distance
     * - the step size for occupancy checks along the path is stored in
     *   m_params.step_size
     * - if expansion towards the selected point is impossible, e.g. due to a
     *   collision simply return the from point to indicate failur
     */

    Eigen::Vector2d diff = to - from;

    double distance = std::min(diff.norm(), m_params.expansion_distance);

    if (distance < 1e-9)
        return from;

    Eigen::Vector2d direction = diff.normalized();

    int step_count = static_cast<int>(std::floor(distance / m_params.step_size));
    double offset = distance - step_count * m_params.step_size;

    Eigen::Vector2d coord = from + direction * offset;

    for (int i = 0; i < step_count; ++i)
    {
        Eigen::Vector2d tmp_coord = coord + direction * m_params.step_size;

        // boundary conversion: double → float grid map
        Eigen::Vector2f tmp_f = tmp_coord.cast<float>();

        Eigen::Vector2i cell = m_grid_map.world_to_cell(tmp_f);

        float occ = m_grid_map.occupancy(cell);

        if (occ > 0.4)
            break;

        coord = tmp_coord;
    }

    return coord;
}

void RRT::map_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    m_grid_map = DistanceGridMap(*msg, 0.2, 0.6);
    RCLCPP_INFO(node_->get_logger(), "Processed new map");
}

void RRT::start_pose_cb(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    visualization_msgs::msg::Marker start_msg;

    start_msg.header.frame_id = "map";
    start_msg.ns = "rrt_start";
    start_msg.action = visualization_msgs::msg::Marker::ADD;

    start_msg.id = 100;
    start_msg.type = visualization_msgs::msg::Marker::ARROW;
    
    start_msg.scale.x = 1.0;
    start_msg.scale.y = 0.1;
    start_msg.scale.z = 0.1;

    start_msg.color.r = 0.0;
    start_msg.color.g = 1.0;
    start_msg.color.b = 0.0;
    start_msg.color.a = 1.0;

    m_start_pose = Eigen::Vector3f(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        extract_yaw(msg->pose.pose.orientation)
    );

    has_start_ = true;

    if (has_goal_)
        compute_path();
    
    start_msg.pose = msg->pose.pose;
    start_msg.header.stamp = node_->get_clock()->now();

    m_path_pub->publish(start_msg);
}

void RRT::goal_pose_cb(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    visualization_msgs::msg::Marker goal_msg;

    goal_msg.header.frame_id = "map";
    goal_msg.ns = "rrt_goal";
    goal_msg.action = visualization_msgs::msg::Marker::ADD;

    goal_msg.id = 200;
    goal_msg.type = visualization_msgs::msg::Marker::ARROW;
    
    goal_msg.scale.x = 1.0;
    goal_msg.scale.y = 0.1;
    goal_msg.scale.z = 0.1;

    goal_msg.color.r = 1.0;
    goal_msg.color.g = 0.0;
    goal_msg.color.b = 0.0;
    goal_msg.color.a = 1.0;

    m_goal_pose = Eigen::Vector3f(
        msg->pose.position.x,
        msg->pose.position.y,
        extract_yaw(msg->pose.orientation)
    );

  

    has_goal_ = true;

    if (has_start_)
        compute_path();

    goal_msg.pose = msg->pose;
    goal_msg.header.stamp = node_->get_clock()->now();

    m_path_pub->publish(goal_msg);
}