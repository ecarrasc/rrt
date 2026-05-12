#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <rrt/distance_grid_map.hpp>
#include <rrt/utils.hpp>


DistanceGridMap::DistanceGridMap(
    const nav_msgs::msg::OccupancyGrid& map,
    float max_distance,
    float occupied_threshold
)
    : m_cell_width(map.info.width)
    , m_cell_height(map.info.height)
    , m_distance(Storage_t::Zero(map.info.height, map.info.width))
    , m_occupancy(Storage_t::Zero(map.info.height, map.info.width))
    , m_resolution(map.info.resolution)
    , m_max_distance(max_distance)
    , m_origin(map.info.origin.position.x,
               map.info.origin.position.y)
{
    // ROS occupancy maps are [0, 100] for known cells and -1 for unknown
    int threshold = occupied_threshold * 100.0;

    for (int y = 0; y < m_cell_height; ++y)
    {
        for (int x = 0; x < m_cell_width; ++x)
        {
            auto index = y * m_cell_width + x;

            if (map.data[index] > threshold)
                m_distance(y, x) = 0.0;
            else
                m_distance(y, x) = max_distance;

            if (map.data[index] < 40 && map.data[index] != -1)
                m_free_cells.push_back({x, y});

            m_occupancy(y, x) = map.data[index] / 100.0;
            if (map.data[index] == -1)
                m_occupancy(y, x) = 0.5;
        }
    }

    int free_count = 0;
    int occupied_count = 0;
    int unknown_count = 0;

    for (auto val : map.data)
    {
        if (val == -1)
            unknown_count++;
        else if (val < 40)
            free_count++;
        else
            occupied_count++;
    }

    RCLCPP_INFO(rclcpp::get_logger("rrt"),
        "Map stats -> free: %d, occupied: %d, unknown: %d, total: %zu",
        free_count, occupied_count, unknown_count, map.data.size());

    RCLCPP_INFO(rclcpp::get_logger("rrt"),
        "Free cells stored: %zu", m_free_cells.size());

    Eigen::Matrix3f step_distance;
    step_distance <<
        std::sqrt(2.0) * m_resolution, m_resolution, std::sqrt(2.0) * m_resolution,
        m_resolution, 0, m_resolution,
        std::sqrt(2.0) * m_resolution, m_resolution, std::sqrt(2.0) * m_resolution;

    for (int y = 1; y < m_cell_height - 1; ++y)
    {
        for (int x = 1; x < m_cell_width - 1; ++x)
        {
            m_distance(y, x) =
                (m_distance.block(y-1, x-1, 3, 3) + step_distance).minCoeff();
        }
    }

    for (int y = m_cell_height - 2; y > 0; --y)
    {
        for (int x = m_cell_width - 2; x > 0; --x)
        {
            m_distance(y, x) =
                (m_distance.block(y-1, x-1, 3, 3) + step_distance).minCoeff();
        }
    }
}

Eigen::Vector2f DistanceGridMap::get_origin() const
{
    return m_origin;
}

float DistanceGridMap::get_resolution() const
{
    return m_resolution;
}

Eigen::Vector2i DistanceGridMap::cell_counts() const
{
    return {m_cell_width, m_cell_height};
}

Eigen::Vector2f DistanceGridMap::world_dimensions() const
{
    return {
        m_cell_width * m_resolution,
        m_cell_height * m_resolution
    };
}

Eigen::Vector2f DistanceGridMap::random_free_coordinate() const
{
    //std::cout << "free cells: " << m_free_cells.size() << std::endl;
    size_t index = m_free_cells.size() * uniform_zero_one();
    return cell_to_world(m_free_cells[index]);
}