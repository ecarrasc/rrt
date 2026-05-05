#pragma once

#include <fstream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <nav_msgs/msg/occupancy_grid.hpp>

class DistanceGridMap
{
private:
    using Storage_t = Eigen::MatrixXf;

public:
    DistanceGridMap() = default;

    DistanceGridMap(
        const nav_msgs::msg::OccupancyGrid& map,
        float max_distance,
        float occupied_threshold
    );

    Eigen::Vector2f get_origin() const;
    float get_resolution() const;
    Eigen::Vector2i cell_counts() const;
    Eigen::Vector2f world_dimensions() const;

    float distance(int x, int y) const;
    float distance(const Eigen::Vector2i& coord) const;

    float occupancy(int x, int y) const;
    float occupancy(const Eigen::Vector2i& coord) const;

    Eigen::Vector2f cell_to_world(const Eigen::Vector2i& coord) const;
    Eigen::Vector2i world_to_cell(const Eigen::Vector2f& coord) const;

    Eigen::Vector2f random_free_coordinate() const;

private:
    size_t coord_to_index(const Eigen::Vector2i& coord) const;

private:
    int m_cell_width;
    int m_cell_height;

    Storage_t m_distance;
    Storage_t m_occupancy;

    Eigen::Affine3f m_world_pose;

    float m_resolution = 0.1;
    float m_default_value = 0.5;
    float m_max_distance = 5.0;

    Eigen::Vector2i m_center = Eigen::Vector2i::Zero();
    Eigen::Vector2f m_origin = Eigen::Vector2f::Zero();

    std::vector<Eigen::Vector2i> m_free_cells;
};

inline
float DistanceGridMap::distance(int x, int y) const
{
    if (x < 0 || x >= m_cell_width || y < 0 || y >= m_cell_height)
    {
        return m_max_distance;
    }
    return m_distance(y, x);
}

inline
float DistanceGridMap::distance(Eigen::Vector2i const& coord) const
{
    return distance(coord[0], coord[1]);
}

inline
float DistanceGridMap::occupancy(int x, int y) const
{
    if (x < 0 || x >= m_cell_width || y < 0 || y >= m_cell_height)
    {
        return 0.5f;
    }

    return m_occupancy(y, x);
}

inline
float DistanceGridMap::occupancy(Eigen::Vector2i const& coord) const
{
    return occupancy(coord[0], coord[1]);
}

inline
Eigen::Vector2f DistanceGridMap::cell_to_world(Eigen::Vector2i const& coord) const
{
    return m_origin + (coord.cast<float>() * m_resolution);
}

inline
Eigen::Vector2i DistanceGridMap::world_to_cell(Eigen::Vector2f const& coord) const
{
    return ((coord - m_origin) / m_resolution).cast<int>();
}

inline
size_t DistanceGridMap::coord_to_index(Eigen::Vector2i const& coord) const
{
    return coord[1] * m_cell_width + coord[0];
}