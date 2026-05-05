#pragma once

#include <set>
#include <unordered_map>
#include <vector>
#include <limits>

#include <Eigen/Core>

class Graph
{
public:
    using Index_t = size_t;
    using Point_t = Eigen::Vector2d;
    using EdgeSet_t = std::unordered_map<Index_t, std::set<Index_t>>;

public:
    Graph() = default;

    Index_t add_vertex(const Point_t& point);
    void add_edge(Index_t vertex_a, Index_t vertex_b);
    Index_t nearest_vertex(const Point_t& query) const;

    Point_t& operator[](Index_t idx)            { return m_vertices[idx]; }
    const Point_t& operator[](Index_t idx) const { return m_vertices[idx]; }

private:
    std::vector<Point_t> m_vertices;
    EdgeSet_t m_edges;
};