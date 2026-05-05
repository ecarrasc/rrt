#include <iostream>
#include <algorithm>   // for std::swap

#include <rrt/graph.hpp>

Graph::Index_t Graph::add_vertex(const Point_t& point)
{
    m_vertices.push_back(point);
    return m_vertices.size() - 1;
}

void Graph::add_edge(Index_t vertex_a, Index_t vertex_b)
{
    // Self links are not permitted
    if (vertex_a == vertex_b)
    {
        std::cout << "Self assignments are invalid." << std::endl;
        return;
    }

    // Ensure vertex_a is always smaller than vertex_b
    if (vertex_b < vertex_a)
    {
        std::swap(vertex_a, vertex_b);
    }

    // Check if both vertex_a and vertex_b are valid nodes
    if (vertex_b >= m_vertices.size())
    {
        std::cout << "Invalid vertex indices provided" << std::endl;
        return;
    }

    // Insert the edge if it doesn't exist already
    // auto edges = m_edges[vertex_a];
    // ⚠️ FIX: reference, not copy
    auto& edges = m_edges[vertex_a];

    if (edges.find(vertex_b) == edges.end())
    {
        edges.insert(vertex_b);
    }
    else
    {
        std::cout << "Edge already present." << std::endl;
    }
}

Graph::Index_t Graph::nearest_vertex(const Point_t& query) const
{
    Index_t best_index = 0;
    double best_distance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < m_vertices.size(); ++i)
    {
        auto dist = (query - m_vertices[i]).norm();
        if (dist < best_distance)
        {
            best_index = i;
            best_distance = dist;
        }
    }

    // std::cout << "Best distance: " << best_distance
    //           << " for vertex " << best_index << std::endl;
    return best_index;
}