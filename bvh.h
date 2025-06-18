#pragma once

#include <Eigen/Dense>

#include <vector>
#include "triangle.h"

class BoundingVolumeHierarchy
{
    public:
    std::tuple<Eigen::Vector3d, Eigen::Vector3d> bounding_box;

    std::vector<int> triangle_indices;
    std::vector<int> vertex_indices;

    int parent_index = -1;          // if this stays -1, it means this is the root node
    int left_child_index = -1;      // if this stays -1, it means this node has no left child
    int right_child_index = -1;     // if this stays -1, it means this node has no right child

    BoundingVolumeHierarchy(const std::tuple<Eigen::Vector3d, Eigen::Vector3d>& bounding_box,
                            const std::vector<int>& triangle_indices,
                            const std::vector<int>& vertex_indices,
                            int parent_index = -1,
                            int left_child_index = -1,
                            int right_child_index = -1)
        : bounding_box(bounding_box),
          triangle_indices(triangle_indices),
          vertex_indices(vertex_indices),
          parent_index(parent_index),
          left_child_index(left_child_index),
          right_child_index(right_child_index) {}

    std::tuple<BoundingVolumeHierarchy, BoundingVolumeHierarchy> split(std::vector<Triangle> all_triangles, std::vector<Eigen::Vector3d> all_vertices) const
    {
        // Split the bounding box into two halves
        Eigen::Vector3d min = std::get<0>(bounding_box);
        Eigen::Vector3d max = std::get<1>(bounding_box);
        Eigen::Vector3d mid = (min + max) / 2.0;

        // Create new triangle and vertex indices for the left and right children
        std::vector<int> left_triangle_indices;
        std::vector<int> right_triangle_indices;
        std::vector<int> left_vertex_indices;
        std::vector<int> right_vertex_indices;

        // Split triangle indices based on the bounding box
        for (int index : triangle_indices)
        {
            Triangle triangle = all_triangles[index];
            Eigen::Vector3d triangle_min = triangle.get_min();
            Eigen::Vector3d triangle_max = triangle.get_max();

            // Check if the triangle is in the left half
            if (triangle_max.x() <= mid.x())
            {
                left_triangle_indices.push_back(index);
                // find the index of a vertex that is part of this triangle
                // all_vertices has all vertices
                for (const auto& vertex : {triangle.v1, triangle.v2, triangle.v3})
                {
                    auto it = std::find(all_vertices.begin(), all_vertices.end(), vertex);
                    if (it != all_vertices.end())
                    {
                        int vertex_index = std::distance(all_vertices.begin(), it);
                        if (std::find(left_vertex_indices.begin(), left_vertex_indices.end(), vertex_index) == left_vertex_indices.end())
                        {
                            left_vertex_indices.push_back(vertex_index);
                        }
                    }
                }
            }
            // Check if the triangle is in the right half
            else if (triangle_min.x() >= mid.x())
            {
                right_triangle_indices.push_back(index);
                for (const auto& vertex : {triangle.v1, triangle.v2, triangle.v3})
                {
                    auto it = std::find(all_vertices.begin(), all_vertices.end(), vertex);
                    if (it != all_vertices.end())
                    {
                        int vertex_index = std::distance(all_vertices.begin(), it);
                        if (std::find(right_vertex_indices.begin(), right_vertex_indices.end(), vertex_index) == right_vertex_indices.end())
                        {
                            right_vertex_indices.push_back(vertex_index);
                        }
                    }
                }
            }
        }

        // Create left and right bounding boxes
        BoundingVolumeHierarchy left_child(
            std::make_tuple(min, mid),
            triangle_indices, vertex_indices, -1, -1, -1);

        BoundingVolumeHierarchy right_child(
            std::make_tuple(mid, max),
            triangle_indices, vertex_indices, -1, -1, -1);

        return std::make_tuple(left_child, right_child);
    }

    std::tuple<BoundingVolumeHierarchy, BoundingVolumeHierarchy> split_SAH(std::vector<Triangle> all_triangles, std::vector<Eigen::Vector3d> all_vertices)
    {
        // SAH (Surface Area Heuristic) is a more complex algorithm that requires calculating the surface area of the bounding boxes
        // and determining the best split based on the distribution of triangles and vertices.






    }

    double surface_area(std::tuple<Eigen::Vector3d, Eigen::Vector3d> box) const
    {
        Eigen::Vector3d min = std::get<0>(box);
        Eigen::Vector3d max = std::get<1>(box);
        Eigen::Vector3d dimensions = max - min;
        return 2.0 * (dimensions.x() * dimensions.y()
                      + dimensions.x() * dimensions.z()
                      + dimensions.y() * dimensions.z());
    }
};


class BVH_Tree
{
    public:
    std::vector<BoundingVolumeHierarchy> nodes;

    BVH_Tree(BoundingVolumeHierarchy root)
    {
        nodes.push_back(root);
    }

    BVH_Tree(const std::vector<BoundingVolumeHierarchy>& nodes)
        : nodes(nodes) {}

    void add_node(const BoundingVolumeHierarchy& node)
    {
        nodes.push_back(node);
    }
    
    BoundingVolumeHierarchy& get_node(int index)
    {
        return nodes[index];
    }

    BoundingVolumeHierarchy get_root()
    {
        return nodes[0];
    }

    int size() const
    {
        return nodes.size();
    }

    void clear()
    {
        nodes.clear();
    }
};