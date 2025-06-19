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

    bool operator==(const BoundingVolumeHierarchy& other) const
    {
        return bounding_box == other.bounding_box;
    }

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

    std::tuple<BoundingVolumeHierarchy, BoundingVolumeHierarchy> split_SAH(std::vector<Triangle> all_triangles, std::vector<Eigen::Vector3d> all_vertices, BoundingVolumeHierarchy parent)
    {
        // SAH (Surface Area Heuristic) is a more complex algorithm that requires calculating the surface area of the bounding boxes
        // and determining the best split based on the distribution of triangles and vertices.

        double best_cost = std::numeric_limits<double>::max();
        double best_split_position = 0.0;
        double best_axis = 0; // 0 for x, 1 for y, 2 for z

        int num_buckets = 8; // Number of buckets for SAH

        double C_trav = 1.0;
        double C_intersect = 1.0;

        double S_total_surface_area = surface_area(parent.bounding_box);

        Eigen::Vector3d min = std::get<0>(parent.bounding_box);
        Eigen::Vector3d max = std::get<1>(parent.bounding_box);

        // Iterate over each axis (x, y, z)
        for (int axis = 0; axis < 3; ++axis)
        {
            // Create buckets for the current axis
            std::vector<std::vector<int>> buckets(num_buckets);
            std::vector<double> bucket_surface_areas(num_buckets, 0.0);
            std::vector<int> bucket_triangle_counts(num_buckets, 0);

            // Distribute triangles into buckets based on their centroids
            for (int index : parent.triangle_indices)
            {
                Triangle triangle = all_triangles[index];
                Eigen::Vector3d centroid = (triangle.v1 + triangle.v2 + triangle.v3) / 3.0;

                int bucket_index = static_cast<int>((centroid[axis] - min[axis]) / (max[axis] - min[axis]) * num_buckets);
                bucket_index = std::clamp(bucket_index, 0, num_buckets - 1);

                buckets[bucket_index].push_back(index);
                bucket_triangle_counts[bucket_index]++;
            }

            // Calculate surface areas and costs for each split position
            double left_area = 0.0;
            double right_area = S_total_surface_area;

            for (int i = 0; i < num_buckets - 1; ++i)
            {
                if (bucket_triangle_counts[i] > 0)
                {
                    Eigen::Vector3d left_min = min;
                    Eigen::Vector3d left_max = max;
                    left_max[axis] = min[axis] + (max[axis] - min[axis]) * (i + 1) / num_buckets;

                    double left_bucket_area = surface_area(std::make_tuple(left_min, left_max));
                    left_area += left_bucket_area * bucket_triangle_counts[i];
                    right_area -= left_bucket_area * bucket_triangle_counts[i];

                    double cost = C_trav + C_intersect * (left_area + right_area);
                    if (cost < best_cost)
                    {
                        best_cost = cost;
                        best_split_position = (min[axis] + max[axis]) / 2.0;
                        best_axis = axis;
                    }
                }
            }
        }

        // Create the bounding boxes for the left and right children based on the best split position
        Eigen::Vector3d left_min = std::get<0>(parent.bounding_box);
        Eigen::Vector3d left_max = std::get<1>(parent.bounding_box);
        Eigen::Vector3d right_min = std::get<0>(parent.bounding_box);
        Eigen::Vector3d right_max = std::get<1>(parent.bounding_box);

        if (best_axis == 0) // x-axis
        {
            left_max.x() = best_split_position;
            right_min.x() = best_split_position;
        }
        else if (best_axis == 1) // y-axis
        {
            left_max.y() = best_split_position;
            right_min.y() = best_split_position;
        }
        else if (best_axis == 2) // z-axis
        {
            left_max.z() = best_split_position;
            right_min.z() = best_split_position;
        }

        BoundingVolumeHierarchy left_child(
            std::make_tuple(left_min, left_max),
            parent.triangle_indices, parent.vertex_indices, parent.parent_index, -1, -1);
        BoundingVolumeHierarchy right_child(
            std::make_tuple(right_min, right_max),
            parent.triangle_indices, parent.vertex_indices, parent.parent_index, -1, -1);

        return std::make_tuple(left_child, right_child);
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

    BoundingVolumeHierarchy& get_root()
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