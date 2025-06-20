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
    int own_index = -1;
    int left_child_index = -1;      // if this stays -1, it means this node has no left child
    int right_child_index = -1;     // if this stays -1, it means this node has no right child

    BoundingVolumeHierarchy(const std::tuple<Eigen::Vector3d, Eigen::Vector3d>& bounding_box,
                            const std::vector<int>& triangle_indices,
                            const std::vector<int>& vertex_indices,
                            int parent_index = -1,
                            int left_child_index = -1,
                            int right_child_index = -1, int own_index = -1)
        : bounding_box(bounding_box),
          triangle_indices(triangle_indices),
          vertex_indices(vertex_indices),
          parent_index(parent_index),
          left_child_index(left_child_index),
          right_child_index(right_child_index),
          own_index(own_index) {}

    bool operator==(const BoundingVolumeHierarchy& other) const
    {
        return bounding_box == other.bounding_box;
    }

    [[nodiscard]] std::vector<BoundingVolumeHierarchy> split(std::vector<Triangle> all_triangles, std::vector<Eigen::Vector3d> all_vertices) const
    {
        // Split the bounding box into two halves
        Eigen::Vector3d min = std::get<0>(bounding_box);
        Eigen::Vector3d max = std::get<1>(bounding_box);
        Eigen::Vector3d dimensions = max - min;

        // Find the axis with the largest extent
        int split_axis = 2;
        if (dimensions.y() > dimensions.x() && dimensions.y() > dimensions.z()) {
            split_axis = 1;  // y-axis has largest extent
        } else if (dimensions.z() > dimensions.x() && dimensions.z() > dimensions.y()) {
            split_axis = 0;  // z-axis has largest extent
        }

        Eigen::Vector3d mid = (min + max) / 2.0;


        // Create new triangle and vertex indices for the left and right children
        std::vector<int> left_triangle_indices;
        std::vector<int> right_triangle_indices;
        std::vector<int> left_vertex_indices;
        std::vector<int> right_vertex_indices;

        // Initialize the bounding boxes for left and right children
        Eigen::Vector3d left_min = min;
        Eigen::Vector3d left_max = max;
        Eigen::Vector3d right_min = min;
        Eigen::Vector3d right_max = max;

        // Adjust the split coordinate based on the chosen axis
        left_max[split_axis] = mid[split_axis];
        right_min[split_axis] = mid[split_axis];

        // Distribute triangles between left and right children
        for (int index : triangle_indices) {
            Triangle triangle = all_triangles[index];
            Eigen::Vector3d triangle_center = (triangle.v1 + triangle.v2 + triangle.v3) / 3.0;

            if (triangle_center[split_axis] <= mid[split_axis]) {
                left_triangle_indices.push_back(index);
                // Add vertices to left child
                for (const auto& vertex : {triangle.v1, triangle.v2, triangle.v3}) {
                    auto it = std::find(all_vertices.begin(), all_vertices.end(), vertex);
                    if (it != all_vertices.end()) {
                        int vertex_index = std::distance(all_vertices.begin(), it);
                        if (std::find(left_vertex_indices.begin(), left_vertex_indices.end(), vertex_index) == left_vertex_indices.end()) {
                            left_vertex_indices.push_back(vertex_index);
                        }
                    }
                }
            } else {
                right_triangle_indices.push_back(index);
                // Add vertices to right child
                for (const auto& vertex : {triangle.v1, triangle.v2, triangle.v3}) {
                    auto it = std::find(all_vertices.begin(), all_vertices.end(), vertex);
                    if (it != all_vertices.end()) {
                        int vertex_index = std::distance(all_vertices.begin(), it);
                        if (std::find(right_vertex_indices.begin(), right_vertex_indices.end(), vertex_index) == right_vertex_indices.end()) {
                            right_vertex_indices.push_back(vertex_index);
                        }
                    }
                }
            }
        }

        // adjust left_min and left_max from triangles


        // Create left and right bounding boxes
        BoundingVolumeHierarchy left_child(
            std::make_tuple(left_min, left_max),
            left_triangle_indices, left_vertex_indices, -1, -1, -1);

        BoundingVolumeHierarchy right_child(
            std::make_tuple(right_min, right_max),
            right_triangle_indices, right_vertex_indices, -1, -1, -1);

        std::vector<BoundingVolumeHierarchy> children;
        children.push_back(left_child);
        children.push_back(right_child);

        return children;
    }

    std::vector<BoundingVolumeHierarchy> split_SAH(std::vector<Triangle> all_triangles, std::vector<Eigen::Vector3d> all_vertices) const {



        // Create the bounding boxes for the left and right children based on the best split position
        Eigen::Vector3d left_min = std::get<0>(bounding_box);
        Eigen::Vector3d left_max = std::get<1>(bounding_box);
        Eigen::Vector3d right_min = std::get<0>(bounding_box);
        Eigen::Vector3d right_max = std::get<1>(bounding_box);

        std::vector<int> left_triangle_indices;
        std::vector<int> right_triangle_indices;
        std::vector<int> left_vertex_indices;
        std::vector<int> right_vertex_indices;

        BoundingVolumeHierarchy left_child(
            std::make_tuple(left_min, left_max),
            left_triangle_indices, left_vertex_indices, parent_index, -1, -1);
        BoundingVolumeHierarchy right_child(
            std::make_tuple(right_min, right_max),
            right_triangle_indices, right_vertex_indices, parent_index, -1, -1);

        std::vector<BoundingVolumeHierarchy> children;
        children.push_back(left_child);
        children.push_back(right_child);

        return children;
    }

    static double surface_area(std::tuple<Eigen::Vector3d, Eigen::Vector3d> box)
    {
        Eigen::Vector3d min = std::get<0>(box);
        Eigen::Vector3d max = std::get<1>(box);
        Eigen::Vector3d dimensions = max - min;
        return 2.0 * (dimensions.x() * dimensions.y()
                      + dimensions.x() * dimensions.z()
                      + dimensions.y() * dimensions.z());
    }

    [[nodiscard]] Eigen::Vector3d get_min() const
    {
        return std::get<0>(bounding_box);
    }

    [[nodiscard]] Eigen::Vector3d get_max() const
    {
        return std::get<1>(bounding_box);
    }

    void set_parent_index(const int index)
    {
        parent_index = index;
    }

    void set_left_child_index(const int index)
    {
        left_child_index = index;
    }

    void set_right_child_index(const int index)
    {
        right_child_index = index;
    }

    void set_own_index(const int index)
    {
        own_index = index;
    }

    [[nodiscard]] int get_parent_index() const
    {
        return parent_index;
    }

    [[nodiscard]] int get_left_child_index() const
    {
        return left_child_index;
    }

    [[nodiscard]] int get_right_child_index() const
    {
        return right_child_index;
    }

    [[nodiscard]] int get_own_index() const
    {
        return own_index;
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