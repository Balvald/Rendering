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
        int split_axis = 0;  // x-axis has largest extent
        if (dimensions.y() > dimensions.x() && dimensions.y() > dimensions.z()) {
            split_axis = 1;  // y-axis has largest extent
        } else if (dimensions.z() > dimensions.x() && dimensions.z() > dimensions.y()) {
            split_axis = 2;  // z-axis has largest extent
        }

        Eigen::Vector3d mid = (min + max) / 2.0;

        // Initialize the bounding boxes for left and right children
        Eigen::Vector3d left_min = min;
        Eigen::Vector3d left_max = max;
        Eigen::Vector3d right_min = min;
        Eigen::Vector3d right_max = max;

        left_max[split_axis] = mid[split_axis];
        right_min[split_axis] = mid[split_axis];

        // compute volume before splitting
        // double volume_before = dimensions.x() * dimensions.y() * dimensions.z();
        // auto left_volume = left_max - left_min;
        // double volume_after = left_volume.x() * left_volume.y() * left_volume.z();
        // auto right_volume = right_max - right_min;
        // double volume_after2 = right_volume.x() * right_volume.y() * right_volume.z();

        // std::cout << "Volume before: " << volume_before << std::endl;
        // std::cout << "Volume after: " << volume_after << std::endl;
        // std::cout << "Volume after2: " << volume_after2 << std::endl;
        // std::cout << "Volume ratio: " << volume_after + volume_after2 << std::endl;

        // Create new triangle and vertex indices for the left and right children
        std::vector<int> left_triangle_indices;
        std::vector<int> right_triangle_indices;
        std::vector<int> left_vertex_indices;
        std::vector<int> right_vertex_indices;

        // iterate over triangle
        for (int i = 0; i < triangle_indices.size(); i++)
        {
            bool in_left = false;
            bool in_right = false;
            Triangle triangle = all_triangles[triangle_indices[i]];

            // Check if vertices of triangle are in left and/or in right bounding box
            Eigen::Vector3d p1 = Eigen::Vector3d(triangle.v1.x(), triangle.v1.y(), triangle.v1.z());
            Eigen::Vector3d p2 = Eigen::Vector3d(triangle.v2.x(), triangle.v2.y(), triangle.v2.z());
            Eigen::Vector3d p3 = Eigen::Vector3d(triangle.v3.x(), triangle.v3.y(), triangle.v3.z());

            if (p1[split_axis] <= left_max[split_axis] || p2[split_axis] <= left_max[split_axis] || p3[split_axis] <= left_max[split_axis])
                in_left = true;
            if (p1[split_axis] >= right_min[split_axis] || p2[split_axis] >= right_min[split_axis] || p3[split_axis] >= right_min[split_axis])
                in_right = true;

            if (in_left) {
                left_triangle_indices.push_back(triangle_indices[i]);
                left_vertex_indices.push_back(triangle.v1i);
                left_vertex_indices.push_back(triangle.v2i);
                left_vertex_indices.push_back(triangle.v3i);
            }

            if (in_right) {
                right_triangle_indices.push_back(triangle_indices[i]);
                right_vertex_indices.push_back(triangle.v1i);
                right_vertex_indices.push_back(triangle.v2i);
                right_vertex_indices.push_back(triangle.v3i);
            }
        }

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