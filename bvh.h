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
    std::vector<int> children_indices;
    
    size_t max_triangles = 5;


};