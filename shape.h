#pragma once

#include <Eigen/Dense>

#include <vector>
#include "triangle.h"

class Shape
{
    public:
        std::vector<Triangle> triangles;
        std::vector<Eigen::Vector3d> vertices;
        std::tuple<Eigen::Vector3d, Eigen::Vector3d> bounding_box;

    Shape(std::vector<Triangle> triangles, std::vector<Eigen::Vector3d> vertices,
          std::tuple<Eigen::Vector3d, Eigen::Vector3d> bounding_box)
        : triangles(triangles), vertices(vertices), bounding_box(bounding_box) {}

    Shape(std::vector<Triangle> triangles, std::vector<Eigen::Vector3d> vertices) : triangles(triangles), vertices(vertices)
    {

    }

    std::vector<Triangle> get_triangles() const
    {
        return triangles;
    }

    std::vector<Eigen::Vector3d> get_vertices() const
    {
        return vertices;
    }

    std::tuple<Eigen::Vector3d, Eigen::Vector3d> get_bounding_box() const
    {
        return bounding_box;
    }

    inline bool hit(const Ray& r) const
    {
        double t_min = std::numeric_limits<double>::min();
        double t_max = std::numeric_limits<double>::max();

        // Ray-box intersection (Bounding Box, Axis aligned to global axes)
        Eigen::Vector3d invD = r.direction().cwiseInverse();
        Eigen::Vector3d t0s = (std::get<0>(bounding_box) - r.origin()).cwiseProduct(invD);
        Eigen::Vector3d t1s = (std::get<1>(bounding_box) - r.origin()).cwiseProduct(invD);

        // Swap t0 and t1 where invD < 0, branchlessly
        Eigen::Vector3d tmin_vec = t0s.cwiseMin(t1s);
        Eigen::Vector3d tmax_vec = t0s.cwiseMax(t1s);

        double t_min_new = std::max(t_min, tmin_vec.maxCoeff());
        double t_max_new = std::min(t_max, tmax_vec.minCoeff());

        return t_max_new > t_min_new;
    }
};