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

        double box_min_x = std::get<0>(bounding_box).x();
        double box_min_y = std::get<0>(bounding_box).y();
        double box_min_z = std::get<0>(bounding_box).z();
        double box_max_x = std::get<1>(bounding_box).x();
        double box_max_y = std::get<1>(bounding_box).y();
        double box_max_z = std::get<1>(bounding_box).z();

        // Ray-box intersection (Bounding Box, Axis aligned to global axes)
        for (int i = 0; i < 3; ++i) {
            double invD = 1.0 / r.direction()[i];
            double t0 = (std::get<0>(bounding_box)[i] - r.origin()[i]) * invD;
            double t1 = (std::get<1>(bounding_box)[i] - r.origin()[i]) * invD;
            if (invD < 0.0) std::swap(t0, t1);
            t_min = t0 > t_min ? t0 : t_min;
            t_max = t1 < t_max ? t1 : t_max;
            if (t_max <= t_min)
                return false;
        }
        return true;
        }
};