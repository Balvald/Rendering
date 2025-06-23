#pragma once

#include <Eigen/Dense>
#include <utility>

#include "ray.h"

class Triangle
{
public:
    Triangle() = default;

    Triangle(Eigen::Vector3d x1, Eigen::Vector3d x2, Eigen::Vector3d x3) : v1(std::move(x1)), v2(std::move(x2)), v3(std::move(x3)), v1i(-1), v2i(-1),
                                                                           v3i(-1) {
    }

    Triangle(Eigen::Vector3d  x1, Eigen::Vector3d  x2, Eigen::Vector3d  x3,
             Eigen::Vector3d  n1, Eigen::Vector3d  n2, Eigen::Vector3d  n3) : v1(std::move(x1)), v2(std::move(x2)), v3(std::move(x3)),
        v1i(-1),
        v2i(-1),
        v3i(-1), normal1(std::move(n1)), normal2(std::move(n2)), normal3(std::move(n3)) {
        predefined_normals = true;
    }

    Eigen::Vector3d v1;
    Eigen::Vector3d v2;
    Eigen::Vector3d v3;

    int v1i{};
    int v2i{};
    int v3i{};

    // make use of predefined normals if we have them
    bool predefined_normals = false;
    Eigen::Vector3d normal1;
    Eigen::Vector3d normal2;
    Eigen::Vector3d normal3;

    [[nodiscard]] Eigen::Vector3d get_normal() const
    {
        Eigen::Vector3d edge1 = v2 - v1;
        Eigen::Vector3d edge2 = v3 - v1;
        return edge1.cross(edge2).normalized();
    }

    [[nodiscard]] Eigen::Vector3d get_normal(const double u, const double v) const
    {
        // Barycentric coordinates
        if (predefined_normals)
        {
            return normal1 * (1 - u - v) + normal2 * u + normal3 * v;
        }
        else
        {
            const Eigen::Vector3d edge1 = v2 - v1;
            const Eigen::Vector3d edge2 = v3 - v1;
            return edge1.cross(edge2).normalized();
        }
    }

    void get_barycentric_coordinates(const Eigen::Vector3d &intersection_point, double& u_trig, double& v_trig) const
    {
        Eigen::Vector3d edge1 = v2 - v1;
        Eigen::Vector3d edge2 = v3 - v1;
        Eigen::Vector3d p = intersection_point - v1;

        double d00 = edge1.dot(edge1);
        double d01 = edge1.dot(edge2);
        double d11 = edge2.dot(edge2);
        double d20 = p.dot(edge1);
        double d21 = p.dot(edge2);

        double denom = d00 * d11 - d01 * d01;

        if (denom == 0)
        {
            u_trig = 0;
            v_trig = 0;
            return;
        }

        u_trig = (d11 * d20 - d01 * d21) / denom;
        v_trig = (d00 * d21 - d01 * d20) / denom;
    }

    [[nodiscard]] Eigen::Vector3d get_min() const
    {
        return Eigen::Vector3d(std::min({v1.x(), v2.x(), v3.x()}),
                               std::min({v1.y(), v2.y(), v3.y()}),
                               std::min({v1.z(), v2.z(), v3.z()}));
    }

    [[nodiscard]] Eigen::Vector3d get_max() const
    {
        return Eigen::Vector3d(std::max({v1.x(), v2.x(), v3.x()}),
                               std::max({v1.y(), v2.y(), v3.y()}),
                               std::max({v1.z(), v2.z(), v3.z()}));
    }

    inline bool hit(const Ray& r, Eigen::Vector3d& out, double& t) const
    {
        // double t_min = 0.001;
        // double t_max = std::numeric_limits<double>::max();

        constexpr auto epsilon = static_cast<double>(std::numeric_limits<float>::epsilon());

        const Eigen::Vector3d vertex0 = this->v1;
        const Eigen::Vector3d vertex1 = this->v2;
        const Eigen::Vector3d vertex2 = this->v3;

        const Eigen::Vector3d edge1 = vertex1 - vertex0;
        const Eigen::Vector3d edge2 = vertex2 - vertex0;
        const Eigen::Vector3d h = r.direction().cross(edge2);
        const double a = edge1.dot(h);
        if (a > -epsilon && a < epsilon)
            return false;    // This ray is parallel to this triangle.
        const double f = 1.0 / a;
        Eigen::Vector3d s = r.origin() - vertex0;
        const double u = f * s.dot(h);
        if (u < 0.0 || u > 1.0)
            return false;
        const Eigen::Vector3d q = s.cross(edge1);
        const double v = f * r.direction().dot(q);
        if (v < 0.0 || u + v > 1.0)
            return false;
        // At this stage we can compute t to find out where the intersection point is on the line.
        t = f * edge2.dot(q);
        if (t > epsilon) // ray intersection
        {
            out = r.origin() + r.direction() * t;
            //std::cout << outIntersectionPoint.x() << "," << outIntersectionPoint.y() << "," << outIntersectionPoint.z() << std::endl;
            return true;
        }
        // This means that there is a line intersection but not a ray intersection.
        return false;
    };
};
