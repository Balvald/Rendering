#pragma once

#include <Eigen/Dense>

#include "ray.h"

class Triangle
{
public:
    Triangle() = default;

    Triangle(Eigen::Vector3d x1, Eigen::Vector3d x2, Eigen::Vector3d x3) : v1(x1), v2(x2), v3(x3) {}

    Triangle(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2, const Eigen::Vector3d& x3,
             const Eigen::Vector3d& n1, const Eigen::Vector3d& n2, const Eigen::Vector3d& n3) : v1(x1), v2(x2), v3(x3), normal1(n1), normal2(n2), normal3(n3)
    {
        predefined_normals = true;
    }

    Eigen::Vector3d v1;
    Eigen::Vector3d v2;
    Eigen::Vector3d v3;

    // make use of predefined normals if we have them
    bool predefined_normals = false;
    Eigen::Vector3d normal1;
    Eigen::Vector3d normal2;
    Eigen::Vector3d normal3;

    Eigen::Vector3d get_normal() const
    {
        Eigen::Vector3d edge1 = v2 - v1;
        Eigen::Vector3d edge2 = v3 - v1;
        return edge1.cross(edge2).normalized();
    }

    Eigen::Vector3d get_normal(double u, double v)
    {
        // Barycentric coordinates
        if (predefined_normals)
        {
            return normal1 * (1 - u - v) + normal2 * u + normal3 * v;
        }
        else
        {
            Eigen::Vector3d edge1 = v2 - v1;
            Eigen::Vector3d edge2 = v3 - v1;
            return edge1.cross(edge2).normalized();
        }
    }

    void get_barycentric_coordinates(Eigen::Vector3d intersection_point, double& u_trig, double& v_trig)
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

    inline bool hit(const Ray& r, Eigen::Vector3d& out, double& t) const
    {
        double t_min = 0.001;
        double t_max = std::numeric_limits<double>::max();

        constexpr double epsilon = FLT_EPSILON;

        Eigen::Vector3d vertex0 = this->v1;
        Eigen::Vector3d vertex1 = this->v2;
        Eigen::Vector3d vertex2 = this->v3;
        Eigen::Vector3d edge1, edge2, h, s, q;
        double a, f, u, v;

        edge1 = vertex1 - vertex0;
        edge2 = vertex2 - vertex0;
        h = r.direction().cross(edge2);
        a = edge1.dot(h);
        if (a > -epsilon && a < epsilon)
            return false;    // This ray is parallel to this triangle.
        f = 1.0 / a;
        s = r.origin() - vertex0;
        u = f * s.dot(h);
        if (u < 0.0 || u > 1.0)
            return false;
        q = s.cross(edge1);
        v = f * r.direction().dot(q);
        if (v < 0.0 || u + v > 1.0)
            return false;
        // At this stage we can compute t to find out where the intersection point is on the line.
        t = f * edge2.dot(q);
        if (t > epsilon) // ray intersection
        {
            Eigen::Vector3d outIntersectionPoint = r.origin() + r.direction() * t;
            //std::cout << outIntersectionPoint.x() << "," << outIntersectionPoint.y() << "," << outIntersectionPoint.z() << std::endl;
            return true;
        }
        // This means that there is a line intersection but not a ray intersection.
        return false;
    };
};
