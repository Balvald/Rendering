#pragma once

#include <Eigen/Dense>

#include "ray.h"

class Triangle
{
public:
    Triangle() = default;

    Triangle(Eigen::Vector3d x1, Eigen::Vector3d x2, Eigen::Vector3d x3) : v1(x1), v2(x2), v3(x3) {}

    Eigen::Vector3d v1;
    Eigen::Vector3d v2;
    Eigen::Vector3d v3;

    /*
    bool hit(Ray& r, double& t) const
    {
        Eigen::Vector3d e1 = v2 - v1;
        Eigen::Vector3d e2 = v3 - v1;

        Eigen::Vector3d pvec = r.direction().cross(e2);
        double det = e1.dot(pvec);

        if (det < 0.00001) return false; // no intersection

        double inv_det = 1.0 / det;

        Eigen::Vector3d tvec = r.origin() - v1;
        double u = tvec.dot(pvec) * inv_det;

        if (u < 0.0 || u > 1.0) return false; // no intersection

        Eigen::Vector3d qvec = tvec.cross(e1);
        double v = r.direction().dot(qvec) * inv_det;

        if (v < 0.0 || u + v > 1.0) return false; // no intersection

        t = e2.dot(qvec) * inv_det;

        return true; // intersection found
    };*/

    Eigen::Vector3d normal() const
    {
        Eigen::Vector3d edge1 = v2 - v1;
        Eigen::Vector3d edge2 = v3 - v1;
        return edge1.cross(edge2).normalized();
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
