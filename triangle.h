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
    }
};
