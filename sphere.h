#pragma once

#include <Eigen/Dense>

#include "ray.h"

// define a sphere object and a method to check if a ray intersects with the sphere

class Sphere
{
public:
    Eigen::Vector3d center;
    double radius;

    Sphere(const Eigen::Vector3d &center, const double radius) : center(center), radius(radius) {}

    inline bool hit(const Ray& r, Eigen::Vector3d& out, double& t) const
    {
        // Ray: p(t) = origin + t * direction
        // Sphere: (p - center).squaredNorm() = radius^2
        Eigen::Vector3d oc = r.origin() - center;
        double a = r.direction().dot(r.direction());
        double b = 2.0 * oc.dot(r.direction());
        double c = oc.dot(oc) - radius * radius;
        double discriminant = b * b - 4 * a * c;

        if (discriminant < 0)
        {
            return false;
        }
        else
        {
            double sqrt_disc = std::sqrt(discriminant);
            double t0 = (-b - sqrt_disc) / (2.0 * a);
            double t1 = (-b + sqrt_disc) / (2.0 * a);

            double t_min = 0.001;
            double t_max = std::numeric_limits<double>::max();

            // Find the nearest t in the valid range
            if (t0 > t_min && t0 < t_max)
            {
                t = t0;
            }
            else if (t1 > t_min && t1 < t_max)
            {
                t = t1;
            }
            else
            {
                return false;
            }
            out = r.origin() + t * r.direction();
            return true;
        }
    }
};
