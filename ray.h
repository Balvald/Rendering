//
//  Florian Probst
//  E-Mail: probstf@informatik.uni-freiburg.de / derbalvald@gmail.com
//

#pragma once

#include <Eigen/Dense>

class Ray
{

    public:
        Ray() = default;
        Ray(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction) : orig(origin), dir(direction) {}

        Eigen::Vector3d origin() const { return orig; }
        Eigen::Vector3d direction() const { return dir; }

        Eigen::Vector3d at(double t) const { return orig + (t * dir); }

        Eigen::Vector3d orig;
        Eigen::Vector3d dir;
};
