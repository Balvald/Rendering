//
//  Florian Probst
//  E-Mail: probstf@informatik.uni-freiburg.de / derbalvald@gmail.com
//

#pragma once

#include <Eigen/Dense>
#include <utility>

class Ray {
public:
    Ray() = default;
    Ray(Eigen::Vector3d  origin, Eigen::Vector3d  direction) : orig(std::move(origin)), dir(std::move(direction)) {}

        [[nodiscard]] Eigen::Vector3d origin() const { return orig; }
        [[nodiscard]] Eigen::Vector3d direction() const { return dir; }

        [[nodiscard]] Eigen::Vector3d at(double t) const { return orig + (t * dir); }

        Eigen::Vector3d orig;
        Eigen::Vector3d dir;
};
