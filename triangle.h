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
};
