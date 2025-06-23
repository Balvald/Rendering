//
//  Florian Probst
//  E-Mail: probstf@informatik.uni-freiburg.de / derbalvald@gmail.com
//
#pragma once

#include <Eigen/Dense>

#include "ray.h"

class Camera
{
public:
    explicit Camera(const Eigen::Vector3d &look_from = Eigen::Vector3d(0, 0, 0))
    {
        viewport_height = 2.0;
        aspect_ratio = 16.0 / 9.0;
        viewport_width = aspect_ratio * viewport_height;
        focal_length = 1.0;

        origin = look_from;
        horizontal = Eigen::Vector3d(viewport_width, 0.0, 0.0);
        vertical = Eigen::Vector3d(0.0, viewport_height, 0.0);
        lower_left_corner = origin - horizontal * 0.5 - vertical * 0.5 - Eigen::Vector3d(0, 0, focal_length);
        
    }

    Camera()
    {
        aspect_ratio = 16.0 / 9.0;
        viewport_height = 2.0;
        viewport_width = aspect_ratio * viewport_height;
        focal_length = 1.0;

        origin = Eigen::Vector3d(0, 0, 0);
        horizontal = Eigen::Vector3d(viewport_width, 0.0, 0.0);
        vertical = Eigen::Vector3d(0.0, viewport_height, 0.0);
        lower_left_corner = origin - horizontal * 0.5 - vertical * 0.5 - Eigen::Vector3d(0, 0, focal_length);
    }

    [[nodiscard]] Ray get_ray(double u, double v) const
    {
        return { origin, lower_left_corner + u * horizontal + v * vertical - origin };
    }

    [[nodiscard]] double get_aspect_ratio() const { return aspect_ratio; }
    void set_aspect_ratio(double aspect_ratio) { this->aspect_ratio = aspect_ratio; }
    [[nodiscard]] double get_viewport_width() const { return viewport_width; }
    [[nodiscard]] double get_viewport_height() const { return viewport_height; }
    [[nodiscard]] double get_focal_length() const { return focal_length; }
    void set_focal_length(double focal_length) { this->focal_length = focal_length; }

    [[nodiscard]] Eigen::Vector3d get_origin() const { return origin; }
    [[nodiscard]] Eigen::Vector3d get_lower_left_corner() const { return lower_left_corner; }
    [[nodiscard]] Eigen::Vector3d get_horizontal() const { return horizontal; }
    [[nodiscard]] Eigen::Vector3d get_vertical() const { return vertical; }

private:
    double aspect_ratio;
    double viewport_width;
    double viewport_height;
    double focal_length;

    Eigen::Vector3d origin{};
    Eigen::Vector3d lower_left_corner{};
    Eigen::Vector3d horizontal{};
    Eigen::Vector3d vertical{};

};