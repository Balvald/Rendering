//
//  Florian Probst
//  E-Mail: probstf@informatik.uni-freiburg.de / derbalvald@gmail.com
//

#include <Eigen/Dense>

#include "ray.h"

class Camera
{
public:
    Camera()
    {
        double aspect_ratio = 16.0 / 9.0;
        double viewport_height = 2.0;
        double viewport_width = aspect_ratio * viewport_height;
        double focal_length = 1.0;

        origin = Eigen::Vector3d(0, 0, 5);
        horizontal = Eigen::Vector3d(viewport_width, 0.0, 0.0);
        vertical = Eigen::Vector3d(0.0, viewport_height, 0.0);
        lower_left_corner = origin - horizontal * 0.5 - vertical * 0.5 - Eigen::Vector3d(0, 0, focal_length);
    }

    ray get_ray(double u, double v) const
    {
        return { origin, lower_left_corner + u * horizontal + v * vertical - origin };
    }

private:
    Eigen::Vector3d  origin{};
    Eigen::Vector3d  lower_left_corner{};
    Eigen::Vector3d  horizontal{};
    Eigen::Vector3d  vertical{};
};