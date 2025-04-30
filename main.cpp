//
//  Florian Probst
//  E-Mail: probstf@informatik.uni-freiburg.de / derbalvald@gmail.com
//

#include <Eigen/Dense>
#include <iostream>
#include <filesystem>
#include <fstream>

#include "ray.h"
#include "camera.h"

#define CIMG
#include "CImg.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#define USE_MPI

#ifdef USE_MPI
#include <mpi.h>
#endif


inline void to_color(const Eigen::Vector3d pixel_color, unsigned char* result)
{
    double r = pixel_color.x();
    double g = pixel_color.y();
    double b = pixel_color.z();

    const double scale = 1.0 / 1.0;

    r = r * scale;
    g = g * scale;
    b = b * scale;

    result[0] = static_cast<unsigned char>(256.0 * std::clamp(r, 0.0, 0.999));
    result[1] = static_cast<unsigned char>(256.0 * std::clamp(g, 0.0, 0.999));
    result[2] = static_cast<unsigned char>(256.0 * std::clamp(b, 0.0, 0.999));
}

inline void write_color(std::ostream &out, const Eigen::Vector3d pixel_color)
{
    unsigned char result[3];
    to_color(pixel_color, result);
    out << static_cast<int>(result[0]) << ' '
        << static_cast<int>(result[1]) << ' '
        << static_cast<int>(result[2]) << '\n';
}

inline Eigen::Vector3d ray_color(ray& r)
{
    return Eigen::Vector3d(0,0,0);
}

int main(int argc, char *argv[])
{
    int rank = 0, size = 1;

#ifdef USE_MPI
    MPI_Init(&argc, &argv);

    // Retrieve process infos
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &size);
#endif

    std::cout << "Hello I am rank " << rank << " of " << size << "\n";

    // Image

    auto aspect_ratio = 16.0 / 9.0;
    int image_width = 400;

    // Calculate the image height, and ensure that it's at least 1.
    int image_height = int(image_width / aspect_ratio);
    image_height = (image_height < 1) ? 1 : image_height;

    // Camera

    auto focal_length = 1.0;
    auto viewport_height = 2.0;
    auto viewport_width = viewport_height * (double(image_width)/image_height);
    auto camera_center = Eigen::Vector3d(0, 0, 0);

    // Calculate the vectors across the horizontal and down the vertical viewport edges.
    auto viewport_u = Eigen::Vector3d(viewport_width, 0, 0);
    auto viewport_v = Eigen::Vector3d(0, -viewport_height, 0);

    // Calculate the horizontal and vertical delta vectors from pixel to pixel.
    auto pixel_delta_u = viewport_u / image_width;
    auto pixel_delta_v = viewport_v / image_height;

    // Calculate the location of the upper left pixel.
    auto viewport_upper_left = camera_center
                             - Eigen::Vector3d(0, 0, focal_length) - viewport_u/2 - viewport_v/2;
    auto pixel00_loc = viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);

    cimg_library::CImg<double> image(image_width, image_height, 1, 3, 0);

    int samples_per_pixel = 10;

    Camera cam = Camera();

    // Render

    std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";

    for (int j = 0; j < image_height; ++j)
    {
        std::cout << "\rScanlines remaining: " << (image_height - j) << ' ' << std::flush;
        for (int i = 0; i < image_width; i++)
        {
            auto pixel_center = pixel00_loc + (i * pixel_delta_u) + (j * pixel_delta_v);
            auto ray_direction = pixel_center - camera_center;
            ray r = ray(camera_center, ray_direction);

            Eigen::Vector3d pixel_color = ray_color(r);
            // write_color(std::cout, pixel_color);

            unsigned char color[3];

            to_color(pixel_color, color);

            image.draw_point(i, image_height - j, color);
        }
    }

    std::stringstream concat;
    concat << "multisampled" << samples_per_pixel << "-" << image_width << "x" << image_height << ".bmp";
    std::string filename = concat.str();

    auto test = image.save(filename.c_str());


    std::cout << "\rDone.                 \n";
    std::cout << "Goodbye from rank " << rank << "\n";

#ifdef USE_MPI
    MPI_Finalize();
#endif

    return 0;
}