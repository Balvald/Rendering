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
#include "triangle.h"

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

inline Eigen::Vector3d ray_color(Ray& r)
{
    return Eigen::Vector3d(0,0,0);
}

void load_model(std::string model_path,
                tinyobj::attrib_t* attrib,
                std::vector<tinyobj::shape_t>* shapes,
                std::vector<tinyobj::material_t>* materials)
{
    std::string err;

    if (!tinyobj::LoadObj(attrib, shapes, materials, &err, model_path.c_str()))
    {
        throw std::runtime_error(err);
    }
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

    // loading models

    std::string path = std::string(".\\models\\testlegacy.obj");

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    // std::vector<Triangle> triangles = std::vector<Triangle>();

    std::vector<Triangle> triangles = {
        // Triangle(Eigen::Vector3d(0, 1, -5), Eigen::Vector3d(-1, -1, -5), Eigen::Vector3d(1, -1, -5)),
        // Triangle(Eigen::Vector3d(1, 1, -6), Eigen::Vector3d(0, -1, -6), Eigen::Vector3d(2, -1, -6))
    };

    load_model(path, &attrib, &shapes, &materials);

    for (auto shape : shapes)
    {
        for (int i = 0; i < shape.mesh.indices.capacity(); i+=3)
        {
            auto v11 = shape.mesh.indices[i].vertex_index * 3 + 0;
            auto v12 = shape.mesh.indices[i].vertex_index * 3 + 1;
            auto v13 = shape.mesh.indices[i].vertex_index * 3 + 2;

            auto v21 = shape.mesh.indices[i+1].vertex_index * 3 + 0;
            auto v22 = shape.mesh.indices[i+1].vertex_index * 3 + 1;
            auto v23 = shape.mesh.indices[i+1].vertex_index * 3 + 2;

            auto v31 = shape.mesh.indices[i+2].vertex_index * 3 + 0;
            auto v32 = shape.mesh.indices[i+2].vertex_index * 3 + 1;
            auto v33 = shape.mesh.indices[i+2].vertex_index * 3 + 2;

            std::cout << "v11: " << attrib.vertices[v11] << " v12: " << attrib.vertices[v12] << " v13: " << attrib.vertices[v13] << std::endl;
            std::cout << "v21: " << attrib.vertices[v21] << " v22: " << attrib.vertices[v22] << " v23: " << attrib.vertices[v23] << std::endl;
            std::cout << "v31: " << attrib.vertices[v31] << " v32: " << attrib.vertices[v32] << " v33: " << attrib.vertices[v33] << std::endl;

            triangles.push_back(
                Triangle(
                    Eigen::Vector3d(attrib.vertices[v11], attrib.vertices[v12], attrib.vertices[v13]),
                    Eigen::Vector3d(attrib.vertices[v21], attrib.vertices[v22], attrib.vertices[v23]),
                    Eigen::Vector3d(attrib.vertices[v31], attrib.vertices[v32], attrib.vertices[v33])));
        }
    }

    // Camera
    Camera cam = Camera(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1));

    std::cout << "Camera aspect ratio: " << cam.get_aspect_ratio() << "\n";

    // Image
    int image_width = 1920;
    int image_height = static_cast<int>(image_width / cam.get_aspect_ratio());

    cimg_library::CImg<float> image(image_width, image_height, 1, 3, 0);

    // cast rays and check for intersections
    std::cout << "Rendering image...\n";

    for (int j = image_height - 1; j >= 0; --j) {
        for (int i = 0; i < image_width; ++i) {
            // Compute normalized coordinates
            double u = double(i) / (image_width - 1);
            double v = double(j) / (image_height - 1);

            // Generate ray from camera
            Ray ray = cam.get_ray(u, v);

            // Check for intersections
            double closest_t = std::numeric_limits<double>::max();
            bool hit_anything = false;

            for (const auto& triangle : triangles) {
                double t;
                if (triangle.hit(ray, t) && t < closest_t) {
                    closest_t = t;
                    hit_anything = true;
                }
            }

            unsigned char color[3];

            // Output color based on hit
            if (hit_anything) {
                color[0] = 255; // Red
                color[1] = 0;   // Green
                color[2] = 0;   // Blue
            } else {
                color[0] = 0; // Red
                color[1] = 0;   // Green
                color[2] = 0;   // Blue
            }

            image.draw_point(i, image_height - j, color);
        }
    }

    std::stringstream concat;
    concat << "render-" << "-" << image_width << "x" << image_height << ".bmp";
    std::string filename = concat.str();

    auto test = image.save(filename.c_str());

    std::cout << "\rDone.                 \n";
    std::cout << "Goodbye from rank " << rank << "\n";

#ifdef USE_MPI
    MPI_Finalize();
#endif

    return 0;
}