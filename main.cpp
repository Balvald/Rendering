//
//  Florian Probst
//  E-Mail: probstf@informatik.uni-freiburg.de / derbalvald@gmail.com
//

// TODO: make several triangles in the scene
// TODO: Implement Phong. (interesting more than one light source)
// with phong shiny, diffuse, show examples for the report.
// analysis of features in phong

// TODO: make testnew.obj
// TODO: acceleration datastructures (boxes, later bvh)

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

Eigen::Vector3d phong(
    const Eigen::Vector3d& V, // View direction
    const Eigen::Vector3d& R, // Reflection direction
    const Eigen::Vector3d& light_color,
    const double schininess)
{

    double diff = std::max(std::pow(V.dot(R), schininess), 0.0);
    Eigen::Vector3d diffuse = diff * light_color;

    return diffuse;
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

    std::string path = std::string(".\\models\\cube.obj");

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    // std::vector<Triangle> triangles = std::vector<Triangle>();

    std::vector<Eigen::Vector3d> vertices = {};
    std::vector<Eigen::Vector3i> faces = {};

    std::vector<Triangle> triangles = {};

    load_model(path, &attrib, &shapes, &materials);

    // Loop over shapes
    for (auto shape : shapes)
    {
        // Loop over faces(polygon)
        size_t index_offset = 0;

        std::cout << "shape.mesh.indices.size(): " << shape.mesh.indices.size() << std::endl;
        std::cout << "shape.mesh.num_face_vertices.size(): " << shape.mesh.num_face_vertices.size() << std::endl;

        for (long long f = 0; f < shape.mesh.num_face_vertices.size(); f++)
        {
            // going to a single face

            size_t fv = shape.mesh.num_face_vertices[static_cast<size_t>(f)];
        
            // std::cout << "fv: " << fv << std::endl;
            // create vector for the face
            std::vector<Eigen::Vector3d> face_vertices = std::vector<Eigen::Vector3d>();

            // Loop over vertices in the face.
            for (size_t v = 0; v < fv; v++)
            {
                // access to vertex
                tinyobj::index_t index = shape.mesh.indices[index_offset + v];
                double vx = attrib.vertices[3*index.vertex_index+0];
                double vy = attrib.vertices[3*index.vertex_index+1];
                double vz = attrib.vertices[3*index.vertex_index+2];
        
                // std::cout << "v[" << index.vertex_index << "] = (" << vx << ", " << vy << ", " << vz << ")" << std::endl;

                // std::cout << attrib.normals.size() << std::endl;

                // Check if `normal_index` is zero or positive. negative = no normal data
                /*if (index.normal_index >= 0) {
                double nx = attrib.normals[3*index.normal_index+0];
                double ny = attrib.normals[3*index.normal_index+1];
                double nz = attrib.normals[3*index.normal_index+2];
                }
        
                // Check if `texcoord_index` is zero or positive. negative = no texcoord data
                if (index.texcoord_index >= 0) {
                double tx = attrib.texcoords[2*index.texcoord_index+0];
                double ty = attrib.texcoords[2*index.texcoord_index+1];
                }
                */

                // Optional: vertex colors
                // double red   = attrib.colors[3*index.vertex_index)+0];
                // double green = attrib.colors[3*index.vertex_index)+1];
                // double blue  = attrib.colors[3*index.vertex_index)+2];

                // std::cout << "v[" << index.vertex_index << "] = (" << vx << ", " << vy << ", " << vz << ")" << std::endl;

                face_vertices.push_back(Eigen::Vector3d(vx, vy, vz));
            }

            Triangle t = Triangle(
                face_vertices[0],
                face_vertices[1],
                face_vertices[2]);

            // std::cout << "Triangle: (" << t.v1.x() << ", " << t.v1.y() << ", " << t.v1.z() << ")," << std::endl;
            // std::cout << "(" << t.v2.x() << ", " << t.v2.y() << ", " << t.v2.z() << ")," << std::endl;
            // std::cout << "(" << t.v3.x() << ", " << t.v3.y() << ", " << t.v3.z() << ")" << std::endl;

            triangles.push_back(t);

            index_offset += fv;
        
            // per-face material
            // shape.mesh.material_ids[f];
        }
    }

    std::cout << "Number of triangles: " << triangles.size() << "\n";

    // print all triangles
    /*
    for (auto triangle : triangles)
    {
        std::cout << "Triangle: (" << triangle.v1.x() << ", " << triangle.v1.y() << ", " << triangle.v1.z() << ")," << std::endl;
        std::cout << "(" << triangle.v2.x() << ", " << triangle.v2.y() << ", " << triangle.v2.z() << ")," << std::endl;
        std::cout << "(" << triangle.v3.x() << ", " << triangle.v3.y() << ", " << triangle.v3.z() << ")" << std::endl;
    }
    */

    // Define light and material properties (add before the render loop)
    Eigen::Vector3d light_pos(0, 0, 0);
    Eigen::Vector3d light_color(1, 1, 1); // white light

    double schininess = 10.0; // lol thats a good typo, Gonna keep it for now

    // Camera
    Camera cam = Camera(Eigen::Vector3d(0, 0, 0));

    std::cout << "Camera aspect ratio: " << cam.get_aspect_ratio() << "\n";

    // Image
    int image_width = 1920;
    int image_height = static_cast<int>(image_width / cam.get_aspect_ratio());

    cimg_library::CImg<float> image(image_width, image_height, 1, 3, 0);

    // cast rays and check for intersections
    std::cout << "Rendering image...\n";

    long long finished_pixels = 0;
    long long total_pixels = image_width * image_height;

    #pragma omp parallel for
    for (int j = image_height - 1; j >= 0; --j)
    {
        #pragma omp parallel for
        for (int i = 0; i < image_width; ++i)
        {
            // Compute normalized coordinates
            double u = double(i) / (image_width - 1);
            double v = double(j) / (image_height - 1);

            // Generate ray from camera
            Ray ray = cam.get_ray(u, v);

            // Check for intersections with triangles
            double closest_t = std::numeric_limits<double>::max();
            bool hit_anything = false;

            Triangle closest_triangle;

            for (Triangle triangle : triangles)
            {
                Eigen::Vector3d intersection_point;
                double t;

                if(triangle.hit(ray, intersection_point, t))
                {
                    if (t < closest_t)
                    {
                        closest_t = t;
                        closest_triangle = triangle;
                        hit_anything = true;
                    }
                }
            }

            // TODO: intersection with bounding boxes

            unsigned char color[3];

            // Output color based on hit
            // Make color dependent on normal of the triangle
            if (hit_anything)
            {

                // Intersection point
                Eigen::Vector3d intersection_point = cam.get_origin() + ray.direction() * closest_t;

                // Surface normal
                Eigen::Vector3d N = closest_triangle.normal();

                // Light direction
                Eigen::Vector3d L = (light_pos - intersection_point).normalized(); // L_light

                // View direction
                Eigen::Vector3d V = (cam.get_origin() - intersection_point).normalized(); // L_cam

                // Reflection direction
                Eigen::Vector3d R = (2.0 * N.dot(L) * N - L).normalized();  // L_refl

                // Combine
                Eigen::Vector3d color_vec = phong(V, R, light_color, schininess); //ambient + diffuse + specular;
                color_vec = color_vec.cwiseMin(1.0).cwiseMax(0.0); // Clamp to [0,1]

                color[0] = static_cast<unsigned char>(255 * color_vec.x());
                color[1] = static_cast<unsigned char>(255 * color_vec.y());
                color[2] = static_cast<unsigned char>(255 * color_vec.z());

                // use the normal of the triangle to determine color
                //color[0] = static_cast<unsigned char>(255 * (1.0 - ray.direction().dot(closest_triangle.normal())));
                //color[1] = static_cast<unsigned char>(255 * (1.0 - ray.direction().dot(closest_triangle.normal())));
                //color[2] = static_cast<unsigned char>(255 * (1.0 - ray.direction().dot(closest_triangle.normal())));

                // print color
                // std::cout << "Color: (" << (int)color[0] << ", " << (int)color[1] << ", " << (int)color[2] << ")" << std::endl;
            }
            else
            {
                color[0] = 0; // Red
                color[1] = 0; // Green
                color[2] = 0; // Blue
            }

            image.draw_point(i, image_height - j, color);

            #pragma omp atomic
            finished_pixels++;

            // std::cout << "\rProgress: " << (100.0 * finished_pixels / total_pixels) << "% (" << finished_pixels << "/" << total_pixels << ")" << std::endl;

        }
    }

    std::stringstream concat;
    concat << "render-" << "phong-test-2" << "-" << image_width << "x" << image_height << ".bmp";
    std::string filename = concat.str();

    auto test = image.save(filename.c_str());

    std::cout << "\rDone.                 \n";
    std::cout << "Goodbye from rank " << rank << "\n";

#ifdef USE_MPI
    MPI_Finalize();
#endif

    return 0;
}