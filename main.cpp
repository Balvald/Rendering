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
#include "bvh.h"
#include "shape.h"

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

Eigen::Vector3d phong(const Eigen::Vector3d& V,
                      const Eigen::Vector3d& N,
                      const Eigen::Vector3d& L,
                      const Eigen::Vector3d& is,
                      const Eigen::Vector3d& id,
                      const Eigen::Vector3d& ia,
                      const double schininess,
                      const double ks,
                      const double kd,
                      const double ka)
{
    // This is the Phong reflection model
    // under the assumption that we have a single point light source

    // shorthand for the reflection model based on the Phong reflection model wikipedia article: https://en.wikipedia.org/wiki/Phong_reflection_model

    // ks specular reflection constant
    // kd diffuse reflection constant
    // ka ambient reflection constant

    // schininess is the shininess constant (alpha) for the material

    // is intensity of specular component
    // id intensity of diffuse component
    // ia intensity of ambient component

    // L_m is the direction vector from the point on the surface toward each light source. (we currently only have one)
    // N is the normal at this point on the surface
    // R_m is the direction that a perfectly reflected ray of light would take.
    // V is the direction pointing towards the viewer

    Eigen::Vector3d R = 2.0 * N.dot(L) * N - L; // reflection direction

    return (ka * ia) + (kd * (N.dot(L)) * id) + (ks * std::pow(std::max(V.dot(R), 0.0), schininess) * is);
}


bool hit_boundingbox(const Ray& r, const std::tuple<Eigen::Vector3d, Eigen::Vector3d>& bounding_box)
{
    double t_min = std::numeric_limits<double>::min();
    double t_max = std::numeric_limits<double>::max();

    // Ray-box intersection (Bounding Box, Axis aligned to global axes)
    Eigen::Vector3d invD = r.direction().cwiseInverse();
    Eigen::Vector3d t0s = (std::get<0>(bounding_box) - r.origin()).cwiseProduct(invD);
    Eigen::Vector3d t1s = (std::get<1>(bounding_box) - r.origin()).cwiseProduct(invD);

    // Swap t0 and t1 where invD < 0, branchlessly
    Eigen::Vector3d tmin_vec = t0s.cwiseMin(t1s);
    Eigen::Vector3d tmax_vec = t0s.cwiseMax(t1s);

    double t_min_new = std::max(t_min, tmin_vec.maxCoeff());
    double t_max_new = std::min(t_max, tmax_vec.minCoeff());

    return t_max_new > t_min_new;
}


int main(int argc, char *argv[])
{

    // Check if the program is run with MPI

    int rank = 0, size = 1;

#ifdef USE_MPI
    MPI_Init(&argc, &argv);

    // Retrieve process infos
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &size);
#endif

    std::cout << "Hello I am rank " << rank << " of " << size << "\n";

    // loading models

    std::string path = std::string(".\\models\\fouranimals.obj");

    // Input handling
    // args: image_width, (image_height) -w -h
    // args: model_path -f
    // args: light_pos -lp
    // args: light_color -lc
    // args: camera_pos --cam-pos
    // args: camera_lookat --cam-lookat
    // args: camera_up --cam-up
    // args: camera_fov --cam-fov
    // args: camera_aspect_ratio (is overwritten if image_width AND image_height are given) --cam-ar

    // Define light and material properties (add before the render loop)
    // (x, y, z)  (assuming x left (-) to right (+), y up (+) to down (-), z back to front)
    Eigen::Vector3d light_pos(0, 0, 0);
    Eigen::Vector3d light_color(1, 1, 1); // white light
    Eigen::Vector3d ambient_light_color(0.1, 0.1, 0.1); // ambient light color

    double schininess = 32.0; // :D

    // Camera
    Camera cam = Camera(Eigen::Vector3d(0, 0, 0));

    std::cout << "Camera aspect ratio: " << cam.get_aspect_ratio() << "\n";

    // Image
    int image_width = 1920;
    int image_height = static_cast<int>(image_width / cam.get_aspect_ratio());

    // Handle command line arguments

    bool image_width_set = false;
    bool image_height_set = false;

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];

        if (arg == "-w" && i + 1 < argc)
        {
            image_width = std::stoi(argv[++i]);
            // image_height = static_cast<int>(image_width / cam.get_aspect_ratio());
        }
        else if (arg == "-h" && i + 1 < argc)
        {
            image_height = std::stoi(argv[++i]);
            // image_width = static_cast<int>(image_height * cam.get_aspect_ratio());
        }
        else if (arg == "-f" && i + 1 < argc)
        {
            path = argv[++i];
        }
        else if (arg == "-lp" && i + 1 < argc)
        {
            light_pos.x() = std::stod(argv[++i]);
            light_pos.y() = std::stod(argv[++i]);
            light_pos.z() = std::stod(argv[++i]);
        }
        else if (arg == "-lc" && i + 1 < argc)
        {
            light_color.x() = std::stod(argv[++i]);
            light_color.y() = std::stod(argv[++i]);
            light_color.z() = std::stod(argv[++i]);
        }
        else if (arg == "--cam-pos" && i + 1 < argc)
        {
            cam.get_origin().x() = std::stod(argv[++i]);
            cam.get_origin().y() = std::stod(argv[++i]);
            cam.get_origin().z() = std::stod(argv[++i]);
        }
        else if (arg == "--cam-lookat" && i + 1 < argc)
        {
            cam.get_lower_left_corner().x() = std::stod(argv[++i]);
            cam.get_lower_left_corner().y() = std::stod(argv[++i]);
            cam.get_lower_left_corner().z() = std::stod(argv[++i]);
        }
        else if (arg == "--cam-up" && i + 1 < argc)
        {
            cam.get_vertical().x() = std::stod(argv[++i]);
            cam.get_vertical().y() = std::stod(argv[++i]);
            cam.get_vertical().z() = std::stod(argv[++i]);
        }
        else if (arg == "--cam-fov" && i + 1 < argc)
        {
            cam.set_focal_length(std::stod(argv[++i]));
        }
        else if (arg == "--cam-ar" && i + 1 < argc)
        {
            if (image_width_set && image_height_set)
            {
                std::cerr << "Error: Cannot set aspect ratio if both image width and height are set." << std::endl;
                return 1;
            }
            cam.set_aspect_ratio(std::stod(argv[++i]));
            if (image_width_set)
            {
                image_height = static_cast<int>(image_width / cam.get_aspect_ratio());
            }
            else if (image_height_set)
            {
                image_width = static_cast<int>(image_height * cam.get_aspect_ratio());
            }
        }
        image_height = static_cast<int>(image_width / cam.get_aspect_ratio());
    }


    cimg_library::CImg<float> image(image_width, image_height, 1, 3, 0);

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    // std::vector<Triangle> triangles = std::vector<Triangle>();

    std::vector<Eigen::Vector3d> vertices = {};
    std::vector<Triangle> triangles = {};

    std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d>> BoundingBoxes = {};

    std::vector<BVH_Tree> bvh_trees = {};

    std::vector<Shape> shape_elements = {};

    load_model(path, &attrib, &shapes, &materials);

    // Loop over shapes
    for (tinyobj::shape_t shape : shapes)
    {
        std::vector<Triangle> shape_triangles = std::vector<Triangle>();
        std::vector<Eigen::Vector3d> shape_vertices = std::vector<Eigen::Vector3d>();

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
            std::vector<Eigen::Vector3d> face_normals = std::vector<Eigen::Vector3d>();

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
                if (index.normal_index >= 0)
                {
                    double nx = attrib.normals[3*index.normal_index+0];
                    double ny = attrib.normals[3*index.normal_index+1];
                    double nz = attrib.normals[3*index.normal_index+2];
                    face_normals.push_back(Eigen::Vector3d(nx, ny, nz));
                }
        
                /*
                // Check if `texcoord_index` is zero or positive. negative = no texcoord data
                if (index.texcoord_index >= 0)
                {
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
            //    face_normals[0],
            //    face_normals[1],
            //    face_normals[2]);

            //Triangle t = Triangle(
            //    face_vertices[0],
            //    face_vertices[1],
            //    face_vertices[2],
            //    face_normals[0],
            //    face_normals[1],
            //    face_normals[2]);

            // std::cout << "Triangle: (" << t.v1.x() << ", " << t.v1.y() << ", " << t.v1.z() << ")," << std::endl;
            // std::cout << "(" << t.v2.x() << ", " << t.v2.y() << ", " << t.v2.z() << ")," << std::endl;
            // std::cout << "(" << t.v3.x() << ", " << t.v3.y() << ", " << t.v3.z() << ")" << std::endl;

            triangles.push_back(t);
            shape_triangles.push_back(t);
            shape_vertices.push_back(face_vertices[0]);
            shape_vertices.push_back(face_vertices[1]);
            shape_vertices.push_back(face_vertices[2]);

            index_offset += fv;
        
            // per-face material
            // shape.mesh.material_ids[f];
        }

        // Build bounding box around the shape
        Eigen::Vector3d min = Eigen::Vector3d::Zero();
        Eigen::Vector3d max = Eigen::Vector3d::Zero();

        for (size_t i = 0; i < shape.mesh.indices.size(); i++)
        {
            tinyobj::index_t index = shape.mesh.indices[i];
            double vx = attrib.vertices[3*index.vertex_index+0];
            double vy = attrib.vertices[3*index.vertex_index+1];
            double vz = attrib.vertices[3*index.vertex_index+2];

            if (i == 0)
            {
                min = Eigen::Vector3d(vx, vy, vz);
                max = Eigen::Vector3d(vx, vy, vz);
            }
            else
            {
                min.x() = std::min(min.x(), vx);
                min.y() = std::min(min.y(), vy);
                min.z() = std::min(min.z(), vz);

                max.x() = std::max(max.x(), vx);
                max.y() = std::max(max.y(), vy);
                max.z() = std::max(max.z(), vz);
            }
        }

        std::cout << "Bounding box: (" << min.x() << ", " << min.y() << ", " << min.z() << "), (" << max.x() << ", " << max.y() << ", " << max.z() << ")" << std::endl;

        // add the bounding box to the list
        BoundingBoxes.push_back(std::make_tuple(min, max));

        std::vector<int> shape_triangle_indices = {};
        for (int i = 0; i < shape_triangles.size(); ++i)
        {
            shape_triangle_indices.push_back(i);
        }
        std::vector<int> shape_vertices_indices = {};
        for (int i = 0; i < shape_vertices.size(); ++i)
        {
            shape_vertices_indices.push_back(i);
        }

        shape_elements.push_back(Shape(shape_triangles, shape_vertices, std::make_tuple(min, max)));
        bvh_trees.push_back(BVH_Tree(BoundingVolumeHierarchy(std::make_tuple(min, max), shape_triangle_indices, shape_vertices_indices, -1, -1, -1)));

        // Split the bounding box into two halves to create a right and left child for the root node
        // This is a simple way to create a BVH tree, but it can be improved by using a more sophisticated algorithm
        if (bvh_trees.back().get_root().left_child_index == -1 && bvh_trees.back().get_root().right_child_index == -1)
        {
            Eigen::Vector3d middle = min + (max - min) / 2.0;

            std::vector<int> left_shape_triangle_indices;
            std::vector<int> left_shape_vertices_indices;

            std::vector<int> right_shape_triangle_indices;
            std::vector<int> right_shape_vertices_indices;

            for (int i = 0; i < shape_triangle_indices.size(); ++i)
            {
                Triangle t = shape_triangles[shape_triangle_indices[i]];
                // Prüfe, ob mindestens ein Vertex im linken Bereich liegt
                bool in_left = (t.v1.x() <= middle.x()) || (t.v2.x() <= middle.x()) || (t.v3.x() <= middle.x());
                // Prüfe, ob mindestens ein Vertex im rechten Bereich liegt
                bool in_right = (t.v1.x() >= middle.x()) || (t.v2.x() >= middle.x()) || (t.v3.x() >= middle.x());

                if (in_left)
                    left_shape_triangle_indices.push_back(shape_triangle_indices[i]);
                if (in_right)
                    right_shape_triangle_indices.push_back(shape_triangle_indices[i]);
            }

            for (int i = 0; i < shape_vertices_indices.size(); ++i)
            {
                Eigen::Vector3d v = shape_vertices[shape_vertices_indices[i]];
                if (v.x() <= middle.x())
                    left_shape_vertices_indices.push_back(shape_vertices_indices[i]);
                if (v.x() >= middle.x())
                    right_shape_vertices_indices.push_back(shape_vertices_indices[i]);
            }

            // **Berechne die Bounding-Box für alle enthaltenen Vertices im linken Kind**
            Eigen::Vector3d left_min = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
            Eigen::Vector3d left_max = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());
            for (int idx : left_shape_vertices_indices)
            {
                const Eigen::Vector3d& v = shape_vertices[idx];
                left_min.x() = left_min.x() < v.x() ? left_min.x() : v.x();
                left_min.y() = left_min.y() < v.y() ? left_min.y() : v.y();
                left_min.z() = left_min.z() < v.z() ? left_min.z() : v.z();
                left_max.x() = left_max.x() > v.x() ? left_max.x() : v.x();
                left_max.y() = left_max.y() > v.y() ? left_max.y() : v.y();
                left_max.z() = left_max.z() > v.z() ? left_max.z() : v.z();
            }

            // **Bounding-Box für rechtes Kind**
            Eigen::Vector3d right_min = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
            Eigen::Vector3d right_max = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());
            for (int idx : right_shape_vertices_indices)
            {
                const Eigen::Vector3d& v = shape_vertices[idx];
                right_min.x() = right_min.x() < v.x() ? right_min.x() : v.x();
                right_min.y() = right_min.y() < v.y() ? right_min.y() : v.y();
                right_min.z() = right_min.z() < v.z() ? right_min.z() : v.z();
                right_max.x() = right_max.x() > v.x() ? right_max.x() : v.x();
                right_max.y() = right_max.y() > v.y() ? right_max.y() : v.y();
                right_max.z() = right_max.z() > v.z() ? right_max.z() : v.z();
            }

            Eigen::Vector3d epsilon(1e-8, 1e-8, 1e-8);

            BoundingVolumeHierarchy left_child_bvh = BoundingVolumeHierarchy(std::make_tuple(left_min-epsilon, left_max+epsilon), left_shape_triangle_indices, left_shape_vertices_indices, -1, -1, -1);
            bvh_trees.back().get_node(0).left_child_index = bvh_trees.back().size();
            bvh_trees.back().add_node(left_child_bvh);

            BoundingVolumeHierarchy right_child_bvh = BoundingVolumeHierarchy(std::make_tuple(right_min-epsilon, right_max+epsilon), right_shape_triangle_indices, right_shape_vertices_indices, -1, -1, -1);
            bvh_trees.back().get_node(0).right_child_index = bvh_trees.back().size();
            bvh_trees.back().add_node(right_child_bvh);

            // Update the parent index of the children
            bvh_trees.back().get_node(bvh_trees.back().get_node(0).left_child_index).parent_index = 0;
            bvh_trees.back().get_node(bvh_trees.back().get_node(0).right_child_index).parent_index = 0;

            // clear indices for the root node as all indices are now in the children
            bvh_trees.back().get_node(0).triangle_indices.clear();
            bvh_trees.back().get_node(0).vertex_indices.clear();

            // print out all indices for each child
            std::cout << "Left child triangle indices: ";
            for (int j = 0; j < left_shape_triangle_indices.size(); ++j)
            {
                std::cout << left_shape_triangle_indices[j] << " ";
            }
            std::cout << "\nLeft child vertex indices: ";
            for (int j = 0; j < left_shape_vertices_indices.size(); ++j)
            {
                std::cout << left_shape_vertices_indices[j] << " ";
            }
            std::cout << "\nRight child triangle indices: ";
            for (int j = 0; j < right_shape_triangle_indices.size(); ++j)
            {
                std::cout << right_shape_triangle_indices[j] << " ";
            }
            std::cout << "\nRight child vertex indices: ";
            for (int j = 0; j < right_shape_vertices_indices.size(); ++j)
            {
                std::cout << right_shape_vertices_indices[j] << " ";
            }
            std::cout << "\n";
            // print out all indices for the root node
            std::cout << "Root node triangle indices: ";
            for (int j = 0; j < shape_triangle_indices.size(); ++j)
            {
                std::cout << shape_triangle_indices[j] << " ";
            }
            std::cout << "\nRoot node vertex indices: ";
            for (int j = 0; j < shape_vertices_indices.size(); ++j)
            {
                std::cout << shape_vertices_indices[j] << " ";
            }
            std::cout << "\n";

            // clear indices for the root node as all indices are now in the children
            bvh_trees.back().get_node(0).triangle_indices.clear();
            bvh_trees.back().get_node(0).vertex_indices.clear();
        }
    }

    // print out all relevant information for each bvh_tree in bvh_trees
    std::cout << "Bounding Volume Hierarchies:\n";
    for (int i = 0; i < bvh_trees.size(); ++i)
    {
        std::cout << "BVH Tree " << i << ":\n";
        std::cout << "Number of nodes: " << bvh_trees[i].size() << "\n";
        std::cout << "Root bounding box: (" 
                  << std::get<0>(bvh_trees[i].get_root().bounding_box).x() << ", "
                  << std::get<0>(bvh_trees[i].get_root().bounding_box).y() << ", "
                  << std::get<0>(bvh_trees[i].get_root().bounding_box).z() << "), ("
                  << std::get<1>(bvh_trees[i].get_root().bounding_box).x() << ", "
                  << std::get<1>(bvh_trees[i].get_root().bounding_box).y() << ", "
                  << std::get<1>(bvh_trees[i].get_root().bounding_box).z() << ")\n";
        std::cout << "Left child index: " << bvh_trees[i].get_root().left_child_index << "\n";
        std::cout << "Right child index: " << bvh_trees[i].get_root().right_child_index << "\n";
        std::cout << "Parent index: " << bvh_trees[i].get_root().parent_index << "\n";
        std::cout << "Triangle indices: ";
        for (int j = 0; j < bvh_trees[i].get_root().triangle_indices.size(); ++j)
        {
            std::cout << bvh_trees[i].get_root().triangle_indices[j] << " ";
        }
        std::cout << "\nVertex indices: ";
        for (int j = 0; j < bvh_trees[i].get_root().vertex_indices.size(); ++j)
        {
            std::cout << bvh_trees[i].get_root().vertex_indices[j] << " ";
        }
        std::cout << "\n\n";
        // print out the bounding boxes of the children
        if (bvh_trees[i].get_root().left_child_index != -1)
        {
            BoundingVolumeHierarchy left_child = bvh_trees[i].get_node(bvh_trees[i].get_root().left_child_index);
            std::cout << "Left child bounding box: (" 
                      << std::get<0>(left_child.bounding_box).x() << ", "
                      << std::get<0>(left_child.bounding_box).y() << ", "
                      << std::get<0>(left_child.bounding_box).z() << "), ("
                      << std::get<1>(left_child.bounding_box).x() << ", "
                      << std::get<1>(left_child.bounding_box).y() << ", "
                      << std::get<1>(left_child.bounding_box).z() << ")\n";
            std::cout << "Triangle indices: ";
            for (int j = 0; j < bvh_trees[i].get_node(bvh_trees[i].get_root().left_child_index).triangle_indices.size(); ++j)
            {
                //std::cout << bvh_trees[i].get_node(bvh_trees[i].get_root().left_child_index).triangle_indices[j] << " ";
            }
            std::cout << "\nVertex indices: ";
            for (int j = 0; j < bvh_trees[i].get_node(bvh_trees[i].get_root().left_child_index).vertex_indices.size(); ++j)
            {
                //std::cout << bvh_trees[i].get_node(bvh_trees[i].get_root().left_child_index).vertex_indices[j] << " ";
            }
            std::cout << "\n\n";
        }
        if (bvh_trees[i].get_root().right_child_index != -1)
        {
            BoundingVolumeHierarchy right_child = bvh_trees[i].get_node(bvh_trees[i].get_root().right_child_index);
            std::cout << "Right child bounding box: (" 
                      << std::get<0>(right_child.bounding_box).x() << ", "
                      << std::get<0>(right_child.bounding_box).y() << ", "
                      << std::get<0>(right_child.bounding_box).z() << "), ("
                      << std::get<1>(right_child.bounding_box).x() << ", "
                      << std::get<1>(right_child.bounding_box).y() << ", "
                      << std::get<1>(right_child.bounding_box).z() << ")\n";
            std::cout << "Triangle indices: ";
            for (int j = 0; j < bvh_trees[i].get_node(bvh_trees[i].get_root().right_child_index).triangle_indices.size(); ++j)
            {
                //std::cout << bvh_trees[i].get_node(bvh_trees[i].get_root().right_child_index).triangle_indices[j] << " ";
            }
            std::cout << "\nVertex indices: ";
            for (int j = 0; j < bvh_trees[i].get_node(bvh_trees[i].get_root().right_child_index).vertex_indices.size(); ++j)
            {
                //std::cout << bvh_trees[i].get_node(bvh_trees[i].get_root().right_child_index).vertex_indices[j] << " ";
            }
            std::cout << "\n\n";
        }
        std::cout << "----------------------------------------\n";
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


    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();


    // cast rays and check for intersections
    std::cout << "Rendering image...\n";

    long long finished_pixels = 0;
    long long total_pixels = image_width * image_height;

    #pragma omp parallel for
    for (int idx = 0; idx < image_width * image_height; ++idx)
    {
        int i = idx % image_width;
        int j = image_height - 1 - (idx / image_width);
        // Compute normalized coordinates
        double u = double(i) / (image_width - 1);
        double v = double(j) / (image_height - 1);

        // Generate ray from camera
        Ray ray = cam.get_ray(u, v);

        // Check for intersections with triangles
        double closest_t = std::numeric_limits<double>::max();
        bool hit_anything = false;

        Triangle closest_triangle;

        for (int k = 0; k < shape_elements.size(); ++k)
        {
            // check if the ray intersects with the bounding box of the shape
            /*if (!shape_elements[k].hit(ray))
            {
                continue; // skip this shape if the ray does not intersect with the bounding box
            }*/
            std::vector<Triangle> shape_triangles = shape_elements[k].get_triangles();

            // go through bounding volume hierarchy
            BVH_Tree bvh_tree = bvh_trees[k];
            BoundingVolumeHierarchy current_node = bvh_tree.get_root();
            // Check if the ray intersects with the bounding box of the BVH root
            if (!hit_boundingbox(ray, current_node.bounding_box))
            {
                continue; // skip this shape if the ray does not intersect with the bounding box
            }

            if (current_node.left_child_index != -1 || current_node.right_child_index != -1)
            {
                // If the node has children, we need to traverse the BVH tree
                std::vector<BoundingVolumeHierarchy> stack;
                stack.push_back(current_node);

                while (!stack.empty())
                {
                    BoundingVolumeHierarchy node = stack.back();
                    stack.pop_back();

                    // Check if the ray intersects with the bounding box of the node
                    if (hit_boundingbox(ray, node.bounding_box))
                    {
                        // Check for intersections with triangles in this node
                        for (int m = 0; m < node.triangle_indices.size(); ++m)
                        {
                            int l = node.triangle_indices[m];

                            Eigen::Vector3d intersection_point;
                            double t;

                            if (shape_triangles[l].hit(ray, intersection_point, t))
                            {
                                if (t < closest_t)
                                {
                                    closest_t = t;
                                    closest_triangle = shape_triangles[l];
                                    hit_anything = true;
                                }
                            }
                        }

                        // Add children to the stack
                        if (node.left_child_index != -1)
                        {
                            stack.push_back(bvh_tree.get_node(node.left_child_index));
                        }
                        if (node.right_child_index != -1)
                        {
                            stack.push_back(bvh_tree.get_node(node.right_child_index));
                        }
                    }
                }
            }
            else
            {
                // If the node has no children, we can check for intersections directly
                for (int m = 0; m < current_node.triangle_indices.size(); ++m)
                {
                    int l = current_node.triangle_indices[m];

                    Eigen::Vector3d intersection_point;
                    double t;

                    if (shape_triangles[l].hit(ray, intersection_point, t))
                    {
                        if (t < closest_t)
                        {
                            closest_t = t;
                            closest_triangle = shape_triangles[l];
                            hit_anything = true;
                        }
                    }
                }
            }

            /*
            // Check for intersections with triangles in the shapes
            for (int l = 0; l < shape_triangles.size(); ++l)
            {
                Eigen::Vector3d intersection_point;
                double t;

                if(shape_triangles[l].hit(ray, intersection_point, t))
                {
                    if (t < closest_t)
                    {
                        closest_t = t;
                        closest_triangle = shape_triangles[l];
                        hit_anything = true;
                    }
                }
            }
            */
        }

        unsigned char color[3];

        // Output color based on hit
        // Make color dependent on normal of the triangle
        if (hit_anything)
        {
            // Intersection point
            Eigen::Vector3d intersection_point = cam.get_origin() + ray.direction() * closest_t;

            // Surface normal
            // Eigen::Vector3d N = closest_triangle.get_normal().normalized(); // N
            // Surface normal at the intersection point
            // get the barycentric coordinates
            double u_trig, v_trig;
            closest_triangle.get_barycentric_coordinates(intersection_point, u_trig, v_trig);

            Eigen::Vector3d N = closest_triangle.get_normal(u_trig, v_trig).normalized();

            // Light direction
            Eigen::Vector3d L = (light_pos - intersection_point).normalized(); // L_light

            // View direction
            Eigen::Vector3d V = (cam.get_origin() - intersection_point).normalized(); // L_cam

            // Reflection direction
            Eigen::Vector3d R = (2.0 * ((N.dot(L)) * N) - L).normalized();  // L_refl

            double ks = 0.7; // specular reflection constant
            double kd = 0.5; // diffuse reflection constant
            double ka = 0.1; // ambient light constant

            // Combine
            Eigen::Vector3d color_vec = phong(V, N, L, light_color, light_color, ambient_light_color, schininess, ks, kd, ka);
            
            // std::cout << "Phong color: (" << color_vec.x() << ", " << color_vec.y() << ", " << color_vec.z() << ")" << std::endl;
            
            // color_vec = color_vec.cwiseMin(1.0).cwiseMax(0.0); // Clamp to [0,1]

            // apply tone mapping
            color_vec.x() = color_vec.x() / (1.0 + color_vec.x());
            color_vec.y() = color_vec.y() / (1.0 + color_vec.y());
            color_vec.z() = color_vec.z() / (1.0 + color_vec.z());

            // apply gamma correction
            // color_vec = color_vec.cwiseSqrt();

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

        //#pragma omp atomic
        //finished_pixels++;

        //#pragma omp critical
        //std::cout << "\rProgress: " << (100.0 * finished_pixels / total_pixels) << "% (" << finished_pixels << "/" << total_pixels << ")" << std::endl;

    }

    std::stringstream concat;
    concat << "render-" << "phong-test-2" << "-" << image_width << "x" << image_height << "-" << path.substr(9) << ".bmp";
    std::string filename = concat.str();

    auto test = image.save(filename.c_str());

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Rendering finished in " << elapsed_seconds.count() << " seconds.\n";

    std::cout << "\rDone.                 \n";
    std::cout << "Goodbye from rank " << rank << "\n";

#ifdef USE_MPI
    MPI_Finalize();
#endif

    return 0;
}