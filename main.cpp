//
//  Florian Probst
//  E-Mail: probstf@informatik.uni-freiburg.de / derbalvald@gmail.com
//

// (interesting more than one light source)
// with phong shiny, diffuse, show examples for the report.
// analysis of features in phong

// TODO: fix acceleration datastructures (bvh and bvh with sah)


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

// #define USE_MPI

// #ifdef USE_MPI
// #include <mpi.h>
// #endif


inline void to_color(const Eigen::Vector3d &pixel_color, unsigned char* result)
{
    double r = pixel_color.x();
    double g = pixel_color.y();
    double b = pixel_color.z();

    constexpr double scale = 1.0 / 1.0;

    r = r * scale;
    g = g * scale;
    b = b * scale;

    result[0] = static_cast<unsigned char>(256.0 * std::clamp(r, 0.0, 0.999));
    result[1] = static_cast<unsigned char>(256.0 * std::clamp(g, 0.0, 0.999));
    result[2] = static_cast<unsigned char>(256.0 * std::clamp(b, 0.0, 0.999));
}

inline void write_color(std::ostream &out, const Eigen::Vector3d &pixel_color)
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

void load_model(const std::string &model_path,
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

    // before I forgot to clamp the dot product, which after the application of the std::pow function would lead to values that are way too big
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

void recursive_bvh_build(BoundingVolumeHierarchy current_node,
                         std::vector<Triangle> &triangles,
                         std::vector<Eigen::Vector3d> &vertices,
                         BVH_Tree &bvh_tree,
                         int max_depth = 10, int current_depth = 0,
                         int method = 0, int max_trig = 20, int num_buckets = 8)
{
    if (current_depth >= max_depth || current_node.triangle_indices.size() <= max_trig)
    {
        // log_file << "Reached max depth or leaf node with " << current_node.triangle_indices.size() << " triangles.\n";
        // log_file << "current_depth: " << current_depth << ", max_depth: " << max_depth << "\n";
        // Current node is left a leaf node!
        return;
    }

    // log_file << "Building BVH at depth " << current_depth << " with " << current_node.triangle_indices.size() << " triangles.\n";
    //  : current_node.split_SAH(triangles, vertices, current_node)
    std::vector<BoundingVolumeHierarchy> children;
    children.reserve(2);
    if (method == 3)
        children = current_node.split_SAH(triangles, vertices, num_buckets);
    else if (method == 2)
        children = current_node.split(triangles, vertices);
    else if (method == 1)
        children = current_node.split_x(triangles, vertices);
    else
        return; // method 0 or otherwise not specified we don't split at all. we just go with the root node being a leaf.

    BoundingVolumeHierarchy& left_child = children[0];
    BoundingVolumeHierarchy& right_child = children[1];

    // write out child triangle indices size
    // log_file << "Left child has " << left_child.triangle_indices.size() << " triangles.\n";
    // log_file << "Right child has " << right_child.triangle_indices.size() << " triangles.\n";

    // Add the left and right children to the BVH tree
    // log_file << "left child is supposed to be at index " << bvh_tree.size() << "\n";
    int left_child_index = static_cast<int>(bvh_tree.size());
    bvh_tree.nodes.push_back(left_child);
    int right_child_index = static_cast<int>(bvh_tree.size());
    bvh_tree.nodes.push_back(right_child);
    // log_file << "current node (parent) get_own_index: " << current_node.get_own_index() << " vs. " << bvh_tree.size() << " as size of bvh_tree\n";
    bvh_tree.nodes[current_node.get_own_index()].set_left_child_index(left_child_index);
    bvh_tree.nodes[left_child_index].set_own_index(static_cast<int>(left_child_index)); // Set own index for the left child
    bvh_tree.nodes[left_child_index].set_parent_index(current_node.get_own_index());
    // log_file << "right child is supposed to be at index " << bvh_tree.size()-1 << "\n";
    bvh_tree.nodes[current_node.get_own_index()].set_right_child_index(right_child_index);
    bvh_tree.nodes[right_child_index].set_own_index(static_cast<int>(right_child_index)); // Set own index for the right child
    bvh_tree.nodes[right_child_index].set_parent_index(current_node.get_own_index());

    // print children indices of current node
    // log_file << "Current node has left child at index " << current_node.get_left_child_index() << " and right child at index " << current_node.get_right_child_index() << ".\n";

    // update the parent index of the children
    // log_file << "Current node has index: " << current_node.get_own_index() << ".\n";
    // log_file << "Current node has parent index " << current_node.get_parent_index() << ".\n";

    // Recursively build the left and right children
    recursive_bvh_build(bvh_tree.nodes[left_child_index], triangles, vertices, bvh_tree, max_depth, current_depth + 1, method, max_trig);
    recursive_bvh_build(bvh_tree.nodes[right_child_index], triangles, vertices, bvh_tree, max_depth, current_depth + 1, method, max_trig);
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


    // loading models

    std::string path = std::string(".\\models\\cube.obj");

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
    // args: sah (sah is the surface area heuristic, which is used to build the BVH tree) --sah

    // Define light and material properties (add before the render loop)
    // (x, y, z)  (assuming x left (-) to right (+), y up (+) to down (-), z back to front)
    Eigen::Vector3d light_pos(0, 0, 0);
    Eigen::Vector3d light_color(1, 1, 1); // white light
    Eigen::Vector3d ambient_light_color(0.1, 0.1, 0.1); // ambient light color

    double schininess = 32.0; // :D

    // Camera
    Camera cam = Camera(Eigen::Vector3d(0, 0, 0));


    // Image
    int image_width = 1000;
    int image_height = static_cast<int>(image_width / cam.get_aspect_ratio());

    // Handle command line arguments

    bool image_width_set = false;
    bool image_height_set = false;

    int method = 0;
    int max_depth = 1;
    int max_trig = 1000000000;
    int num_buckets = 8;

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
        else if (arg == "--sah")
        {
            method = 3;
        }
        else if (arg == "--num-buckets")
        {
            num_buckets = std::stoi(argv[++i]);
        }
        else if (arg == "--longest-extend")
        {
            method = 2;
        }
        else if (arg == "--split-x")
        {
            method = 1;
        }
        else if (arg == "--method")
        {
            method = std::stoi(argv[++i]);
        }
        else if (arg == "--max-depth")
        {
            max_depth = std::stoi(argv[++i]);
        }
        else if (arg == "--max-trig")
        {
            max_trig = std::stoi(argv[++i]);
        }
        image_height = static_cast<int>(image_width / cam.get_aspect_ratio());
    }

    std::ofstream log_file;
    std::string log_file_name = "log-" + path.substr(9) + "-"
                                       + std::to_string(method) + "-"
                                       + std::to_string(max_depth) + "-"
                                       + std::to_string(max_trig) + "-"
                                       + std::to_string(num_buckets) + ".txt";
    log_file.open(log_file_name);
    log_file << "Hello I am rank " << rank << " of " << size << "\n";
    log_file << "Camera aspect ratio: " << cam.get_aspect_ratio() << "\n";


    cimg_library::CImg<float> image(image_width, image_height, 1, 3, 0);

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    // std::vector<Triangle> triangles = std::vector<Triangle>();

    std::vector<Eigen::Vector3d> vertices = {};
    std::vector<Triangle> triangles = {};

    std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d>> BoundingBoxes = {};

    load_model(path, &attrib, &shapes, &materials);

    // Loop over shapes
    for (tinyobj::shape_t shape : shapes)
    {

        // Loop over faces(polygon)
        size_t index_offset = 0;

        log_file << "shape.mesh.indices.size(): " << shape.mesh.indices.size() << std::endl;
        log_file << "shape.mesh.num_face_vertices.size(): " << shape.mesh.num_face_vertices.size() << std::endl;

        for (long long f = 0; f < shape.mesh.num_face_vertices.size(); f++)
        {
            // going to a single face

            size_t fv = shape.mesh.num_face_vertices[static_cast<size_t>(f)];
        
            // log_file << "fv: " << fv << std::endl;
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
        
                // log_file << "v[" << index.vertex_index << "] = (" << vx << ", " << vy << ", " << vz << ")" << std::endl;

                // log_file << attrib.normals.size() << std::endl;

                // Check if `normal_index` is zero or positive. negative = no normal data
                if (index.normal_index >= 0)
                {
                    double nx = attrib.normals[3*index.normal_index+0];
                    double ny = attrib.normals[3*index.normal_index+1];
                    double nz = attrib.normals[3*index.normal_index+2];
                    face_normals.emplace_back(nx, ny, nz);
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

                // log_file << "v[" << index.vertex_index << "] = (" << vx << ", " << vy << ", " << vz << ")" << std::endl;

                face_vertices.emplace_back(vx, vy, vz);
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

            // log_file << "Triangle: (" << t.v1.x() << ", " << t.v1.y() << ", " << t.v1.z() << ")," << std::endl;
            // log_file << "(" << t.v2.x() << ", " << t.v2.y() << ", " << t.v2.z() << ")," << std::endl;
            // log_file << "(" << t.v3.x() << ", " << t.v3.y() << ", " << t.v3.z() << ")" << std::endl;

            t.v1i = static_cast<int>(vertices.size());
            vertices.push_back(face_vertices[0]);
            t.v2i = static_cast<int>(vertices.size());
            vertices.push_back(face_vertices[1]);
            t.v3i = static_cast<int>(vertices.size());
            vertices.push_back(face_vertices[2]);
            triangles.push_back(t);

            index_offset += fv;
        
            // per-face material
            // shape.mesh.material_ids[f];
        }
    }

    log_file << "Number of triangles: " << triangles.size() << "\n";

    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

    // Create root node for the Scene

    std::vector<int> triangle_indices;
    std::vector<int> vertex_indices;
    triangle_indices.reserve(triangles.size());
    vertex_indices.reserve(vertices.size());

    for (int i = 0; i < triangles.size(); ++i)
        triangle_indices.push_back(i);
    for (int i = 0; i < vertices.size(); ++i)
        vertex_indices.push_back(i);

    // get bounding box of scene
    Eigen::Vector3d min_corner = vertices[0];
    Eigen::Vector3d max_corner = vertices[0];

    for (int i = 0; i < vertices.size(); ++i)
    {
        Eigen::Vector3d v = vertices[i];
        if (v.x() < min_corner.x())
            min_corner.x() = v.x();
        if (v.y() < min_corner.y())
            min_corner.y() = v.y();
        if (v.z() < min_corner.z())
            min_corner.z() = v.z();
        if (v.x() > max_corner.x())
            max_corner.x() = v.x();
        if (v.y() > max_corner.y())
            max_corner.y() = v.y();
        if (v.z() > max_corner.z())
            max_corner.z() = v.z();
    }

    std::tuple<Eigen::Vector3d, Eigen::Vector3d> scene_bounding_box = std::make_tuple(min_corner, max_corner);

    BoundingVolumeHierarchy root_node = BoundingVolumeHierarchy(scene_bounding_box, triangle_indices, vertex_indices, -1, -1, -1, 0);

    BVH_Tree bvh_tree = BVH_Tree(root_node);

    recursive_bvh_build(bvh_tree.nodes[0], triangles, vertices, bvh_tree, max_depth, 0, method, max_trig, num_buckets);

    std::chrono::steady_clock::time_point bvh_end = std::chrono::steady_clock::now();

    // cast rays and check for intersections
    log_file << "Rendering image...\n";

    std::chrono::steady_clock::time_point render_start = std::chrono::steady_clock::now();

    long long finished_pixels = 0;
    long long total_pixels = image_width * image_height;

    #pragma omp parallel for
    for (int idx = 0; idx < image_width * image_height; ++idx)
    {
        int i = idx % image_width;
        int j = image_height - 1 - (idx / image_width);
        // Compute normalized coordinates
        double u = static_cast<double>(i) / (image_width - 1);
        double v = static_cast<double>(j) / (image_height - 1);

        // Generate ray from camera
        Ray ray = cam.get_ray(u, v);

        // Check for intersections with triangles
        double closest_t = std::numeric_limits<double>::max();
        bool hit_anything = false;

        Triangle closest_triangle;

        // go through bounding volume hierarchy
        // log_file << "Hit basic node" << std::endl;
        // If the node has children, we need to traverse the BVH tree
        std::vector<int> bvh_index_stack;
        bvh_index_stack.reserve(max_depth * 2);
        bvh_index_stack.push_back(0);

        while (!bvh_index_stack.empty())
        {
            // log_file << "Now checking out bvh node: " << bvh_index_stack.back() << " in Pixel (" << i << ", " << j << ")" << std::endl;
            const int current_index = bvh_index_stack.back();
            BoundingVolumeHierarchy& node = bvh_tree.get_node(current_index);
            bvh_index_stack.pop_back();

            // Check if the ray intersects with the bounding box of the node
            if (hit_boundingbox(ray, node.bounding_box))
            {
                // log_file << "Ray intersects with bounding box of node: " << current_index << " in Pixel (" << i << ", " << j << ")" << std::endl;
                // print children indices
                // log_file << "Left child index: " << node.left_child_index << ", Right child index: " << node.right_child_index << std::endl;

                // Add children to the stack
                if (node.left_child_index != -1)
                {
                    // log_file << "Pushing left child: " << node.left_child_index << " to stack." << std::endl;
                    bvh_index_stack.push_back(node.left_child_index);
                }
                if (node.right_child_index != -1)
                {
                    // log_file << "Pushing right child: " << node.right_child_index << " to stack." << std::endl;
                    bvh_index_stack.push_back(node.right_child_index);
                }
                if (node.left_child_index == -1 && node.right_child_index == -1)
                {
                    // log_file << "Found Leaf node: " << node.own_index << " in Pixel (" << i << ", " << j << ")" << std::endl;
                    // If the node has no children, we can check for intersections directly
                    // #pragma omp parallel for
                    // for (int l = 0; l < triangles.size(); ++l)
                    for (int l : node.triangle_indices)
                    {
                        // log_file << "Checking triangle: " << l << " in Pixel (" << i << ", " << j << ")" << std::endl;

                        double t = std::numeric_limits<double>::max();
                        Eigen::Vector3d intersection_point = Eigen::Vector3d::Zero();

                        if (triangles[l].hit(ray, intersection_point, t))
                        {
                            bool closer = (t < closest_t);
                            #pragma omp critical
                            if (closer)
                            {
                                closest_t = t;
                                closest_triangle = triangles[l];
                                hit_anything = true;
                            }
                        }
                    }
                }
            }
            else
            {
                // log_file << "Ray does not intersect with bounding box of node: " << current_index << " in Pixel (" << i << ", " << j << ")" << std::endl;
            }
        }

        unsigned char color[3];

        // Output color based on hit
        // Make color dependent on normal of the triangle
        if (hit_anything)
        {
            // log_file << "Coloring :D" << std::endl;
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
            
            // log_file << "Phong color: (" << color_vec.x() << ", " << color_vec.y() << ", " << color_vec.z() << ")" << std::endl;
            
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
            // log_file << "Color: (" << (int)color[0] << ", " << (int)color[1] << ", " << (int)color[2] << ")" << std::endl;
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
        //log_file << "\rProgress: " << (100.0 * finished_pixels / total_pixels) << "% (" << finished_pixels << "/" << total_pixels << ")" << std::endl;

    }

    std::stringstream concat;
    concat << "render-" << "phong-test-3" << "-" << image_width << "x" << image_height <<
        "-method-" << method << "-md-" << max_depth <<"-mt-" << max_trig << "-" << path.substr(9) << ".bmp";
    std::string filename = concat.str();

    auto test = image.save(filename.c_str());

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::chrono::duration<double> bvh_end_seconds = bvh_end - start;
    std::chrono::duration<double> render_end_seconds = end - render_start;
    log_file << "Full Rendering Pass finished in " << elapsed_seconds.count() << " seconds.\n";
    log_file << "BVH building finished in " << bvh_end_seconds.count() << " seconds.\n";
    log_file << "Rendering finished in " << render_end_seconds.count() << " seconds.\n";

    log_file << "\rDone.                 \n";
    log_file << "Goodbye from rank " << rank << "\n";

    log_file.close();

#ifdef USE_MPI
    MPI_Finalize();
#endif

    return 0;
}