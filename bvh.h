#pragma once

#include <Eigen/Dense>

#include <vector>
#include "triangle.h"

class BoundingVolumeHierarchy
{
    public:
    std::vector<Triangle> triangles;
    std::vector<Eigen::Vector3d> vertices;
    std::tuple<Eigen::Vector3d, Eigen::Vector3d> bounding_box;

    // theoretically allows for more than two children if thats desired
    // but currently only two children are created in the constructors
    std::vector<BoundingVolumeHierarchy> children;
    
    size_t max_triangles = 5;


    BoundingVolumeHierarchy(std::vector<Triangle> triangles, std::vector<Eigen::Vector3d> vertices,
                            std::tuple<Eigen::Vector3d, Eigen::Vector3d> bounding_box)
        : triangles(triangles), vertices(vertices), bounding_box(bounding_box)
    {
        // split triangles onto children if they exceed max_triangles
        if (triangles.size() > max_triangles)
        {
            Eigen::Vector3d center = (std::get<0>(bounding_box) + std::get<1>(bounding_box)) / 2.0;
            std::vector<Triangle> left_triangles;
            std::vector<Triangle> right_triangles;
            std::vector<Eigen::Vector3d> left_vertices;
            std::vector<Eigen::Vector3d> right_vertices;

            #pragma omp parallel for
            for (int i = 0; i < triangles.size(); ++i)
            {
                Eigen::Vector3d centroid = (triangles[i].v1 + triangles[i].v2 + triangles[i].v3) / 3.0;
                if (centroid.x() < center.x())
                {
                    left_triangles.push_back(triangles[i]);
                    // check if vertices are already in the left_vertices vector
                    if (std::find(left_vertices.begin(), left_vertices.end(), triangles[i].v1) == left_vertices.end())
                        left_vertices.push_back(triangles[i].v1);
                    if (std::find(left_vertices.begin(), left_vertices.end(), triangles[i].v2) == left_vertices.end())
                        left_vertices.push_back(triangles[i].v2);
                    if (std::find(left_vertices.begin(), left_vertices.end(), triangles[i].v3) == left_vertices.end())
                        left_vertices.push_back(triangles[i].v3);
                }
                else
                {
                    right_triangles.push_back(triangles[i]);
                    // check if vertices are already in the right_vertices vector
                    if (std::find(right_vertices.begin(), right_vertices.end(), triangles[i].v1) == right_vertices.end())
                        right_vertices.push_back(triangles[i].v1);
                    if (std::find(right_vertices.begin(), right_vertices.end(), triangles[i].v2) == right_vertices.end())
                        right_vertices.push_back(triangles[i].v2);
                    if (std::find(right_vertices.begin(), right_vertices.end(), triangles[i].v3) == right_vertices.end())
                        right_vertices.push_back(triangles[i].v3);
                }
            }

            if (!left_triangles.empty())
            {
                children.emplace_back(left_triangles, left_vertices, std::make_tuple(std::get<0>(bounding_box), center));
            }
            if (!right_triangles.empty())
            {
                children.emplace_back(right_triangles, right_vertices, std::make_tuple(center, std::get<1>(bounding_box)));
            }
        }

    }

    BoundingVolumeHierarchy(std::vector<Triangle> triangles, std::vector<Eigen::Vector3d> vertices)
        : triangles(triangles), vertices(vertices)
    {
        // Calculate bounding box from vertices
        if (vertices.empty())
        {
            bounding_box = std::make_tuple(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
            return;
        }

        Eigen::Vector3d min = vertices[0];
        Eigen::Vector3d max = vertices[0];

        for (const auto& v : vertices)
        {
            min = min.cwiseMin(v);
            max = max.cwiseMax(v);
        }

        bounding_box = std::make_tuple(min, max);

// split triangles onto children if they exceed max_triangles
        if (triangles.size() > max_triangles)
        {
            Eigen::Vector3d center = (std::get<0>(bounding_box) + std::get<1>(bounding_box)) / 2.0;
            std::vector<Triangle> left_triangles;
            std::vector<Triangle> right_triangles;
            std::vector<Eigen::Vector3d> left_vertices;
            std::vector<Eigen::Vector3d> right_vertices;

            #pragma omp parallel for
            for (int i = 0; i < triangles.size(); ++i)
            {
                Eigen::Vector3d centroid = (triangles[i].v1 + triangles[i].v2 + triangles[i].v3) / 3.0;
                if (centroid.x() < center.x())
                {
                    left_triangles.push_back(triangles[i]);
                    // check if vertices are already in the left_vertices vector
                    if (std::find(left_vertices.begin(), left_vertices.end(), triangles[i].v1) == left_vertices.end())
                        left_vertices.push_back(triangles[i].v1);
                    if (std::find(left_vertices.begin(), left_vertices.end(), triangles[i].v2) == left_vertices.end())
                        left_vertices.push_back(triangles[i].v2);
                    if (std::find(left_vertices.begin(), left_vertices.end(), triangles[i].v3) == left_vertices.end())
                        left_vertices.push_back(triangles[i].v3);
                }
                else
                {
                    right_triangles.push_back(triangles[i]);
                    // check if vertices are already in the right_vertices vector
                    if (std::find(right_vertices.begin(), right_vertices.end(), triangles[i].v1) == right_vertices.end())
                        right_vertices.push_back(triangles[i].v1);
                    if (std::find(right_vertices.begin(), right_vertices.end(), triangles[i].v2) == right_vertices.end())
                        right_vertices.push_back(triangles[i].v2);
                    if (std::find(right_vertices.begin(), right_vertices.end(), triangles[i].v3) == right_vertices.end())
                        right_vertices.push_back(triangles[i].v3);
                }
            }

            if (!left_triangles.empty())
            {
                children.emplace_back(left_triangles, left_vertices, std::make_tuple(std::get<0>(bounding_box), center));
            }
            if (!right_triangles.empty())
            {
                children.emplace_back(right_triangles, right_vertices, std::make_tuple(center, std::get<1>(bounding_box)));
            }
        }
    }

    BoundingVolumeHierarchy(std::vector<Triangle> triangles, std::vector<Eigen::Vector3d> vertices,
                            size_t max_triangles)
        : triangles(triangles), vertices(vertices), max_triangles(max_triangles)
    {
        // Calculate bounding box from vertices
        if (vertices.empty())
        {
            bounding_box = std::make_tuple(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
            return;
        }

        Eigen::Vector3d min = vertices[0];
        Eigen::Vector3d max = vertices[0];

        for (const auto& v : vertices)
        {
            min = min.cwiseMin(v);
            max = max.cwiseMax(v);
        }

        bounding_box = std::make_tuple(min, max);

// split triangles onto children if they exceed max_triangles
        if (triangles.size() > max_triangles)
        {
            Eigen::Vector3d center = (std::get<0>(bounding_box) + std::get<1>(bounding_box)) / 2.0;
            std::vector<Triangle> left_triangles;
            std::vector<Triangle> right_triangles;
            std::vector<Eigen::Vector3d> left_vertices;
            std::vector<Eigen::Vector3d> right_vertices;

            #pragma omp parallel for
            for (int i = 0; i < triangles.size(); ++i)
            {
                Eigen::Vector3d centroid = (triangles[i].v1 + triangles[i].v2 + triangles[i].v3) / 3.0;
                if (centroid.x() < center.x())
                {
                    left_triangles.push_back(triangles[i]);
                    // check if vertices are already in the left_vertices vector
                    if (std::find(left_vertices.begin(), left_vertices.end(), triangles[i].v1) == left_vertices.end())
                        left_vertices.push_back(triangles[i].v1);
                    if (std::find(left_vertices.begin(), left_vertices.end(), triangles[i].v2) == left_vertices.end())
                        left_vertices.push_back(triangles[i].v2);
                    if (std::find(left_vertices.begin(), left_vertices.end(), triangles[i].v3) == left_vertices.end())
                        left_vertices.push_back(triangles[i].v3);
                }
                else
                {
                    right_triangles.push_back(triangles[i]);
                    // check if vertices are already in the right_vertices vector
                    if (std::find(right_vertices.begin(), right_vertices.end(), triangles[i].v1) == right_vertices.end())
                        right_vertices.push_back(triangles[i].v1);
                    if (std::find(right_vertices.begin(), right_vertices.end(), triangles[i].v2) == right_vertices.end())
                        right_vertices.push_back(triangles[i].v2);
                    if (std::find(right_vertices.begin(), right_vertices.end(), triangles[i].v3) == right_vertices.end())
                        right_vertices.push_back(triangles[i].v3);
                }
            }

            if (!left_triangles.empty())
            {
                children.emplace_back(left_triangles, left_vertices, std::make_tuple(std::get<0>(bounding_box), center));
            }
            if (!right_triangles.empty())
            {
                children.emplace_back(right_triangles, right_vertices, std::make_tuple(center, std::get<1>(bounding_box)));
            }
        }
    }

    std::vector<Triangle> get_triangles() const
    {
        return triangles;
    }

    size_t get_triangles_count() const
    {
        return triangles.size();
    }

    inline bool hit(const Ray& r) const
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
};