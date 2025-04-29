//
//  Florian Probst
//  E-Mail: probstf@informatik.uni-freiburg.de / derbalvald@gmail.com
//

#include <Eigen/Dense>
#include <iostream>
#include <filesystem>
#include <fstream>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#define USE_MPI

#ifdef USE_MPI
#include <mpi.h>
#endif


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

    int image_width = 256;
    int image_height = 256;

    // Render

    std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";

    for (int j = 0; j < image_height; ++j) {
        std::clog << "\rScanlines remaining: " << (image_height - j) << ' ' << std::flush;
        for (int i = 0; i < image_width; i++) {
            auto r = double(i) / (image_width-1);
            auto g = double(j) / (image_height-1);
            auto b = 0.0;

            int ir = int(255.999 * r);
            int ig = int(255.999 * g);
            int ib = int(255.999 * b);

            std::cout << ir << ' ' << ig << ' ' << ib << '\n';
        }
    }

    std::clog << "\rDone.                 \n";

    std::cout << "Goodbye from rank " << rank << "\n";

#ifdef USE_MPI
    MPI_Finalize();
#endif

    return 0;
}