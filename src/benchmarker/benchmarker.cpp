#include "benchmarker.hpp"
#include "common.hpp"
#include "util/misc.hpp"

// OpenVDB
#include <openvdb/Exceptions.h>
#include <openvdb/Types.h>
#include <openvdb/math/Ray.h>
#include <openvdb/math/Transform.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/tools/RayIntersector.h>
#include <openvdb/tools/RayTracer.h> // for Film

#include <cmath>

template <class FP_Type>
Benchmarker<FP_Type>::Benchmarker(const OptionsT &options) : options(options)
{
  using RayIntersectorT = openvdb::tools::LevelSetRayIntersector<GridT>;

  // Create Level Set sphere
  // see
  // https://www.openvdb.org/documentation/doxygen/namespaceopenvdb_1_1v8__0_1_1tools.html#a47e7b3c363d0d3a15b5859c4b06e9d8b
  FP_Type half_width = 2;

  auto level_set = openvdb::tools::createLevelSetSphere<GridT>(
      options["radius"].as<double>(),     // radius of the sphere in world units
      {0, 0, 0},                          // center of the sphere in world units
      options["voxel_size"].as<double>(), // voxel size in world units
      2.0                                 // half the width of the narrow band, in voxel units
  );

  RayIntersectorT lsri(*level_set);

  // numbe rof rays used for the benchmark
  ray_vals = logspace(options["nrays_min"].as<int>(), options["nrays_max"].as<int>(), BASE2,
                      options["nbench"].as<int>());
}

template <class FP_Type> void Benchmarker<FP_Type>::run(size_t n_rays)
{
  assert(n_rays > 0);
  PLOG_INFO << "\nRunning benchmark for " << n_rays << " Rays" << std::endl;

  // generate a circular range of rays with origin at 0,0,0
  // all rays point along the x-y-Plane. z is kept at 0 for now
  Vec3T eye({0, 0, 0});
  std::vector<RayT> rays(n_rays);
  std::vector<Vec3T> reference_solutions(n_rays);
  std::vector<FP_Type> alpha_vals = linspace<FP_Type>(0.0, 2.0 * pi, n_rays);

  for (size_t i = 0; i < n_rays; i++)
  {
    // Generate Rays
    Vec3T direction({
        std::cos(alpha_vals[i]), // x = Cos(α)
        std::sin(alpha_vals[i]), // y = Sin(α)
        0                        // z = 0
    });
    direction.normalize();
    rays[i] = RayT(eye, direction);

    // calculate reference solution
    FP_Type radius = options["radius"].as<FP_Type>();
    Vec3T solution({radius * std::cos(alpha_vals[i]), radius * std::sin(alpha_vals[i]), 0});
    reference_solutions[i] = solution;
  }
}
