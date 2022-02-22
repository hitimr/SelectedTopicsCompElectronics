#include "benchmarker.hpp"
#include "common.hpp"
#include "util/misc.hpp"
#include "util/timer.hpp"

// OpenVDB
#include <openvdb/Exceptions.h>
#include <openvdb/math/Transform.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/LevelSetSphere.h>

// NanoVDB
#include <nanovdb/util/CudaDeviceBuffer.h>
#include <nanovdb/util/GridBuilder.h>
#include <nanovdb/util/HDDA.h>
#include <nanovdb/util/IO.h>
#include <nanovdb/util/Primitives.h>
#include <nanovdb/util/Ray.h>
#include <cmath>

using namespace openvdb;


#if defined(NANOVDB_USE_CUDA)
using BufferT = nanovdb::CudaDeviceBuffer;
#else
using BufferT = nanovdb::HostBuffer;
#endif

Benchmarker::Benchmarker(const OptionsT &options) : options(options)
{

  // numbe rof rays used for the benchmark
  ray_vals = logspace(options["nrays_min"].as<int>(), options["nrays_max"].as<int>(), BASE2,
                      options["nbench"].as<int>());
}

void Benchmarker::run(size_t n_rays)
{

  assert(n_rays > 0);
  PLOG_INFO << "\nRunning benchmark for " << n_rays << " Rays" << std::endl;

  // generate Sphere
  auto level_set = tools::createLevelSetSphere<GridT>(
      options["radius"].as<FP_Type>(),     // radius of the sphere in world units
      {0, 0, 0},                           // center of the sphere in world units
      options["voxel_size"].as<FP_Type>(), // voxel size in world units
      2.0                                  // half the width of the narrow band, in voxel units
  );

  // Ray Intersector
  tools::LevelSetRayIntersector<GridT> ray_intersector(*level_set);

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
    RayT ray = RayT(eye, direction);
    rays[i] = ray;

    // calculate reference solution
    FP_Type radius = options["radius"].as<FP_Type>();
    Vec3T solution({radius * std::cos(alpha_vals[i]), radius * std::sin(alpha_vals[i]), 0});
    reference_solutions[i] = solution;
  }

  std::vector<Vec3T> calculated(n_rays, Vec3T(0, 0, 0)); // results

  // Run Benchmark
  Timer timer;
  timer.reset();
  for (size_t i = 0; i < n_rays; i++)
  {
    ray_intersector.intersectsWS(rays[i], calculated[i]);
  }

  double time = timer.get();
  PLOG_INFO << "Finished in " << time << "s (" << (double)n_rays / (1000 * time) << " kRays/s)"
            << std::endl;

  // Verify Solution
  FP_Type voxel_size = options["voxel_size"].as<FP_Type>();
  FP_Type eps = voxel_size / 2;
  Vec3T vec_eps(eps, eps, eps);
  for (size_t i = 0; i < n_rays; i++)
  {
    assert(openvdb::math::isApproxEqual(calculated[i], reference_solutions[i], vec_eps));
  }

  // results for each ray
  PLOG_DEBUG << "Voxel size: " << voxel_size << std::endl;
  PLOG_DEBUG << "Calculated | reference" << std::endl;
  for (size_t i = 0; i < n_rays; i++)
  {
    PLOG_DEBUG << "Ray " << i << std::endl;
    PLOG_DEBUG << "x: " << calculated[i].x() << "|" << reference_solutions[i].x() << std::endl;
    PLOG_DEBUG << "y: " << calculated[i].y() << "|" << reference_solutions[i].y() << std::endl;
    PLOG_DEBUG << "z: " << calculated[i].z() << "|" << reference_solutions[i].z() << std::endl;
    PLOG_DEBUG << std::endl;
  }
}

void Benchmarker::run_nanoVDB(size_t nrays)
{
  using GridT = nanovdb::FloatGrid;
  using CoordT = nanovdb::Coord;
  using RealT = float;
  using Vec3T = nanovdb::Vec3<RealT>;
  using RayT = nanovdb::Ray<RealT>;


  nanovdb::GridHandle<BufferT> handle;
  handle = nanovdb::createLevelSetSphere<float, float, BufferT>(
      100.0f, nanovdb::Vec3f(-20, 0, 0), 1.0, 3.0, nanovdb::Vec3d(0), "sphere");

  auto *h_grid = handle.grid<float>();
}
