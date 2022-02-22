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
#include <cmath>
#include <nanovdb/util/CudaDeviceBuffer.h>
#include <nanovdb/util/GridBuilder.h>
#include <nanovdb/util/HDDA.h>
#include <nanovdb/util/IO.h>
#include <nanovdb/util/Primitives.h>
#include <nanovdb/util/Ray.h>

using namespace openvdb;

#if defined(NANOVDB_USE_CUDA)
using BufferT = nanovdb::CudaDeviceBuffer;
#else
using BufferT = nanovdb::HostBuffer;
#endif

Benchmarker::Benchmarker(const OptionsT &options) : options(options)
{

  // set number of rays for the benchmark
  ray_vals = logspace(options["nrays_min"].as<int>(), options["nrays_max"].as<int>(), BASE2,
                      options["nbench"].as<int>());

  // Sphere Parameters
  sphere_radius_outer = options["radius"].as<FP_Type>();
  voxel_size = options["voxel_size"].as<FP_Type>();
  level_set_half_width = 2.0;
}

void Benchmarker::run(size_t n_rays)
{

  assert(n_rays > 0);
  PLOG_INFO << "\nRunning benchmark for " << n_rays << " Rays" << std::endl;

  // generate Sphere
  auto level_set = tools::createLevelSetSphere<GridT>(
      sphere_radius_outer, // radius of the sphere in world units
      Vec3T(0, 0, 0),      // center of the sphere in world units
      voxel_size,          // voxel size in world units
      level_set_half_width // half the width of the narrow band, in voxel units
  );

  // Ray Intersector
  tools::LevelSetRayIntersector<GridT> ray_intersector(*level_set);

  // generate a circular range of rays with origin at 0,0,0
  // all rays point along the x-y-Plane. z is kept at 0 for now
  std::vector<RayT> rays = generate_rays<RayT>(n_rays);
  std::vector<Vec3T> reference_solutions = calculate_reference_solution<Vec3T>(n_rays);

  // Run Benchmark
  std::vector<Vec3T> calculated(n_rays, Vec3T(0, 0, 0)); // results
  Timer timer;
  timer.reset();
  for (size_t i = 0; i < n_rays; i++)
  {
    ray_intersector.intersectsWS(rays[i], calculated[i]);
  }

  double time = timer.get();
  PLOG_INFO << "Finished in " << time << "s (" << (double)n_rays / (1000 * time) << " kRays/s)"
            << std::endl;

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

  // Verify Solution
  FP_Type voxel_size = options["voxel_size"].as<FP_Type>();
  FP_Type eps = voxel_size / 2;
  Vec3T vec_eps(eps, eps, eps);
  for (size_t i = 0; i < n_rays; i++)
  {
    assert(openvdb::math::isApproxEqual(calculated[i], reference_solutions[i], vec_eps));
  }
}

void Benchmarker::run_nanoVDB(size_t n_rays)
{
  using GridT = nanovdb::FloatGrid;
  using CoordT = nanovdb::Coord;
  using FP_type = float;
  using Vec3T = nanovdb::Vec3<FP_type>;
  using RayT = nanovdb::Ray<FP_type>;

  nanovdb::GridHandle<BufferT> handle;
  handle = nanovdb::createLevelSetSphere<FP_type, FP_type, BufferT>(
      sphere_radius_outer, {0, 0, 0}, voxel_size, level_set_half_width);

  auto *h_grid = handle.grid<FP_type>();

  std::vector<RayT> rays = generate_rays<RayT>(n_rays);
  std::vector<Vec3T> reference_solutions = calculate_reference_solution<Vec3T>(n_rays);

  // Run Benchmark
  std::vector<Vec3T> calculated(n_rays, Vec3T(0, 0, 0)); // results
  auto acc = h_grid->tree().getAccessor();
  CoordT ijk;
  float t0; // must be float
  float v;
  Timer timer;

  timer.reset();
  for (size_t i = 0; i < n_rays; i++)
  {
    nanovdb::ZeroCrossing(rays[i], acc, ijk, v, t0);
    calculated[i] = Vec3T(ijk.x() * voxel_size, ijk.y() * voxel_size, ijk.z() * voxel_size);
  }
  
  return;
}

// TODO: rename to RayT
template <class T> std::vector<T> Benchmarker::generate_rays(size_t n_rays)
{
  using Vec3T = typename T::Vec3T;

  std::vector<FP_Type> alpha_vals = linspace<FP_Type>(0.0, 2.0 * pi, n_rays);
  std::vector<T> rays(n_rays);
  Vec3T eye(0, 0, 0);

  for (size_t i = 0; i < n_rays; i++)
  {
    // Generate Rays
    Vec3T direction(std::cos(alpha_vals[i]), // x = Cos(α)
                    std::sin(alpha_vals[i]), // y = Sin(α)
                    0                        // z = 0
    );
    direction.normalize();
    T ray(eye, direction);
    rays[i] = ray;
  }

  return rays;
}

template <typename T> std::vector<T> Benchmarker::calculate_reference_solution(size_t n_rays)
{
  std::vector<T> reference_solution(n_rays);
  std::vector<FP_Type> alpha_vals = linspace<FP_Type>(0.0, 2.0 * pi, n_rays);

  for (size_t i = 0; i < n_rays; i++)
  {
    Vec3T solution(sphere_radius_outer * std::cos(alpha_vals[i]),
                   sphere_radius_outer * std::sin(alpha_vals[i]), 0);
    reference_solution[i] = solution;
  }
  return reference_solution;
}
