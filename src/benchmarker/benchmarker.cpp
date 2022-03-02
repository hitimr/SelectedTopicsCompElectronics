#include "benchmarker.hpp"
#include "common.hpp"
#include "nanoVDB_GPU.hpp"
#include "util/misc.hpp"
#include "util/timer.hpp"

// OpenVDB
#include <openvdb/Exceptions.h>
#include <openvdb/math/Transform.h>

#include <openvdb/tools/LevelSetSphere.h>

#include <cmath>

using namespace openvdb;




/**
 * @brief Generate Rays for the benchmark.
 * Currently all Rays start at (0,0,0) and are equally spread in a circular field
 *
 * @tparam NVDB_RayT either OpenVDB Rays or NanoVDB Rays
 * @param n_rays number of rays that should be generated
 * @return std::vector<NVDB_RayT>
 */
template <class RayT> std::vector<RayT>generate_rays(size_t n_rays)
{
  using Vec3T = typename RayT::Vec3T;
  using RealT = typename Vec3T::ValueType;

  std::vector<RealT> alpha_vals = linspace<RealT>(0.0, 2.0 * M_PI, n_rays);
  std::vector<RayT> rays(n_rays);
  Vec3T eye(0, 0, 0);

  for (size_t i = 0; i < n_rays; i++)
  {
    // Generate Rays
    Vec3T direction(std::cos(alpha_vals[i]), // x = Cos(α)
                    std::sin(alpha_vals[i]), // y = Sin(α)
                    0                        // z = 0
    );
    direction.normalize();
    RayT ray(eye, direction);
    rays[i] = ray;
  }

  return rays;
}


Benchmarker::Benchmarker(const OptionsT &options) : options(options)
{

  // Sphere Parameters
  sphere_radius_outer = (FP_Type)options["radius"].as<double>();
  voxel_size = (FP_Type)options["voxel_size"].as<double>();
  level_set_half_width = 2.0;
}

void Benchmarker::run_openVDB(const OVBD_GridT::Ptr &level_set, size_t n_rays)
{
  assert(n_rays > 0);
  PLOG_INFO << "\nRunning OpenVDB benchmark for " << n_rays << " Rays" << std::endl;

  // Ray Intersector: Triple nested types. nice...
  tools::LevelSetRayIntersector<OVBD_GridT, tools::LinearSearchImpl<OVBD_GridT, 0, FP_Type>,
                                OVBD_GridT::TreeType::RootNodeType::ChildNodeType::LEVEL, OVBD_RayT>
      ray_intersector(*level_set);

  std::vector<OVBD_RayT> rays = generate_rays<OVBD_RayT>(n_rays);
  std::vector<OVBD_Vec3T> reference_solutions = calculate_reference_solution<OVBD_Vec3T>(n_rays);

  // Run Benchmark
  // TODO: make multiple repeats with custom wrapper function
  std::vector<FP_Type> times(n_rays);
  std::vector<bool> intersections(n_rays);

  Timer timer;
  timer.reset();
  for (size_t i = 0; i < n_rays; i++)
  {
    ray_intersector.intersectsIS(rays[i].worldToIndex(*level_set));
  }

  double time = timer.get();
  PLOG_INFO << "OpenVDB Finished in " << time << "s (" << (double)n_rays / (1000 * time)
            << " kRays/s)" << std::endl;

  // verify_results(times, intersections);
}

void Benchmarker::run()
{
  // set number of rays for the benchmark
  ray_vals = logspace(options["nrays_min"].as<int>(), options["nrays_max"].as<int>(), BASE2,
                      options["nbench"].as<int>());

  OVBD_Vec3T center(0, 0, 0);
  OVBD_GridT::Ptr level_set = tools::createLevelSetSphere<OVBD_GridT>(
      sphere_radius_outer, // radius of the sphere in world units
      center,              // center of the sphere in world units
      voxel_size,          // voxel size in world units
      level_set_half_width // half the width of the narrow band, in voxel units
  );

  // NanoVDB Level Set
  auto handle = nanovdb::createLevelSetSphere<FP_Type, FP_Type, nanovdb::HostBuffer>(
      sphere_radius_outer, {0, 0, 0}, voxel_size,
      level_set_half_width); // TODO: replace hardcoded center

  for (size_t n_rays : ray_vals)
  {
    run_openVDB(level_set, n_rays);
    run_nanoVDB_CPU(handle, n_rays);
    run_nanoVDB_GPU(handle, n_rays); // TODO: rename to levelset or something
  }
}

void Benchmarker::run_nanoVDB_CPU(nanovdb::GridHandle<nanovdb::HostBuffer> &handle, size_t n_rays)
{
  using NVDB_RayT = nanovdb::Ray<FP_Type>;

  auto *h_grid = handle.grid<FP_Type>();

  std::vector<NVDB_RayT> rays = generate_rays<NVDB_RayT>(n_rays);
  std::vector<NVBD_Vec3T> reference_solutions = calculate_reference_solution<NVBD_Vec3T>(n_rays);

  // Run Benchmark
  std::vector<NVBD_Vec3T> calculated(n_rays, NVBD_Vec3T(0, 0, 0)); // results
  auto acc = h_grid->tree().getAccessor();
  NVBD_CoordT ijk;
  float t0;
  float v;
  Timer timer;

  timer.reset();
  for (size_t i = 0; i < n_rays; i++)
  {
    nanovdb::ZeroCrossing(rays[i], acc, ijk, v, t0);

    // nanovdb only supports raytracing/ intersection in index space
    // we need to transform results back to word coordinates
  }

  double time = timer.get();
  PLOG_INFO << "NanoVDB Finished in " << time << "s (" << (double)n_rays / (1000 * time)
            << " kRays/s)" << std::endl;

  // int err_pos;
  // assert(verify_results<NVBD_Vec3T>(calculated, reference_solutions, err_pos));
}

// calculate ray intersections analytically
template <typename Vec3T>
std::vector<Vec3T> Benchmarker::calculate_reference_solution(size_t n_rays)
{
  std::vector<Vec3T> reference_solution(n_rays);
  std::vector<FP_Type> alpha_vals = linspace<FP_Type>(0.0, 2.0 * pi, n_rays);

  for (size_t i = 0; i < n_rays; i++)
  {
    Vec3T solution(sphere_radius_outer * std::cos(alpha_vals[i]),
                   sphere_radius_outer * std::sin(alpha_vals[i]), 0);
    reference_solution[i] = solution;
  }
  return reference_solution;
}

// verify results by comparing them to precomputed reference solutions
bool Benchmarker::verify_results(const std::vector<FP_Type> &calculated,
                                 const std::vector<bool> &intersections)
{
  FP_Type eps = voxel_size / 2;
  FP_Type corerct_solution = sphere_radius_outer;

  for (size_t i = 0; i < calculated.size(); i++)
  {
    if (std::abs(calculated[i] - corerct_solution) > eps)
    {
      PLOG_ERROR << "Calculated time does not match at pos " << i << std::endl;
      PLOG_ERROR << "Received:\t" << calculated[i] << std::endl;
      PLOG_ERROR << "Should be:\t" << corerct_solution << std::endl;
      return false;
    }

    if (intersections[i] == false)
    {
      PLOG_ERROR << "Calculated intersection does not match at pos " << i << std::endl;
      PLOG_ERROR << "Received:\t" << intersections[i] << std::endl;
      PLOG_ERROR << "Should be:\t" << true << std::endl;
    }
  }

  return true;
}
