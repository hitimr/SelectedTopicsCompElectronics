#include "benchmarker.hpp"
#include "common.hpp"
#include "util/misc.hpp"
#include "util/timer.hpp"

// OpenVDB
#include <openvdb/Exceptions.h>
#include <openvdb/math/Transform.h>
#include <openvdb/tools/RayIntersector.h>

#include <openvdb/tools/LevelSetSphere.h>

#include <cmath>
#include <vector>

using namespace openvdb;

// TODO: reintroduce result verification for all variants

/**
 * @brief Generate Rays for the benchmark.
 * Currently all Rays start at (0,0,0) and are equally spread in a circular field
 *
 * @tparam NVDB_RayT either OpenVDB Rays or NanoVDB Rays
 * @param n_rays number of rays that should be generated
 * @return std::vector<NVDB_RayT>
 */
template <class RayT> std::vector<RayT> Benchmarker::generate_rays(size_t n_rays)
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

// calculate ray intersections analytically
// TODO: Docstring
template <typename Vec3T>
std::vector<Vec3T> Benchmarker::calculate_reference_solution(size_t n_rays,
                                                             FP_Type sphere_radius_outer)
{
  std::vector<Vec3T> reference_solution(n_rays);
  std::vector<FP_Type> alpha_vals = linspace<FP_Type>(0.0, 2.0 * M_PI, n_rays);

  for (size_t i = 0; i < n_rays; i++)
  {
    Vec3T solution(sphere_radius_outer * std::cos(alpha_vals[i]),
                   sphere_radius_outer * std::sin(alpha_vals[i]), 0);
    reference_solution[i] = solution;
  }
  return reference_solution;
}

Benchmarker::Benchmarker(const OptionsT &options) : options(options)
{

  // Sphere Parameters
  sphere_radius_outer = (FP_Type)options["radius"].as<double>();
  voxel_size = (FP_Type)options["voxel_size"].as<double>();
  level_set_half_width = 2.0;
  eps = voxel_size * math::Sqrt(3.);
}

void Benchmarker::run_openVDB(const OVBD_GridT::Ptr &level_set, size_t n_rays)
{
  assert(n_rays > 0);
  PLOG_INFO << "Running OpenVDB benchmark for " << n_rays << " Rays" << std::endl;

  // Ray Intersector: Triple nested types. nice...
  tools::LevelSetRayIntersector<OVBD_GridT, tools::LinearSearchImpl<OVBD_GridT, 0, FP_Type>,
                                OVBD_GridT::TreeType::RootNodeType::ChildNodeType::LEVEL, OVBD_RayT>
      ray_intersector(*level_set);

  std::vector<OVBD_RayT> rays = generate_rays<OVBD_RayT>(n_rays);
  std::vector<OVBD_Vec3T> reference_solutions =
      calculate_reference_solution<OVBD_Vec3T>(n_rays, sphere_radius_outer);

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

  // required because OpenVDB and NanoVDB require different Classes for Vec3
  FP_Type center_x = 0;
  FP_Type center_y = 0;
  FP_Type center_z = 0;

  // OpenVDB Level Set
  PLOG_INFO << "Generating Level set for OpenVDB" << std::endl;
  OVBD_GridT::Ptr level_set_ovbd = tools::createLevelSetSphere<OVBD_GridT>(
      sphere_radius_outer,            // radius of the sphere in world units
      {center_x, center_y, center_z}, // center of the sphere in world units
      voxel_size,                     // voxel size in world units
      level_set_half_width            // half the width of the narrow band, in voxel units
  );

  // NanoVDB CPU Level Set
  PLOG_INFO << "Generating Level set for NanoVDB on CPU" << std::endl;
  auto level_set_cpu = nanovdb::createLevelSetSphere<FP_Type, FP_Type, nanovdb::HostBuffer>(
      sphere_radius_outer, {center_x, center_y, center_z}, voxel_size, level_set_half_width);

  // NanoVDB GPU Level Set
  PLOG_INFO << "Generating Level set for NanoVDB on GPU" << std::endl;
  auto level_set_gpu = nanovdb::createLevelSetSphere<FP_Type, FP_Type, nanovdb::CudaDeviceBuffer>(
      sphere_radius_outer, {center_x, center_y, 0}, voxel_size, level_set_half_width,
      nanovdb::Vec3R(0));

  for (size_t n_rays : ray_vals)
  {
    run_openVDB(level_set_ovbd, n_rays);
    run_nanoVDB_CPU(level_set_cpu, n_rays);
    run_nanoVDB_GPU(level_set_gpu, n_rays);
    PLOG_INFO << "Done" << std::endl << std::endl;
  }
}

// TODO: move to separate File
void Benchmarker::run_nanoVDB_CPU(nanovdb::GridHandle<nanovdb::HostBuffer> &level_set,
                                  size_t n_rays)
{
  using NVDB_RayT = nanovdb::Ray<FP_Type>;

  auto *h_grid = level_set.grid<FP_Type>();

  std::vector<NVDB_RayT> rays = generate_rays<NVDB_RayT>(n_rays);
  std::vector<NVBD_Vec3T> reference_intersections =
      calculate_reference_solution<NVBD_Vec3T>(n_rays, sphere_radius_outer);

  auto acc = h_grid->tree().getAccessor();
  std::vector<NVBD_CoordT> result_coords(n_rays);
  FP_Type t0;
  FP_Type v;
  Timer timer;

  // Run Benchmark
  timer.reset();
  for (size_t i = 0; i < n_rays; i++)
  {
    nanovdb::ZeroCrossing(rays[i], acc, result_coords[i], v, t0);
  }
  double time = timer.get();

  PLOG_INFO << "NanoVDB on CPU Finished in " << time << "s (" << (double)n_rays / (1000 * time)
            << " kRays/s)" << std::endl;

  std::vector<NVBD_Vec3T> result_intersections(n_rays);
  for (size_t i = 0; i < n_rays; i++)
  {
    result_intersections[i] = h_grid->indexToWorldF<NVBD_Vec3T>(result_coords[i].asVec3s());
  }

  verify_results<NVBD_Vec3T>(result_intersections, reference_intersections);
}

// verify results by comparing them to precomputed reference solutions
template <typename Vec3T>
bool Benchmarker::verify_results(const std::vector<Vec3T> &result_intersections,
                                 const std::vector<Vec3T> &reference_intersections)
{
  assert(result_intersections.size() == reference_intersections.size());
  bool err_flag = false;

  for (size_t i = 0; i < result_intersections.size(); i++)
  {
    if(!isClose_vec3(result_intersections[i], reference_intersections[i]))
    {
      PLOG_ERROR << "Calculated time does not match at pos " << i << std::endl;
      // TODO: Vec3 printf/cout
      // PLOG_ERROR << "Received:\t" << result_intersections[i] << std::endl;
      // PLOG_ERROR << "Should be:\t" << reference_intersections[i] << std::endl;
      err_flag = true;
    }
  }
  return err_flag;
}

template <typename Vec3T> bool Benchmarker::isClose_vec3(const Vec3T &a, const Vec3T &b)
{
  if ((std::abs(a[0] - b[0]) > eps) || (std::abs(a[1] - b[1]) > eps) ||
      (std::abs(a[2] - b[2]) > eps))
  {
    return false;
  }
  return true;
}
