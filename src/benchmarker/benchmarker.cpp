#include "benchmarker.hpp"
#include "common.hpp"
#include "util/misc.hpp"
#include "util/timer.hpp"

// OpenVDB
#include <nanovdb/util/IO.h>
#include <nanovdb/util/NanoToOpenVDB.h>
#include <nanovdb/util/OpenToNanoVDB.h>
#include <nanovdb/util/Primitives.h>
#include <openvdb/Exceptions.h>
#include <openvdb/math/Transform.h>
#include <openvdb/tools/Composite.h>
#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/tools/RayIntersector.h>
#include <openvdb/tools/ValueTransformer.h>

#include <cmath>
#include <fstream>
#include <omp.h>
#include <vector>

using namespace openvdb;

/**
 * @brief Generate Rays for the benchmark.
 * Currently all Rays start at (0,0,0) and are equally spread in a circular field
 *
 * @tparam NVDB_RayT either OpenVDB Rays or NanoVDB Rays
 * @param n_rays number of rays that should be generated
 * @return std::vector<NVDB_RayT>
 * TODO: Update
 */
template <class GridT, class RayT>
std::vector<RayT> Benchmarker::generate_rays(GridT &grid, size_t n_rays)
{
  using Vec3T = typename RayT::Vec3T;

  std::vector<RayT> rays(n_rays);
  std::vector<FP_Type> alpha_vals;
  size_t sqrt_n_rays = sqrt(n_rays);
  FP_Type theta, phi;

  switch (ray_dim)
  {
  case DIM2:
    // Generate rays in circular arrangement on x-y-Plane
    alpha_vals = linspace<FP_Type>(0.0, 2.0 * M_PI, n_rays);
    for (size_t i = 0; i < n_rays; i++)
    {
      // normalized direction
      Vec3T direction(std::cos(alpha_vals[i]), // x = Cos(α)
                      std::sin(alpha_vals[i]), // y = Sin(α)
                      0                        // z = 0
      );
      direction.normalize();

      // Eye
      Vec3T eye(direction * (options["r0"].as<double>() + ray_offset));
      grid.indexToWorld(eye);

      // Final Ray
      rays[i] = RayT(grid.worldToIndex(eye), direction);
    }
    break;

  case DIM3:
    assert(floor(sqrt(n_rays)) == sqrt(n_rays));
    alpha_vals = linspace<FP_Type>(0.0, 2.0 * M_PI, sqrt_n_rays);

#pragma omp parallel for
    for (size_t i = 0; i < sqrt_n_rays; i++)
    {
      for (size_t j = 0; j < sqrt_n_rays; j++)
      {
        theta = alpha_vals[i];
        phi = alpha_vals[j];

        // normalized direction
        Vec3T direction(std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi),
                        std::cos(theta));
        direction.normalize();

        // Eye
        Vec3T eye(direction * (options["r0"].as<double>() + ray_offset));
        grid.indexToWorld(eye);

        // Final Ray
        rays[i * sqrt_n_rays + j] = RayT(grid.worldToIndex(eye), direction);
      }
    }
    break;

  default:
    throw RuntimeError("Only 2D or 3D available");
  }

  return rays;
}

/**
 * @brief Convenience Function to convert between IS and WS.
 *
 * @tparam GridT
 * @tparam Vec3T
 * @param grid
 * @param iPoints
 * @return std::vector<Benchmarker::OVBD_Vec3T>
 */
template <class GridT, class Vec3T>
std::vector<Benchmarker::OVBD_Vec3T> Benchmarker::indexToWorld(GridT &grid,
                                                               std::vector<Vec3T> &iPoints)
{
  std::vector<OVBD_Vec3T> wPoints(iPoints.size());
  for (size_t i = 0; i < iPoints.size(); i++)
  {
    wPoints[i] = grid.indexToWorld(
        OVBD_Vec3T((FP_Type)iPoints[i][0], (FP_Type)iPoints[i][1], (FP_Type)iPoints[i][2]));
  }
  return wPoints;
}

// calculate ray intersections analytically
/**
 * @brief Calculate a reference solution using analytical methods
 *  Depending on the benchmark being in 2D or 3D different solutions are derived
 *
 * The distribution is the same as in generate_rays() but this time the only points on the outer
 * sphere are generated
 *
 * @tparam Vec3T
 * @param n_rays
 * @param radius
 * @return std::vector<Vec3T>
 */
template <typename Vec3T>
std::vector<Vec3T> Benchmarker::calculate_reference_solution(size_t n_rays, FP_Type radius)
{
  std::vector<Vec3T> reference_solution(n_rays);
  std::vector<FP_Type> alpha_vals = linspace<FP_Type>(0.0, 2.0 * M_PI, n_rays);
  size_t sqrt_n_rays = sqrt(n_rays);
  FP_Type theta, phi;

  switch (ray_dim)
  {
  case DIM2:
    for (size_t i = 0; i < n_rays; i++)
    {
      Vec3T solution(radius * std::cos(alpha_vals[i]), radius * std::sin(alpha_vals[i]), 0);
      reference_solution[i] = solution;
    }
    break;

  case DIM3:
    assert(floor(sqrt(n_rays)) == sqrt(n_rays));
    alpha_vals = linspace<FP_Type>(0.0, 2.0 * M_PI, sqrt_n_rays);

#pragma omp parallel for
    for (size_t i = 0; i < sqrt_n_rays; i++)
    {
      for (size_t j = 0; j < sqrt_n_rays; j++)
      {
        theta = alpha_vals[i];
        phi = alpha_vals[j];

        Vec3T solution(radius * std::sin(theta) * std::cos(phi),
                       radius * std::sin(theta) * std::sin(phi), radius * std::cos(theta));

        reference_solution[i * sqrt_n_rays + j] = solution;
      }
    }
    break;

  default:
    throw RuntimeError("Only 2D or 3D available");
  }
  return reference_solution;
}

Benchmarker::Benchmarker(const OptionsT &options) : options(options)
{
  ray_dim = (int)options["ray_dim"].as<int>();

  // Grid Settings
  voxel_size = (FP_Type)options["voxel_size"].as<double>();
  level_set_half_width = (FP_Type)options["half_width"].as<double>();
  eps = voxel_size * math::Sqrt(3.);
  ray_offset = (FP_Type)options["ray_offset"].as<double>();

  // GPU
  grid_size = (size_t)options["grid_size"].as<int>();
  block_size = (size_t)options["block_size"].as<int>();
}
// TODO: rename level_set to grid everywhere
void Benchmarker::run_openVDB(OVBD_GridT &grid,  std::vector<OVBD_Vec3T> const & reference_solution, size_t n_rays)
{
  assert(n_rays > 0);

  // Ray Intersector: Triple nested types. nice...
  using IntersectorT =
      tools::LevelSetRayIntersector<OVBD_GridT, tools::LinearSearchImpl<OVBD_GridT, 0, FP_Type>,
                                    OVBD_GridT::TreeType::RootNodeType::ChildNodeType::LEVEL,
                                    OVBD_RayT>;
  IntersectorT ray_intersector(grid);

  std::vector<OVBD_RayT> rays = generate_rays<OVBD_GridT, OVBD_RayT>(grid, n_rays);

  // Run Benchmark
  std::vector<OVBD_Vec3T> iResults(n_rays);

  Timer timer;
  timer.reset();
#pragma omp parallel for firstprivate(ray_intersector)
  for (size_t i = 0; i < n_rays; i++)
  {
    ray_intersector.intersectsIS(rays[i], iResults[i]);
  }
  double time = timer.get();
  PLOG_INFO << "OpenVDB Finished in " << time << "s (" << (double)n_rays / (1000 * time)
            << " kRays/s)" << std::endl;

  auto wResults = indexToWorld(grid, iResults);
  analyze_results(wResults, reference_solution);

  write_results(result_file, "OpenVDB", n_rays, time, 1, omp_get_num_threads(),
                options["cpu_price"].as<double>(), options["cpu_power"].as<double>());
}

// convenience function
Benchmarker::OVBD_GridT Benchmarker::generate_sphere(FP_Type radius)
{
  return *tools::createLevelSetSphere<OVBD_GridT>(
      radius,                                      // radius of the sphere in world units
      {center_x, center_y, center_z},              // center of the sphere in world units
      (FP_Type)options["voxel_size"].as<double>(), // voxel size in world units
      level_set_half_width // half the width of the narrow band, in voxel units
  );
}

Benchmarker::OVBD_GridT Benchmarker::generate_doubleSphere()
{
  OVBD_GridT grid = generate_sphere((FP_Type)options["r1"].as<double>());
  OVBD_GridT sphere_0 = generate_sphere((FP_Type)options["r0"].as<double>());

  // use geometric difference to generate a sphere with an empty core
  openvdb::tools::csgDifference(grid, sphere_0);

  // invert entire grid to generate 2 concentric spheres
  // taken from https://academysoftwarefoundation.github.io/openvdb/codeExamples.html
  // -> Value transformation
  struct Local
  {
    static inline void op(const openvdb::FloatGrid::ValueAllIter &iter)
    {
      iter.setValue(*iter * -1.0);
    }
  };
  openvdb::tools::foreach (grid.beginValueAll(), Local::op);

  // Meta data
  grid.setGridClass(openvdb::GRID_LEVEL_SET);
  grid.setName("LevelSetSphere");

  std::string file_name = misc::abs_path(global_settings["paths"]["grid_output_file"]);
  save_grid(file_name, grid);

  return grid;
}

void Benchmarker::save_grid(std::string filename, OVBD_GridT &grid)
{
  // save to file (see:
  // https://academysoftwarefoundation.github.io/openvdb/codeExamples.html#sHelloWorld)
  openvdb::io::File file(filename);
  openvdb::GridPtrVec grids;
  grids.push_back(std::make_shared<OVBD_GridT>(grid));
  file.write(grids);
  file.close();
}

void Benchmarker::run()
{
  // set number of rays for the benchmark
  PLOG_INFO << "Setting up Benchmark run..." << std::endl;
  ray_vals = logspace(options["p_rays_start"].as<int>(), options["p_rays_end"].as<int>(), BASE2,
                      options["n_bench"].as<int>());

  PLOG_INFO << "Generating Level Set using OpenVDB" << std::endl;
  auto ovdb_grid = generate_doubleSphere();

  // Convert to nanoVDBV
  // Note: it is possible to create Level sets directly in NanoVDB as well bus it's significantly
  // slower
  PLOG_INFO << "Converting level set to NanoVDB Datastructure" << std::endl;
  auto nvdb_grid_cpu = nanovdb::openToNanoVDB<nanovdb::HostBuffer>(ovdb_grid);
  auto nvdb_grid_gpu = nanovdb::openToNanoVDB<nanovdb::CudaDeviceBuffer>(ovdb_grid);

  // output Files
  std::string out_file_name = options.count("outfile")
                                  ? options["outfile"].as<std::vector<std::string>>()[0]
                                  : misc::abs_path(global_settings["paths"]["outfile_timings"]);
                          
  result_file.open(out_file_name);
  init_result_file(result_file);
  PLOG_INFO << "Writing results to " << out_file_name << std::endl;


  // Run Benchmarks
  int counter = 0;
  for (size_t n_rays : ray_vals)
  {
    counter ++;
    if (ray_dim == DIM3)
    {
      // change n_rays to next perfect square
      size_t sqrt_n_rays = (size_t)sqrt((FP_Type)n_rays);
      n_rays = sqrt_n_rays * sqrt_n_rays;
    }
    assert(n_rays > 0);
    PLOG_INFO << "Running benchmark for " << n_rays << " Rays (" << counter << "/" <<  ray_vals.size() << ")" << std::endl;

    std::vector<OVBD_Vec3T> reference_solution =
      calculate_reference_solution<OVBD_Vec3T>(n_rays, options["r1"].as<double>());


    run_openVDB(ovdb_grid, reference_solution, n_rays);
    run_nanoVDB_CPU(nvdb_grid_cpu, reference_solution, n_rays);
    run_nanoVDB_GPU(nvdb_grid_gpu, reference_solution, n_rays);

    PLOG_INFO << "Done" << std::endl << std::endl;
  }

  result_file.close();
}

void Benchmarker::run_nanoVDB_CPU(nanovdb::GridHandle<nanovdb::HostBuffer> &level_set, std::vector<OVBD_Vec3T> const & reference_solution,
                                  size_t n_rays)
{

  nanovdb::FloatGrid *h_grid = level_set.grid<FP_Type>();

  std::vector<NVDB_RayT> rays = generate_rays<NVDB_GridT, NVDB_RayT>(*h_grid, n_rays);

  auto acc = h_grid->tree().getAccessor();
  std::vector<NVBD_CoordT> iResults(n_rays);
  FP_Type t0 = 0.;
  FP_Type v = 0.;
  Timer timer;

  // Run Benchmark
  timer.reset();
#pragma omp parallel for firstprivate(acc, v, t0)
  for (size_t i = 0; i < n_rays; i++)
  {
    nanovdb::ZeroCrossing(rays[i], acc, iResults[i], v, t0);
  }
  double time = timer.get();

  PLOG_INFO << "NanoVDB on CPU Finished in " << time << "s (" << (double)n_rays / (1000 * time)
            << " kRays/s)" << std::endl;

  auto wResults = indexToWorld(*h_grid, iResults);
  analyze_results(wResults, reference_solution);
  write_results(result_file, "NanoVDB_CPU", n_rays, time, 1, omp_get_num_threads(),
                options["cpu_price"].as<double>(), options["cpu_power"].as<double>());
}

// verify results by comparing them to precomputed reference solutions
bool Benchmarker::analyze_results(const std::vector<OVBD_Vec3T> &wResults,
                                  const std::vector<OVBD_Vec3T> &wReference)
{
  assert(wResults.size() == wReference.size());

  bool err_flag = false;
#pragma omp parallel for shared(err_flag)
  for (size_t i = 0; i < wResults.size(); i++)
  {
    if (!isClose_vec3(wResults[i], wReference[i]))
    {
      #pragma omp critical
      {
        PLOG_ERROR << "Calculated value does not match at pos " << i << std::endl;
        PLOG_ERROR << "Received:\t(" << wResults[i][0] << "|" << wResults[i][1] << "|"
                  << wResults[i][2] << ")" << std::endl;

        PLOG_ERROR << "Should be:\t(" << wReference[i][0] << "|" << wReference[i][1] << "|"
                  << wReference[i][2] << ")" << std::endl;
        err_flag = true;
      }
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
