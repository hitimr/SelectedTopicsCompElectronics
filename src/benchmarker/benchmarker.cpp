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
  size_t sqrt_n_rays = 0;
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
      Vec3T eye(direction * (sphere_radius_inner + ray_offset));
      grid.indexToWorld(eye);

      // Final Ray
      rays[i] = RayT(grid.worldToIndex(eye), direction);
    }
    break;

  case DIM3:
    assert(floor(sqrt(n_rays)) == sqrt(n_rays));
    alpha_vals = linspace<FP_Type>(0.0, 2.0 * M_PI, sqrt_n_rays);

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
        Vec3T eye(direction * (sphere_radius_inner + ray_offset));
        grid.indexToWorld(eye);

        // Final Ray
        rays[i] = RayT(grid.worldToIndex(eye), direction);
      }
    }
    break;

  default:
    throw RuntimeError("Only 2D or 3D available");
  }

  return rays;
}

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
// TODO: Docstring
template <typename Vec3T>
std::vector<Vec3T> Benchmarker::calculate_reference_solution(size_t n_rays, FP_Type radius)
{
  std::vector<Vec3T> reference_solution(n_rays);
  std::vector<FP_Type> alpha_vals = linspace<FP_Type>(0.0, 2.0 * M_PI, n_rays);

  for (size_t i = 0; i < n_rays; i++)
  {
    Vec3T solution(radius * std::cos(alpha_vals[i]), radius * std::sin(alpha_vals[i]), 0);
    reference_solution[i] = solution;
  }
  return reference_solution;
}

Benchmarker::Benchmarker(const OptionsT &options) : options(options)
{
  // Sphere Parameters
  sphere_radius_inner = (FP_Type)options["r0"].as<double>();
  sphere_radius_outer = (FP_Type)options["r1"].as<double>();
  ray_dim = (int)options["ray_dim"].as<int>();

  // Grid Settings
  voxel_size = (FP_Type)options["voxel_size"].as<double>();
  level_set_half_width = (FP_Type)options["half_width"].as<double>();
  eps = voxel_size * math::Sqrt(3.);
  ray_offset = (FP_Type)options["ray_offset"].as<double>();

  // GPU
  grid_size = (size_t)options["grid_size"].as<int>();
  block_size = (size_t)options["block_size"].as<int>();

  // Benchmark
  n_bench = options["nbench"].as<int>();

  // Sanity checks
  assert(0 <= sphere_radius_inner);
  assert(sphere_radius_inner < sphere_radius_outer);
  assert(0 < voxel_size);
  assert(0 < level_set_half_width);
  assert(0 < grid_size);
  assert(0 < block_size);
  assert(0 < n_bench);
  assert((ray_dim == DIM2) || (ray_dim == DIM3));
}

double Benchmarker::run_openVDB(OVBD_GridT &level_set, size_t n_rays)
{
  assert(n_rays > 0);
  PLOG_INFO << "Running OpenVDB benchmark for " << n_rays << " Rays" << std::endl;

  // Ray Intersector: Triple nested types. nice...
  tools::LevelSetRayIntersector<OVBD_GridT, tools::LinearSearchImpl<OVBD_GridT, 0, FP_Type>,
                                OVBD_GridT::TreeType::RootNodeType::ChildNodeType::LEVEL, OVBD_RayT>
      ray_intersector(level_set);

  std::vector<OVBD_RayT> rays = generate_rays<OVBD_GridT, OVBD_RayT>(level_set, n_rays);
  std::vector<OVBD_Vec3T> reference_intersections =
      calculate_reference_solution<OVBD_Vec3T>(n_rays, sphere_radius_outer);

  // Run Benchmark
  std::vector<OVBD_Vec3T> iResults(n_rays);

  Timer timer;
  timer.reset();
  for (size_t i = 0; i < n_rays; i++)
  {
    ray_intersector.intersectsIS(rays[i], iResults[i]);
  }
  double time = timer.get();
  PLOG_INFO << "OpenVDB Finished in " << time << "s (" << (double)n_rays / (1000 * time)
            << " kRays/s)" << std::endl;

  auto wResults = indexToWorld(level_set, iResults);
  verify_results(wResults, reference_intersections);

  return time;
}

// convenience function
Benchmarker::OVBD_GridT Benchmarker::generate_sphere(FP_Type radius)
{
  return *tools::createLevelSetSphere<OVBD_GridT>(
      radius,                         // radius of the sphere in world units
      {center_x, center_y, center_z}, // center of the sphere in world units
      voxel_size,                     // voxel size in world units
      level_set_half_width            // half the width of the narrow band, in voxel units
  );
}

Benchmarker::OVBD_GridT Benchmarker::generate_doubleSphere()
{
  OVBD_GridT grid = generate_sphere(sphere_radius_outer);
  OVBD_GridT sphere_0 = generate_sphere(sphere_radius_inner);

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

  save_grid("nano_grid.vdb", grid);

  return grid;
}

void Benchmarker::save_grid(std::string filename, OVBD_GridT &grid)
{
  // generate absolute file path
  std::string outfile(global_settings["out_dir"]);
  outfile += filename;

  // save to file (see:
  // https://academysoftwarefoundation.github.io/openvdb/codeExamples.html#sHelloWorld)
  // openvdb::io::File vdb_file(outfile);
  // std::vector<OVBD_GridT&> grids;
  // grids.push_back(grid);
  // vdb_file.write(grids);
  // vdb_file.close();
}

void Benchmarker::run()
{
  // set number of rays for the benchmark
  ray_vals =
      logspace(options["p_rays_start"].as<int>(), options["p_rays_end"].as<int>(), BASE2, n_bench);

  auto level_set_ovbd = generate_doubleSphere();

  // Convert to nanoVDBV
  // Note: it is possible to create Level sets directly in NanoVDB as well bus this is slower
  auto level_set_cpu = nanovdb::openToNanoVDB<nanovdb::HostBuffer>(level_set_ovbd);
  auto level_set_gpu = nanovdb::openToNanoVDB<nanovdb::CudaDeviceBuffer>(level_set_ovbd);

  // convert grid back to open_vdb and save it.
  // mainly for debugging and checking if the grid is correctly converted
  // save_grid("nano_grid.vdb", nanovdb::nanoToOpenVDB(level_set_cpu));

  // output File
  std::string outFile_name(global_settings["proj_root"]);
  outFile_name += global_settings["outfile_timings"];
  std::ofstream outFile(outFile_name);
  assert(outFile.is_open());

  // .csv Header
  outFile << "n_rays;time_ovdb;time_nvdb_cpu;time_nvdb_gpu" << std::endl;

  // Run Benchmarks
  for (size_t n_rays : ray_vals)
  {
    if (ray_dim == DIM3)
    {
      // change n_rays to next perfect square
      size_t sqrt_n_rays = (size_t)sqrt((FP_Type)n_rays);
      n_rays = sqrt_n_rays * sqrt_n_rays;
    }

    assert(n_rays > 0);

    outFile << n_rays << ";";
    outFile << run_openVDB(level_set_ovbd, n_rays) << ";";
    outFile << run_nanoVDB_CPU(level_set_cpu, n_rays) << ";";
    outFile << run_nanoVDB_GPU(level_set_gpu, n_rays) << std::endl;

    PLOG_INFO << "Done" << std::endl << std::endl;
  }
  outFile.close();
}

double Benchmarker::run_nanoVDB_CPU(nanovdb::GridHandle<nanovdb::HostBuffer> &level_set,
                                    size_t n_rays)
{

  nanovdb::FloatGrid *h_grid = level_set.grid<FP_Type>();

  std::vector<NVDB_RayT> rays = generate_rays<NVDB_GridT, NVDB_RayT>(*h_grid, n_rays);
  std::vector<OVBD_Vec3T> reference_intersections =
      calculate_reference_solution<OVBD_Vec3T>(n_rays, sphere_radius_outer);

  auto acc = h_grid->tree().getAccessor();
  std::vector<NVBD_CoordT> iResults(n_rays);
  FP_Type t0;
  FP_Type v;
  Timer timer;

  // Run Benchmark
  timer.reset();
  for (size_t i = 0; i < n_rays; i++)
  {
    nanovdb::ZeroCrossing(rays[i], acc, iResults[i], v, t0);
  }
  double time = timer.get();

  PLOG_INFO << "NanoVDB on CPU Finished in " << time << "s (" << (double)n_rays / (1000 * time)
            << " kRays/s)" << std::endl;

  auto wResults = indexToWorld(*h_grid, iResults);
  verify_results(wResults, reference_intersections);

  return time;
}

// verify results by comparing them to precomputed reference solutions
template <typename Vec3T>
bool Benchmarker::verify_results(const std::vector<Vec3T> &wResults,
                                 const std::vector<Vec3T> &wReference)
{
  assert(wResults.size() == wReference.size());

  bool err_flag = false;

  for (size_t i = 0; i < wResults.size(); i++)
  {
    if (!isClose_vec3(wResults[i], wReference[i]))
    {
      PLOG_ERROR << "Calculated value does not match at pos " << i << std::endl;
      PLOG_ERROR << "Received:\t(" << wResults[i][0] << "|" << wResults[i][1] << "|"
                 << wResults[i][2] << ")" << std::endl;

      PLOG_ERROR << "Should be:\t(" << wReference[i][0] << "|" << wReference[i][1] << "|"
                 << wReference[i][2] << ")" << std::endl;
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
