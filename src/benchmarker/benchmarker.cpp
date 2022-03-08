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
std::vector<RayT> Benchmarker::generate_rays(GridT grid, size_t n_rays)
{
  using Vec3T = typename RayT::Vec3T;
  using RealT = typename Vec3T::ValueType;

  std::vector<RealT> alpha_vals = linspace<RealT>(0.0, 2.0 * M_PI, n_rays);
  std::vector<RayT> rays(n_rays);

  for (size_t i = 0; i < n_rays; i++)
  {
    // normalized direction
    Vec3T direction(std::cos(alpha_vals[i]), // x = Cos(α)
                    std::sin(alpha_vals[i]), // y = Sin(α)
                    0                        // z = 0
    );
    direction.normalize();

    // Eye
    Vec3T eye(direction * (sphere_radius_0 + 0.5));

    // Finaly Ray
    RayT wRay(eye, direction);
    rays[i] = wRay.worldToIndex(*grid);
  }

  return rays;
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
  sphere_radius_0 = (FP_Type)options["r0"].as<double>();
  sphere_radius_1 = (FP_Type)options["r1"].as<double>();

  voxel_size = (FP_Type)options["voxel_size"].as<double>();
  level_set_half_width = (FP_Type)options["half_width"].as<double>();
  eps = voxel_size * math::Sqrt(3.);

  assert(0 <= sphere_radius_0);
  assert(sphere_radius_0 < sphere_radius_1);
  assert(0 < voxel_size);
  assert(0 < level_set_half_width);
}

void Benchmarker::run_openVDB(const OVBD_GridT::Ptr &level_set, size_t n_rays)
{
  assert(n_rays > 0);
  PLOG_INFO << "Running OpenVDB benchmark for " << n_rays << " Rays" << std::endl;

  // Ray Intersector: Triple nested types. nice...
  tools::LevelSetRayIntersector<OVBD_GridT, tools::LinearSearchImpl<OVBD_GridT, 0, FP_Type>,
                                OVBD_GridT::TreeType::RootNodeType::ChildNodeType::LEVEL, OVBD_RayT>
      ray_intersector(*level_set);

  std::vector<OVBD_RayT> rays = generate_rays<OVBD_GridT::Ptr, OVBD_RayT>(level_set, n_rays);
  std::vector<OVBD_Vec3T> reference_intersections =
      calculate_reference_solution<OVBD_Vec3T>(n_rays, sphere_radius_1);

  // Run Benchmark
  std::vector<OVBD_Vec3T> result_intersections(n_rays);

  Timer timer;
  timer.reset();
  // TODO: make multiple repeats with custom wrapper function
  for (size_t i = 0; i < n_rays; i++)
  {
    ray_intersector.intersectsWS(rays[i], result_intersections[i]);
  }
  double time = timer.get();
  PLOG_INFO << "OpenVDB Finished in " << time << "s (" << (double)n_rays / (1000 * time)
            << " kRays/s)" << std::endl;

  verify_results(result_intersections, reference_intersections);
}

void Benchmarker::run_all()
{
  run_singleSphere(); // TODO: rename
}

// convenience funtion
Benchmarker::OVBD_GridT::Ptr Benchmarker::generate_sphere(FP_Type radius)
{
  return tools::createLevelSetSphere<OVBD_GridT>(
      radius,                         // radius of the sphere in world units
      {center_x, center_y, center_z}, // center of the sphere in world units
      voxel_size,                     // voxel size in world units
      level_set_half_width            // half the width of the narrow band, in voxel units
  );
}

Benchmarker::OVBD_GridT::Ptr Benchmarker::generate_doubleSphere()
{
  auto grid = generate_sphere(sphere_radius_1);
  auto sphere_0 = generate_sphere(sphere_radius_0);

  // use geometric difference to generate a sphere with an empty core
  openvdb::tools::csgDifference(*grid, *sphere_0);

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
  openvdb::tools::foreach (grid->beginValueAll(), Local::op);

  // Meta data
  grid->setGridClass(openvdb::GRID_LEVEL_SET);
  grid->setName("LevelSetSphere");

  save_grid("nano_grid.vdb", grid);

  return grid;
}

void Benchmarker::save_grid(std::string filename, const openvdb::GridBase::Ptr grid)
{
  // generate absolute file path
  std::string outfile(global_settings["out_dir"]);
  outfile += filename;

  // save to file (see:
  // https://academysoftwarefoundation.github.io/openvdb/codeExamples.html#sHelloWorld)
  openvdb::io::File vdb_file(outfile);
  openvdb::GridPtrVec grids;
  grids.push_back(grid);
  vdb_file.write(grids);
  vdb_file.close();
}

void Benchmarker::run_singleSphere()
{
  // set number of rays for the benchmark
  ray_vals = logspace(options["nrays_min"].as<int>(), options["nrays_max"].as<int>(), BASE2,
                      options["nbench"].as<int>());

  auto level_set_ovbd = generate_doubleSphere();

  // Convert to nanoVDBV
  // Note: it is possible to create Level sets directly in NanoVDB as well bus this is slower
  auto level_set_cpu = nanovdb::openToNanoVDB<nanovdb::HostBuffer>(*level_set_ovbd);
  auto level_set_gpu = nanovdb::openToNanoVDB<nanovdb::CudaDeviceBuffer>(*level_set_ovbd);

  // convert grid back to open_vdb and save it.
  // mainly for debugging and checking if the grid is correctly converted
  save_grid("nano_grid.vdb", nanovdb::nanoToOpenVDB(level_set_cpu));

  // Run Benchmarks
  for (size_t n_rays : ray_vals)
  {
    run_openVDB(level_set_ovbd, n_rays);
    // run_nanoVDB_CPU(level_set_cpu, n_rays);
    // run_nanoVDB_GPU(level_set_gpu, n_rays);
    PLOG_INFO << "Done" << std::endl << std::endl;
  }
}

// TODO: move to separate File
void Benchmarker::run_nanoVDB_CPU(nanovdb::GridHandle<nanovdb::HostBuffer> &level_set,
                                  size_t n_rays)
{ /*
  using NVDB_RayT = nanovdb::Ray<FP_Type>;

  auto *h_grid = level_set.grid<FP_Type>();

  std::vector<NVDB_RayT> rays = generate_rays(h_grid, n_rays);
  std::vector<NVBD_Vec3T> reference_intersections =
      calculate_reference_solution<NVBD_Vec3T>(n_rays, sphere_radius_1);

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
  */
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
    if (!isClose_vec3(result_intersections[i], reference_intersections[i]))
    {
      PLOG_ERROR << "Calculated value does not match at pos " << i << std::endl;
      PLOG_ERROR << "Received:\t(" << result_intersections[i][0] << "|"
                 << result_intersections[i][1] << "|" << result_intersections[i][2] << ")"
                 << std::endl;

      PLOG_ERROR << "Should be:\t(" << reference_intersections[i][0] << "|"
                 << reference_intersections[i][1] << "|" << reference_intersections[i][2] << ")"
                 << std::endl;
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
