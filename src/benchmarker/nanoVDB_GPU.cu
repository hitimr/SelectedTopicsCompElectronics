#include "benchmarker.hpp"
#include "nanoVDB_GPU.hpp"

#include <nanovdb/util/Ray.h>

#include <vector>

__global__ void run(nanovdb::Grid<nanovdb::NanoTree<FP_Type>> *d_level_set, size_t n_rays)
{

  auto acc = d_level_set->tree().getAccessor();
  nanovdb::Coord ijk;
  FP_Type t0;
  FP_Type v;
}

void run_nanoVDB_GPU(nanovdb::GridHandle<nanovdb::CudaDeviceBuffer> &grid_handle, size_t n_rays)
{
  using FP_Type = float;
  using RayT = nanovdb::Ray<FP_Type>;
  using Vec3T = nanovdb::Vec3<FP_Type>;

  FP_Type sphere_radius_outer = 5; // TODO: replace with optionsd argument

  assert(n_rays > 0);
  PLOG_INFO << "Running NanoVDP on GPU benchmark for " << n_rays << " Rays" << std::endl;

  std::vector<RayT> rays = generate_rays<RayT>(n_rays);
  std::vector<Vec3T> reference_solutions =
      calculate_reference_solution<Vec3T>(n_rays, sphere_radius_outer);

  // Send data to GPU
  grid_handle.deviceUpload(); // TODO: move outside so it wont be called every time
  nanovdb::Grid<nanovdb::NanoTree<FP_Type>> * d_grid_handle = grid_handle.deviceGrid<FP_Type>();
  if (!d_grid_handle)
    throw std::runtime_error("GridHandle does not contain a valid device grid");

  // Run Benchmark
  std::vector<Vec3T> calculated(n_rays, Vec3T(0, 0, 0)); // results
  run<<<256, 256>>>(d_grid_handle, n_rays);
}
