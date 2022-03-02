#include "benchmarker.hpp"
#include "nanoVDB_GPU.hpp"

#include <nanovdb/util/Ray.h>

#include <vector>

void run_nanoVDB_GPU(nanovdb::GridHandle<nanovdb::CudaDeviceBuffer> &level_set, size_t n_rays)
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
  level_set.deviceUpload(); // TODO: move outside so it wont be called every time

  // Run Benchmark
  std::vector<Vec3T> calculated(n_rays, Vec3T(0, 0, 0)); // results
}
