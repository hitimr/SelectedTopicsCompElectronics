#include "common.hpp"
#include "nanoVDB_GPU.hpp"
#include "benchmarker.hpp"

#include <nanovdb/util/Ray.h>

#include <vector>

void run_nanoVDB_GPU(nanovdb::GridHandle<nanovdb::HostBuffer> &handle, size_t n_rays)
{
  using FP_Type = float; // TODO: merge this with project wide type
  using NVDB_RayT = nanovdb::Ray<FP_Type>;
  assert(n_rays > 0);
  PLOG_INFO << "Running NanoVDP on GPU benchmark for " << n_rays << " Rays" << std::endl;

  std::vector<NVDB_RayT> rays = generate_rays<NVDB_RayT>(n_rays);
  // std::vector<NVBD_Vec3T> reference_solutions = calculate_reference_solution<NVBD_Vec3T>(n_rays);
}
