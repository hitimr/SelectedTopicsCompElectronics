#include "benchmarker.hpp"

#include <nanovdb/util/Ray.h>

#include <vector>

// TODO: switch to common definition of
using FP_Type = float;
using RayT = nanovdb::Ray<FP_Type>;
using Vec3T = nanovdb::Vec3<FP_Type>;

// TODO: rename fucntion
__global__ void run_cuda(nanovdb::Grid<nanovdb::NanoTree<FP_Type>> *d_level_set, RayT *rays,
                         FP_Type *time_results, nanovdb::Coord *result_coords, size_t n_rays)
{

  unsigned int n_threads = blockDim.x * gridDim.x;
  unsigned int thread_id = blockIdx.x * blockDim.x + threadIdx.x;

  auto acc = d_level_set->tree().getAccessor();
  nanovdb::Coord ijk;
  FP_Type value;

  for (unsigned int i = thread_id; i < n_rays; i += n_threads)
  {
    nanovdb::ZeroCrossing(rays[i], acc, result_coords[i], value, time_results[i]);
  }
}

void Benchmarker::run_nanoVDB_GPU(nanovdb::GridHandle<nanovdb::CudaDeviceBuffer> &grid_handle,
                                  size_t n_rays)
{
  using FP_Type = float;
  using RayT = nanovdb::Ray<FP_Type>;
  using Vec3T = nanovdb::Vec3<FP_Type>;

  size_t grid_size = 256;
  size_t block_size = 256;
  size_t bytes = 0;

  FP_Type sphere_radius_outer = 5; // TODO: replace with optionsd argument

  assert(n_rays > 0);

  std::vector<Vec3T> reference_solutions =
      calculate_reference_solution<Vec3T>(n_rays, sphere_radius_outer);

  // Init Grid on GPU
  grid_handle.deviceUpload(); // TODO: move outside so it wont be called every time
  nanovdb::Grid<nanovdb::NanoTree<FP_Type>> *d_grid_handle = grid_handle.deviceGrid<FP_Type>();
  if (!d_grid_handle)
    throw std::runtime_error("GridHandle does not contain a valid device grid");

  // Init rays on GPU
  bytes = sizeof(RayT) * n_rays;
  std::vector<RayT> rays = generate_rays<RayT>(n_rays);
  RayT *d_rays;
  cudaMalloc(&d_rays, bytes);
  cudaMemcpy(d_rays, rays.data(), bytes, cudaMemcpyHostToDevice);

  // Allocate Results on GPU
  bytes = sizeof(Vec3T) * n_rays;
  std::vector<FP_Type> result_times(n_rays);
  FP_Type *d_result_times;
  cudaMalloc(&d_result_times, bytes);

  bytes = sizeof(nanovdb::Coord) * n_rays;
  std::vector<nanovdb::Coord> result_coords(n_rays);
  nanovdb::Coord *d_result_coords;
  cudaMalloc(&d_result_coords, bytes);

  // Start Bennchmark
  Timer timer;
  timer.reset();
  run_cuda<<<grid_size, block_size>>>(d_grid_handle, d_rays, d_result_times, d_result_coords,
                                      n_rays);
  cudaDeviceSynchronize();
  double time = timer.get();

  PLOG_INFO << "NanoVDB on GPU Finished in " << time << "s (" << (double)n_rays / (1e6 * time)
            << " MRays/s)" << std::endl;

  // Transfer results back to CPU
  cudaMemcpy(result_times.data(), d_result_times, sizeof(result_times[0]) * n_rays,
             cudaMemcpyDeviceToHost);

  cudaMemcpy(result_coords.data(), d_result_coords, sizeof(result_coords[0]) * n_rays,
             cudaMemcpyDeviceToHost);

  auto *h_grid = grid_handle.grid<FP_Type>();
  std::vector<nanovdb::Vec3<FP_Type>> result_intersections(n_rays);
  for (size_t i = 0; i < n_rays; i++)
  {
    result_intersections[i] = h_grid->indexToWorldF<Vec3T>(result_coords[i].asVec3s());
  }

  cudaFree(d_rays);
  cudaFree(d_result_coords);
  cudaFree(d_result_times);
}
