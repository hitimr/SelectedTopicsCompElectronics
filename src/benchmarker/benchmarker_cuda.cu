#include "benchmarker.hpp"
#include <nanovdb/util/Ray.h>
#include <vector>

__global__ void kernel_raytracing(nanovdb::Grid<nanovdb::NanoTree<FP_Type>> *d_level_set,
                                  Benchmarker::NVDB_RayT *rays, FP_Type *time_results,
                                  nanovdb::Coord *result_coords, size_t n_rays)
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

void Benchmarker::run_nanoVDB_GPU(nanovdb::GridHandle<nanovdb::CudaDeviceBuffer> &level_set,
                                  size_t n_rays)
{
  size_t bytes = 0;
  nanovdb::FloatGrid *grid_handle = level_set.grid<FP_Type>();

  std::vector<OVBD_Vec3T> reference_intersections =
      calculate_reference_solution<OVBD_Vec3T>(n_rays, options["r1"].as<double>());

  // Init Grid on GPU
  level_set.deviceUpload(); // TODO: move outside so it wont be called every time
  nanovdb::Grid<nanovdb::NanoTree<FP_Type>> *d_grid_handle = level_set.deviceGrid<FP_Type>();
  if (!d_grid_handle)
    throw std::runtime_error("GridHandle does not contain a valid device grid");

  // Init rays on GPU
  bytes = sizeof(NVDB_RayT) * n_rays;
  std::vector<NVDB_RayT> rays =
      generate_rays<NVDB_GridT, NVDB_RayT>(*grid_handle, n_rays); // TODO: change to levelset
  NVDB_RayT *d_rays;
  cudaMalloc(&d_rays, bytes);
  cudaMemcpy(d_rays, rays.data(), bytes, cudaMemcpyHostToDevice);

  // Allocate Results on GPU
  bytes = sizeof(OVBD_Vec3T) * n_rays;
  std::vector<FP_Type> result_times(n_rays);
  FP_Type *d_result_times;
  cudaMalloc(&d_result_times, bytes);
  bytes = sizeof(nanovdb::Coord) * n_rays;
  std::vector<nanovdb::Coord> result_coords(n_rays);
  nanovdb::Coord *d_result_coords;
  cudaMalloc(&d_result_coords, bytes);

  // Start Benchmark
  Timer timer;
  timer.reset();
  kernel_raytracing<<<grid_size, block_size>>>(d_grid_handle, d_rays, d_result_times,
                                               d_result_coords, n_rays);
  cudaDeviceSynchronize();
  double time = timer.get();

  // Transfer results back to CPU
  cudaMemcpy(result_times.data(), d_result_times, sizeof(result_times[0]) * n_rays,
             cudaMemcpyDeviceToHost);

  cudaMemcpy(result_coords.data(), d_result_coords, sizeof(result_coords[0]) * n_rays,
             cudaMemcpyDeviceToHost);

  auto wResults = indexToWorld(*grid_handle, result_coords);
  analyze_results(wResults, reference_intersections);

  // free up GPU Allocations
  cudaFree(d_rays);
  cudaFree(d_result_coords);
  cudaFree(d_result_times);

  PLOG_INFO << "NanoVDB on GPU Finished in " << time << "s (" << (double)n_rays / (1e6 * time)
            << " MRays/s)" << std::endl;

  write_results(result_file, "NanoVDB_GPU", n_rays, time, grid_size, block_size,
                options["cpu_price"].as<double>());
}
