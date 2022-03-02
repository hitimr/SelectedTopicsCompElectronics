#include "benchmarker.hpp"
#include "nanoVDB_GPU.hpp"

#include <nanovdb/util/Ray.h>

#include <vector>

// TODO: switch to common definition of
using FP_Type = float;
using RayT = nanovdb::Ray<FP_Type>;
using Vec3T = nanovdb::Vec3<FP_Type>;

// TODO: rename fucntion
__global__ void run(nanovdb::Grid<nanovdb::NanoTree<FP_Type>> *d_level_set, RayT *rays,
                    size_t n_rays)
{

  unsigned int n_threads = blockDim.x * gridDim.x;
  unsigned int thread_id = blockIdx.x * blockDim.x + threadIdx.x;

  auto acc = d_level_set->tree().getAccessor();
  nanovdb::Coord ijk;
  FP_Type t0 = 0;
  FP_Type v;

  for (unsigned int i = thread_id; i < n_rays; i += n_threads)
  {
        nanovdb::ZeroCrossing(rays[i], acc, ijk, v, t0);
        assert(t0 > 0); // TODO: replace with proper result verification

  }
}

void run_nanoVDB_GPU(nanovdb::GridHandle<nanovdb::CudaDeviceBuffer> &grid_handle, size_t n_rays)
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

  // Run Benchmark
  std::vector<Vec3T> calculated(n_rays, Vec3T(0, 0, 0)); // results

  Timer timer;
  timer.reset();
  run<<<grid_size, block_size>>>(d_grid_handle, d_rays, n_rays);
  cudaDeviceSynchronize();

  double time = timer.get();
  PLOG_INFO << "NanoVDB on GPU Finished in " << time << "s (" << (double)n_rays / (1000 * time)
            << " kRays/s)" << std::endl;

  cudaFree(d_rays);
}
