#pragma once

#include "common.hpp"

// OpenVDB
#include <openvdb/Types.h>
#include <openvdb/math/Ray.h>
#include <openvdb/openvdb.h>

// NanoVDB
#include <nanovdb/NanoVDB.h>
#include <nanovdb/util/CudaDeviceBuffer.h>
#include <nanovdb/util/GridBuilder.h>
#include <nanovdb/util/HDDA.h>
#include <nanovdb/util/HostBuffer.h>
#include <nanovdb/util/IO.h>
#include <nanovdb/util/Primitives.h>
#include <nanovdb/util/Ray.h>

// Boost
#include <boost/program_options.hpp>

// CUDA Kernels
__global__ void run_cuda(nanovdb::Grid<nanovdb::NanoTree<FP_Type>> *d_level_set,
                         nanovdb::Ray<FP_Type> *rays, size_t n_rays);

class Benchmarker
{
public:
  // TODO: change to better names
  using OVBD_Vec3T = openvdb::math::Vec3<FP_Type>;
  using OVBD_RayT = openvdb::math::Ray<FP_Type>;
  using OVBD_GridT = openvdb::FloatGrid;

  using NVBD_CoordT = nanovdb::Coord;
  using NVBD_Vec3T = nanovdb::Vec3<FP_Type>;

  using OptionsT = boost::program_options::variables_map;

  Benchmarker(const OptionsT &options);
  ~Benchmarker(){};

  const OptionsT &options;

  // Benchmark settings
  FP_Type voxel_size = -1;
  FP_Type sphere_radius_outer = -1;
  FP_Type level_set_half_width = -1;
  std::vector<int> ray_vals;
  FP_Type eps = -1;

  // Methods
  void run();
  void run_openVDB(const OVBD_GridT::Ptr &level_set2, size_t nrays);
  void run_nanoVDB_CPU(nanovdb::GridHandle<nanovdb::HostBuffer> &level_set, size_t nrays);
  void run_nanoVDB_GPU(nanovdb::GridHandle<nanovdb::CudaDeviceBuffer> &grid_handle, size_t n_rays);

  template <typename RayT> std::vector<RayT> generate_rays(size_t n_rays);
  template <typename Vec3T>
  std::vector<Vec3T> calculate_reference_solution(size_t n_rays, FP_Type sphere_radius_outer);

  template <typename Vec3T>
  bool verify_results(const std::vector<Vec3T> &result_intersections,
                      const std::vector<Vec3T> &reference_intersections);

  template <typename Vec3T> bool isClose_vec3(const Vec3T &a, const Vec3T &b);
};

// template <>
// std::vector<nanovdb::Vec3<FP_Type>>
//     Benchmarker::calculate_reference_solution<nanovdb::Vec3<FP_Type>>(size_t, FP_Type);