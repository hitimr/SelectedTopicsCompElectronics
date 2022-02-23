#pragma once
#include <openvdb/Types.h>
#include <openvdb/math/Ray.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/RayIntersector.h>

// Boost
#include <boost/program_options.hpp>

class Benchmarker
{
public:
  using FP_Type = double;
  using OVBD_Vec3T = openvdb::math::Vec3<FP_Type>;
  using RayT = openvdb::math::Ray<FP_Type>;
  using GridT = openvdb::DoubleGrid;

  using OptionsT = boost::program_options::variables_map;

  const OptionsT &options;

  FP_Type voxel_size = -1;
  FP_Type sphere_radius_outer = -1;
  FP_Type level_set_half_width = -1;

  std::vector<int> ray_vals;
  const FP_Type pi = std::acos(-1);

  ~Benchmarker(){};
  Benchmarker(const OptionsT &options);

  void run();
  void run_openVDB(size_t nrays);
  void run_nanoVDB(size_t nrays);
  template <typename T> std::vector<T> generate_rays(size_t n_rays);
  template <typename Vec3T> std::vector<Vec3T> calculate_reference_solution(size_t n_rays);

  template <typename T>
  bool verify_results(const std::vector<T> &calculated, const std::vector<T> &reference,
                      int &err_pos = -1);
};

/*
// Valid instantiations. Required for Template ClasA
template class Benchmarker<double>;
template class Benchmarker<float>;
*/
