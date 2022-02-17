#pragma once
#include <openvdb/Types.h>
#include <openvdb/math/Ray.h>
#include <openvdb/openvdb.h>

// Boost
#include <boost/program_options.hpp>

template <class FP_Type = double> class Benchmarker
{
  using OptionsT = boost::program_options::variables_map;
  using Vec3T = openvdb::math::Vec3<FP_Type>;
  using RayT = openvdb::math::Ray<FP_Type>;
  using TreeT = typename openvdb::tree::Tree4<FP_Type, 5, 4, 3>::Type;
  using GridT = openvdb::Grid<TreeT>; // Taken from openvdb.h

private:
  // openvdb::FloatGrid::Ptr level_set;
  std::vector<int> ray_vals;
  const OptionsT & options;

  const FP_Type pi = std::acos(-1);

public:
  ~Benchmarker(){};
  Benchmarker(const OptionsT & options);

  void run(size_t nrays);
};

// Valid instantiations
template class Benchmarker<double>;
template class Benchmarker<float>;
