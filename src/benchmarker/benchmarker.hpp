#pragma once
#include <openvdb/Types.h>
#include <openvdb/math/Ray.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/RayIntersector.h>

// Boost
#include <boost/program_options.hpp>

class Benchmarker
{

  /*
    old version

    using Vec3T = openvdb::math::Vec3<FP_Type>;
    using RayT = openvdb::math::Ray<FP_Type>;
    using TreeT = typename openvdb::tree::Tree4<FP_Type, 5, 4, 3>::Type;
    using GridT = openvdb::Grid<TreeT>;
    openvdb::tools::LevelSetRayIntersector<GridT> lsri(*level_set);

      using RayIntersectorT =
        openvdb::tools::LevelSetRayIntersector<GridT, openvdb::tools::LinearSearchImpl<GridT>,
                                               GridT::TreeType::RootNodeType::ChildNodeType::LEVEL,
    RayT>;


  */

#ifdef USE_FLOAT

#else
  using FP_Type = double;
  using Vec3T = openvdb::math::Vec3<FP_Type>;
  using RayT = openvdb::math::Ray<FP_Type>;
  using GridT = openvdb::DoubleGrid;
#endif

  using OptionsT = boost::program_options::variables_map;

private:
  GridT::Ptr m_level_set;
  const OptionsT &options;

  std::vector<int> ray_vals;
  const FP_Type pi = std::acos(-1);

public:
  ~Benchmarker(){};
  Benchmarker(const OptionsT &options);

  void run(size_t nrays);
};

/*
// Valid instantiations. Required for Template ClasA
template class Benchmarker<double>;
template class Benchmarker<float>;
*/
