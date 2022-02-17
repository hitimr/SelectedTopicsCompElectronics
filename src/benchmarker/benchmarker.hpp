#pragma once

// OpenVDB
#include <openvdb/Exceptions.h>
#include <openvdb/Types.h>
#include <openvdb/math/Ray.h>
#include <openvdb/math/Transform.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/tools/RayIntersector.h>
#include <openvdb/tools/RayTracer.h> // for Film

// Boost
#include <boost/program_options.hpp>

template <class FP_Type> class Benchmarker
{
  using Vec3T = openvdb::math::Vec3<FP_Type>;
  using OptionsT = boost::program_options::variables_map;
  using TreeT = openvdb::tree::Tree4<double, 5, 4, 3>::Type; // For some reason i cant put FP_TYPE here..
  using GridT = openvdb::Grid <TreeT>; // Taken from openvdb.h
  using RayIntersectorT = openvdb::tools::LevelSetRayIntersector<GridT>;

private:
  openvdb::FloatGrid::Ptr level_set;

public:
  ~Benchmarker(){};
  Benchmarker(OptionsT options);
  Benchmarker(){};
};

// Valid instantiations
template class Benchmarker<double>;
template class Benchmarker<float>;