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

template <class FP_Type = double> class Benchmarker
{
  using OptionsT = boost::program_options::variables_map;

private:
  // openvdb::FloatGrid::Ptr level_set;

public:
  ~Benchmarker(){};
  Benchmarker(OptionsT options);
  Benchmarker(){};
};

// Valid instantiations
template class Benchmarker<double>;
template class Benchmarker<float>;
