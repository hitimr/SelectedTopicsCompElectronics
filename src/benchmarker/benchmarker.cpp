#include "benchmarker.hpp"
//#include "common.hpp"

// OpenVDB
#include <openvdb/Exceptions.h>
#include <openvdb/Types.h>
#include <openvdb/math/Ray.h>
#include <openvdb/math/Transform.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/tools/RayIntersector.h>
#include <openvdb/tools/RayTracer.h> // for Film

template <class FP_Type> Benchmarker<FP_Type>::Benchmarker(OptionsT options)
{
  // Create Level Set sphere
  // for details see:
  // https://www.openvdb.org/documentation/doxygen/namespaceopenvdb_1_1v8__0_1_1tools.html#a47e7b3c363d0d3a15b5859c4b06e9d8b
  const Vec3T center(0, 0, 0);
  const FP_Type half_width = 2;



  GridT::Ptr ls = openvdb::tools::createLevelSetSphere<GridT>(
      options["radius"].as<FP_Type>(),     // radius of the sphere in world units
      center,                              // center of the sphere in world units
      options["voxel_size"].as<FP_Type>(), // voxel size in world units
      half_width                           // half the width of the narrow band, in voxel units
  );

  RayIntersectorT lsri(*ls);
}
