#include <openvdb/Exceptions.h>
#include <openvdb/Types.h>
#include <openvdb/math/Ray.h>
#include <openvdb/math/Transform.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/tools/RayIntersector.h>
#include <openvdb/tools/RayTracer.h> // for Film

#include <cassert>
#include <iostream>
#include <vector>

#define VERBOSE true

typedef float RealT;
typedef openvdb::math::Ray<double> RayT;
typedef RayT::Vec3Type Vec3T;

using namespace openvdb;

std::vector<RealT> linspace(RealT start, RealT end, size_t count)
{
  assert(end - start != 0);
  RealT step = (end - start) / count;
  std::vector<RealT> ret_vals(count);

  for (int i = 0; i < count; i++)
  {
    ret_vals[i] = start + i * step;
  }
  return ret_vals;
}

int main()
{
  /**
   * Init
   *
   */
  openvdb::initialize();

  // Create Level Set sphere
  // for details see:
  // https://www.openvdb.org/documentation/doxygen/namespaceopenvdb_1_1v8__0_1_1tools.html#a47e7b3c363d0d3a15b5859c4b06e9d8b
  const RealT radius = 5;
  const Vec3f center(0, 0, 0);
  const float voxel_size = 0.1f;
  const float half_width = 2;
  openvdb::FloatGrid::Ptr ls =
      tools::createLevelSetSphere<FloatGrid>(radius,     // radius of the sphere in world units
                                             center,     // center of the sphere in world units
                                             voxel_size, // voxel size in world units
                                             half_width  // half the width of the narrow band, in voxel units
      );

  // intersector
  openvdb::tools::LevelSetRayIntersector<FloatGrid> lsri(*ls);

  // generate a circular range of rays with origin at 0,0,0
  size_t n_rays = 12;
  Vec3T eye({0, 0, 0}); // all rays have a common origin
  std::vector<RayT> rays(n_rays);
  std::vector<Vec3T> reference_solutions(n_rays);
  std::vector<RealT> alpha_vals = linspace(0, 2 * M_PI, n_rays);
  for (size_t i = 0; i < n_rays; i++)
  {
    Vec3T direction({
        math::Cos(alpha_vals[i]), // x = Cos(α)
        math::Sin(alpha_vals[i]), // y = Sin(α)
        0                         // z = 0
    });

    direction.normalize();
    rays[i] = RayT(eye, direction);

    Vec3T solution({radius * math::Cos(alpha_vals[i]), radius * math::Sin(alpha_vals[i]), 0});
    reference_solutions[i] = solution;
  }

  /**
   * Benchmark
   *
   */
  std::vector<Vec3T> calculated(n_rays, Vec3T(0, 0, 0));
  for (size_t i = 0; i < n_rays; i++)
  {
    lsri.intersectsWS(rays[i], calculated[i]);
  }

  /**
   * Summary
   *
   */
  if (VERBOSE)
  {
    std::cout << "Benchmark finished" << std::endl;
    for (size_t i = 0; i < n_rays; i++)
    {
      std::cout << "Ray " << i << std::endl;
      std::cout << "x: " << calculated[i].x() << "|" << reference_solutions[i].x() << std::endl;
      std::cout << "y: " << calculated[i].y() << "|" << reference_solutions[i].y() << std::endl;
      std::cout << std::endl;
    }
  }

  // Create a VDB file object and write out the grid.
  // openvdb::io::File("mygrids.vdb").write({ls});
}