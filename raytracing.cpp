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

typedef float RealT;
typedef openvdb::math::Ray<double> RayT;
typedef RayT::Vec3Type     Vec3T;

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
  openvdb::initialize();
  // Create a FloatGrid and populate it with a narrow-band
  // signed distance field of a sphere.
  const RealT r = 5;
  const Vec3f c(0, 0, 0);
  const float s = 0.1f;
  const float w = 2;

  // outer sphere
  openvdb::FloatGrid::Ptr ls = tools::createLevelSetSphere<FloatGrid>(r, c, s, w);

  // intersector
  openvdb::tools::LevelSetRayIntersector<FloatGrid> lsri(*ls);

  // generate a circular range of rays with origin at 0,0,0
  size_t n_rays = 12;
  Vec3T origin({0, 0, 0}); // all rays have a common origin
  std::vector<RayT> rays(n_rays);
  std::vector<RealT> alpha_vals = linspace(0, 2 * M_PI, n_rays);
  for (size_t i = 0; i < n_rays; i++)
  {
    Vec3T direction({
        math::Cos(alpha_vals[i]), // x = Cos(α)
        math::Sin(alpha_vals[i]), // y = Sin(α)
        0                         // z = 0
    });
    rays[i] = RayT(direction, direction);
  }
  /*
    // calculate interse4ctions
    std::vector<Vec3T> intersections(n_rays, Vec3T(0,0,0));
    for (size_t i = 0; i < n_rays; i++)
    {
      Vec3T xyz(0);
      lsri.intersectsWS(xyz);
    }
  */

  const Vec3T dir(1.0, 0.0, 0.0);
  const Vec3T eye(2.0, 0.0, 0.0);
  RayT ray(eye, dir);

  Vec3T xyz(0);
  lsri.intersectsWS(ray, xyz);
  std::cout << xyz << std::endl;

  // Create a VDB file object and write out the grid.
  openvdb::io::File("mygrids.vdb").write({ls});
}