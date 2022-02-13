#include <openvdb/Exceptions.h>
#include <openvdb/Types.h>
#include <openvdb/math/Ray.h>
#include <openvdb/math/Transform.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/tools/RayIntersector.h>
#include <openvdb/tools/RayTracer.h> // for Film


#include <iostream>
#include <vector>


using namespace openvdb;

typedef float RealT;
typedef math::Ray<double> RayT;
typedef RayT::Vec3Type Vec3T;


//std::vector<FP_TYPE> linspace(FP_TYPE )

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

/*
  int n_rays = 12;
  for(int i = 0; i < n_rays; i++)
  {
    FP_TYPE x 
  }
*/
  Vec3f dir(1.0, 0.0, 0.0);
  dir = dir / dir.length();
  Vec3f eye(0.5, 0.5, 0.0);
  math::Ray<Real> ray(eye, dir);

  Vec3T xyz(0);
  Real time = 0;
  lsri.intersectsWS(ray, xyz);
  std::cout << xyz << std::endl;


  // Create a VDB file object and write out the grid.
  openvdb::io::File("mygrids.vdb").write({ls});
}