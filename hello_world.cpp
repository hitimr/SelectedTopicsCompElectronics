#include <vector>
#include <openvdb/openvdb.h>
#include <openvdb/math/Ray.h>
#include <openvdb/Types.h>
#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/tools/RayIntersector.h>
#include <openvdb/tools/RayTracer.h>

using namespace openvdb;

int main()
{
    openvdb::initialize();
    // Create a FloatGrid and populate it with a narrow-band
    // signed distance field of a sphere.
    const float r = 2.0;
    const Vec3f c(0,0,0);
    const float s = 0.5f, w = 2;

    openvdb::FloatGrid::Ptr ls = tools::createLevelSetSphere<FloatGrid>(r, c, s, w);

    openvdb::tools::LevelSetRayIntersector<FloatGrid> lsri(*ls);

    const Vec3f dir(1.0, 0.0, 0.0);
    const Vec3f eye(2.0, 0.0, 0.0);
    const math::Ray<float> ray(eye, dir);


    Real time = 0;
    //lsri.intersectsWS(ray)



    // Create a VDB file object and write out the grid.
    openvdb::io::File("mygrids.vdb").write({ls});


}