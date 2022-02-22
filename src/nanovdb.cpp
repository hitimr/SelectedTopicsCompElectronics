#include "common.hpp"
#include <algorithm>
#include <iostream>
#include <nanovdb/util/CudaDeviceBuffer.h>
#include <nanovdb/util/GridBuilder.h>
#include <nanovdb/util/HDDA.h>
#include <nanovdb/util/IO.h>
#include <nanovdb/util/Primitives.h>
#include <nanovdb/util/Ray.h>

#if defined(NANOVDB_USE_CUDA)
using BufferT = nanovdb::CudaDeviceBuffer;
#else
using BufferT = nanovdb::HostBuffer;
#endif

int main(int ac, char **av)
{
  using GridT = nanovdb::FloatGrid;
  using CoordT = nanovdb::Coord;
  using RealT = float;
  using Vec3T = nanovdb::Vec3<RealT>;
  using RayT = nanovdb::Ray<RealT>;


  nanovdb::GridHandle<BufferT> handle;
  handle = nanovdb::createLevelSetSphere<float, float, BufferT>(
      100.0f, nanovdb::Vec3f(-20, 0, 0), 1.0, 3.0, nanovdb::Vec3d(0), "sphere");

  auto *h_grid = handle.grid<float>();

  Vec3T rayEye(0, 0, 0);
  Vec3T rayDir(1, 0, 0);
  RayT ray(rayEye, rayDir);

  auto acc = h_grid->tree().getAccessor();
  float t0;
  CoordT ijk;
  float v;
  nanovdb::ZeroCrossing(ray, acc, ijk, v, t0);



  return 0;
}