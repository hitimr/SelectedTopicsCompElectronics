#include "precision.hpp"
#include "stdafx.hpp"

NanoVec3T indexToWorld(NanoCoordT &index, nanovdb::GridHandle<NanoBufferT> grid_handle)
{
  auto grid = grid_handle.grid<float>();
  return NanoVec3T(index[0] * grid->voxelSize()[0], index[1] * grid->voxelSize()[1], index[2] * grid->voxelSize()[2]);
}

int main()
{
  logging::init();

  // Generate Sphere
  PLOG_INFO << "Generating Sphere";
  OpenGridT open_grid = *openvdb::tools::createLevelSetSphere<OpenGridT>(
      SPHERE_RADIUS, OpenVec3T(SPHERE_CENTER_X, SPHERE_CENTER_Y, SPHERE_CENTER_Z), VOXEL_SIZE, HALFWIDTH);
  nanovdb::GridHandle<NanoBufferT> nano_sphere = nanovdb::openToNanoVDB<NanoBufferT>(open_grid);
  auto nano_grid = nano_sphere.grid<float>();

  // Generate Rays
  PLOG_INFO << "Generating Rays";
  std::vector<float> origins_z = math::linspace<float>(RAY_START_Z, RAY_END_Z, RAY_COUNT);
  std::vector<NanoRayT> nano_iRays(RAY_COUNT);
  std::vector<NanoRayT> nano_wRays(RAY_COUNT);
  std::vector<OpenRayT> open_wRays(RAY_COUNT);
  std::vector<OpenRayT> open_iRays(RAY_COUNT);

  for (int i = 0; i < RAY_COUNT; i++)
  {
    // OpenVDB
    OpenVec3T open_eye(0, 0, origins_z[i]);
    OpenVec3T open_direction(1, 0, 0);

    OpenRayT open_wRay(open_eye, open_direction);
    OpenRayT open_iRay(open_grid.worldToIndex(open_eye), open_direction);
    open_wRays[i] = open_wRay;
    open_iRays[i] = open_iRay;

    // NanoVDB
    NanoVec3T nano_eye(0, 0, origins_z[i]);
    NanoVec3T nano_direction(1, 0, 0);

    NanoRayT nano_wRay(nano_eye, nano_direction);
    NanoRayT nano_iRay(nano_sphere.grid<float>()->worldToIndex(nano_eye), nano_direction);
    nano_iRays[i] = nano_iRay;
    nano_wRays[i] = nano_wRay;
  }

  // Calculate analytical solution
  PLOG_INFO << "Calculating analytical solution";
  std::vector<float> gt_intersect_x(RAY_COUNT);
  std::vector<float> gt_intersect_y(RAY_COUNT);
  std::vector<float> alpha_vals = math::linspace<float>(1.5 * M_PI, 0.5 * M_PI, RAY_COUNT);
  for (int i = 0; i < RAY_COUNT; i++)
  {
    gt_intersect_x[i] = SPHERE_CENTER_X + SPHERE_RADIUS * std::cos(alpha_vals[i]);
    gt_intersect_y[i] = SPHERE_CENTER_Y + SPHERE_RADIUS * std::sin(alpha_vals[i]);
  }

  // Measurement data
  // OpenVDB
  std::vector<float> open_values(RAY_COUNT);
  std::vector<float> open_times(RAY_COUNT);
  std::vector<float> open_idx_x(RAY_COUNT);
  std::vector<float> open_idx_y(RAY_COUNT);
  std::vector<float> open_intersect_x(RAY_COUNT);
  std::vector<float> open_intersect_y(RAY_COUNT);
  std::vector<OpenVec3T> open_intersect(RAY_COUNT);

  // NanoVDB
  std::vector<float> nano_values(RAY_COUNT);
  std::vector<float> nano_times(RAY_COUNT);
  std::vector<float> nano_idx_x(RAY_COUNT);
  std::vector<float> nano_idx_y(RAY_COUNT);
  std::vector<float> nano_intersect_x(RAY_COUNT);
  std::vector<float> nano_intersect_y(RAY_COUNT);
  std::vector<NanoCoordT> nano_coords(RAY_COUNT);

  PLOG_INFO << "Performing Ray intersection";

  // Perform Ray Interseciton for OpenVDB
  openvdb::tools::LevelSetRayIntersector<OpenGridT, openvdb::tools::LinearSearchImpl<OpenGridT, 0, float>,
                                         OpenGridT::TreeType::RootNodeType::ChildNodeType::LEVEL, OpenRayT>
      open_intersector(open_grid);

  for (int i = 0; i < RAY_COUNT; i++)
  {
    open_intersector.intersectsIS(open_iRays[i], open_intersect[i], open_times[i]);
    open_idx_x[i] = open_intersect[i][0];
    open_idx_y[i] = open_intersect[i][1];
    open_values[i] = open_intersector.getIsoValue();

    // Transform intersection point back to world space
    auto wIntersect = open_grid.indexToWorld(open_intersect[i]);
    open_intersect_x[i] = wIntersect[0];
    open_intersect_y[i] = wIntersect[2];
  }

  // Perform Ray Intersection for NanoVDB
  auto accessor = nano_sphere.grid<float>()->tree().getAccessor();
  for (int i = 0; i < RAY_COUNT; i++)
  {
    nanovdb::ZeroCrossing(nano_wRays[i], accessor, nano_coords[i], nano_values[i], nano_times[i]);
    nano_idx_x[i] = (float)nano_coords[i].x();
    nano_idx_y[i] = (float)nano_coords[i].y();

    // Transform intersection point back to world space
    auto wIntersect = nano_grid->indexToWorld(nano_coords[i].asVec3s());
    nano_intersect_x[i] = wIntersect[0];
    nano_intersect_y[i] = wIntersect[2];
  }

  // write Data To File
  PLOG_INFO << "Writing Data to File";
  ULDataFrame df;
  std::vector<size_t> idx(RAY_COUNT);
  std::iota(idx.begin(), idx.end(), 0);

  // clang-format off
  df.load_data(
    std::move(idx), 
    std::make_pair("origin_z", origins_z), 
    std::make_pair("gt_intersect_x", gt_intersect_x),
    std::make_pair("gt_intersect_y", gt_intersect_y),
    std::make_pair("nano_intersect_x", nano_intersect_x),
    std::make_pair("open_intersect_x", open_intersect_x),
    std::make_pair("nano_intersect_y", nano_intersect_y),
    std::make_pair("open_intersect_y", open_intersect_y),
    std::make_pair("nano_idx_x", nano_idx_x),
    std::make_pair("open_idx_x", open_idx_x),
    std::make_pair("nano_idx_y", nano_idx_y),
    std::make_pair("open_idx_y", open_idx_y),
    std::make_pair("nano_value", nano_values),
    std::make_pair("nano_time", nano_times),
    std::make_pair("open_time", open_times)
    );

  df.write<
    size_t, 
    float>
    ("data.csv", hmdf::io_format::csv2);
  //clang-format on

  return 0;
}