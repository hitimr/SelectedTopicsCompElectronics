#pragma once
#include "stdafx.hpp"

#define SPHERE_RADIUS 50.
#define SPHERE_CENTER_X 100.
#define SPHERE_CENTER_Y 0.
#define SPHERE_CENTER_Z 0.
#define VOXEL_SIZE 5
#define HALFWIDTH 25

#define RAY_COUNT 100
#define RAY_START_Z -50.
#define RAY_END_Z 50.

using OpenRayT = openvdb::math::Ray<float>;
using NanoRayT = nanovdb::Ray<float>;
using NanoVec3T = nanovdb::Vec3f;
using OpenVec3T = openvdb::Vec3f;
using OpenGridT = openvdb::FloatGrid;
using NanoGridT = nanovdb::FloatGrid;
using NanoCoordT = nanovdb::Coord;
using NanoBufferT = nanovdb::HostBuffer;
using ULDataFrame = hmdf::StdDataFrame<unsigned long>;