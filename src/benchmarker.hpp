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




#include "common.hpp"

class Benchmarker
{
private:
openvdb::FloatGrid::Ptr level_set;

public:
    Benchmarker(openvdb::FloatGrid::Ptr level_set) : level_set(level_set) {}
    ~Benchmarker();
};

