#define DEFAULT_NBENCH 10
#define DEFAULT_NRAYS_START 10
#define DEFAULT_NRAYS_END 100

// Used Types
typedef double RealT;
typedef openvdb::math::Ray<double> RayT;
typedef RayT::Vec3Type Vec3T;
typedef openvdb::tools::LevelSetRayIntersector<openvdb::FloatGrid> RayIntersectorT;


// Logging
#include <plog/Log.h>
#include <plog/Init.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Appenders/ColorConsoleAppender.h>
