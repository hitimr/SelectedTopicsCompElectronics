#pragma once



// Used Types
// typedef float RealT;
using RealT = double;
typedef openvdb::math::Ray<RealT> RayT;
typedef RayT::Vec3Type Vec3T;
typedef openvdb::tools::LevelSetRayIntersector<openvdb::FloatGrid> RayIntersectorT;
typedef boost::program_options::variables_map OptionsT;

// Logging
#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Initializers/RollingFileInitializer.h>
#include <plog/Log.h>

class CustomPlogFormatter
{
public:
  static plog::util::nstring
  header() // This method returns a header for a new file. In our case it is empty.
  {
    return plog::util::nstring();
  }

  static plog::util::nstring
  format(const plog::Record &record) // This method returns a string from a record.
  {
    plog::util::nostringstream ss;
    ss << record.getMessage(); // Produce a simple string with a log message.

    return ss.str();
  }
};

// Default CLI Params
#define DEFAULT_NBENCH 16
#define DEFAULT_NRAYS_MIN 1
#define DEFAULT_NRAYS_MAX 24
#define DEFAULT_LOG_LEVEL plog::info
#define DEFAULT_VOXEL_SIZE 0.05
#define DEFAULT_RADIUS 5.0


// MISC
#define BASE2 2
#define BASE10 10
