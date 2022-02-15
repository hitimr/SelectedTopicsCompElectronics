#pragma once
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
#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Initializers/RollingFileInitializer.h>


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
