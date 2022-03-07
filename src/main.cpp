#include <openvdb/Types.h>
#include <openvdb/math/Ray.h>
#include <openvdb/math/Transform.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/tools/RayIntersector.h>
#include <openvdb/tools/RayTracer.h> // for Film

// Boost
#include <boost/program_options.hpp>

// Custom
#include "benchmarker/benchmarker.hpp"
#include "common.hpp"
#include "util/misc.hpp"
#include "util/timer.hpp"

// Standard Library
#include <cassert>
#include <iostream>
#include <fstream>
#include <time.h>
#include <vector>

// Namespaces
using namespace openvdb;

using OptionsT = boost::program_options::variables_map;

Timer global_timer;

OptionsT parse_options(int ac, char **av)
{
  namespace po = boost::program_options;

  po::options_description desc("Allowed options");

  // clang-format off
  desc.add_options()
    // General
    ("help,h", "produce help message")
    ("loglevel,l", po::value<int>()->default_value(DEFAULT_LOG_LEVEL), "Log Level 0=none, fatal, error, warning, info, debug, 6=verbose")
    
    // Colume Settings
    ("voxel_size", po::value<double>()->default_value(DEFAULT_VOXEL_SIZE), "voxel size in world units")
    ("radius", po::value<double>()->default_value(DEFAULT_RADIUS), "sphere radius")

    // Benchmakr settings
    ("nrays_min", po::value<int>()->default_value(DEFAULT_NRAYS_MIN), "minimum number of rays per benchmark")
    ("nrays_max", po::value<int>()->default_value(DEFAULT_NRAYS_MAX), "maximum number of rays per benchmark")
    ("nbench,nb", po::value<int>()->default_value(DEFAULT_NBENCH), "number of points for the benchmark");
  // clang-format on
  po::variables_map vm;
  po::store(po::parse_command_line(ac, av, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << "\n";
    exit(EXIT_SUCCESS);
  }

  // check sanity of arguments
  assert(vm["nrays_min"].as<int>() < vm["nrays_max"].as<int>());

  return vm;
}

int main(int ac, char **av)
{
  global_timer = Timer();

  // parse global json file
  std::ifstream i("../../globals.json");
  json j;
  i >> j;
  std::cout << j["out_dir"] << std::endl;

  // Parse CLI options
  OptionsT options = parse_options(ac, av);

  // Set up Logging
  static plog::ColorConsoleAppender<CustomPlogFormatter> consoleAppender;
  int loglevel = options["loglevel"].as<int>();
  plog::init(plog::Severity(loglevel), &consoleAppender);

  // Init OpenVBD
  openvdb::initialize();

  Benchmarker benchmarker(options);
  benchmarker.run_all();

  return EXIT_SUCCESS;
}