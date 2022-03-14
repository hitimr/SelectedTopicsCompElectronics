#include <openvdb/Types.h>
#include <openvdb/math/Ray.h>
#include <openvdb/math/Transform.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/tools/RayIntersector.h>
#include <openvdb/tools/RayTracer.h> // for Film

// Boost
#include <boost/program_options.hpp>
#include <omp.h>

// Custom
#include "benchmarker/benchmarker.hpp"
#include "common.hpp"
#include "util/misc.hpp"
#include "util/timer.hpp"

// Standard Library
#include <cassert>
#include <fstream>
#include <iostream>
#include <time.h>
#include <vector>

// Namespaces
using namespace openvdb;

using OptionsT = boost::program_options::variables_map;

Timer global_timer;
json global_settings;

OptionsT parse_options(int ac, char **av)
{
  namespace po = boost::program_options;

  po::options_description desc("Allowed options");

  // clang-format off
  desc.add_options()
    // General
    ("help,h", "produce help message")
    ("loglevel,l", po::value<int>()->default_value(DEFAULT_LOG_LEVEL), "Log Level 0=none, fatal, error, warning, info, debug, 6=verbose")
    
    // Grid Settings
    ("voxel_size", 
    po::value<double>()->default_value(global_settings["defaults"]["voxel_size"]), 
    "voxel size in world units")

    ("r0", 
    po::value<double>()->default_value(global_settings["defaults"]["default_radius_inner"]), 
    "sphere radius r0")

    ("r1", 
    po::value<double>()->default_value(global_settings["defaults"]["default_radius_outer"]), 
    "sphere radius r1")

    ("half_width", 
    po::value<double>()->default_value(global_settings["defaults"]["half_width"]), 
    "level set half width in voxel units")

    ("ray_offset",
    po::value<double>()->default_value(global_settings["defaults"]["ray_offset"]),
    "If rays are placed directly on the inner sphere they may immediately intersect again\n\
    For now a small offset is used to prevent this.\n\
    Ideally the offset is still within the narrow band")

    // Benchmark settings
    ("p_rays_start,p0",  
    po::value<int>()->default_value(global_settings["defaults"]["p_rays_start"]), 
    "2s complement of minimum number of rays for the benchmark")

    ("p_rays_end,p1",    
    po::value<int>()->default_value(global_settings["defaults"]["p_rays_end"]), 
    "2s complement of the maximum number of rays for the benchmark")

    ("nbench,nb",     
    po::value<int>()->default_value(global_settings["defaults"]["n_bench"]), 
    "number of benchmarks to perform. Ray counts are logarithmically spaced between 2^p0 and 2^p1")

    ("omp_n_threads",     
    po::value<int>()->default_value(global_settings["defaults"]["omp_n_threads"]), 
    "Number of threads used for OpenVDB and NanoVDB on CPU")

    // GPU Settings
    ("grid_size",     
    po::value<int>()->default_value(global_settings["defaults"]["gpu_grid_size"]), 
    "number of blocks used on GPU")

    ("block_size",     
    po::value<int>()->default_value(global_settings["defaults"]["gpu_block_size"]), 
    "number of threads used per block")

    ("ray_dim,d",     
    po::value<int>()->default_value(global_settings["defaults"]["ray_distribution_dimensions"]), 
    "[2, 3] number of dimensions used for distribution (circular or spherical)");

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
  assert(vm["p_rays_start"].as<int>() < vm["p_rays_end"].as<int>());
  assert(vm["omp_n_threads"].as<int>() > 0);

  return vm;
}

json parse_globals()
{
  std::string proj_root = get_proj_root_dir();

  // parse global json file
  std::ifstream infile(proj_root + "globals.json");
  json j;
  infile >> j;

  j["proj_root"] = proj_root;
  j["out_dir"] = proj_root + std::string(j["out_dir"]); // Update with absolute path

  return j;
}

int main(int ac, char **av)
{
  global_timer = Timer();
  global_settings = parse_globals();

  // Create output folder if it does not exists yet
  int exit_status = -1;
  std::string outdir = global_settings["out_dir"];
  execCommand("mkdir -p " + outdir, exit_status);
  assert(exit_status == EXIT_SUCCESS);

  // Parse CLI options
  OptionsT options = parse_options(ac, av);

  // Set up Logging
  static plog::ColorConsoleAppender<CustomPlogFormatter> consoleAppender;
  int loglevel = options["loglevel"].as<int>();
  plog::init(plog::Severity(loglevel), &consoleAppender);

  // Init OpenVBD
  openvdb::initialize();

  int n_threads = options["omp_n_threads"].as<int>();
  omp_set_num_threads(n_threads);


  Benchmarker benchmarker(options);
  benchmarker.run();

  PLOG_INFO << "Benchmark finished after " << global_timer.get() << "s" << std::endl;
  return EXIT_SUCCESS;
}