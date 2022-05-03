#include <openvdb/Types.h>
#include <openvdb/math/Ray.h>
#include <openvdb/math/Transform.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/LevelSetSphere.h>
#include <openvdb/tools/RayIntersector.h>
#include <openvdb/tools/RayTracer.h> // for Film

// Boost
#include <boost/assert.hpp>
#include <boost/program_options.hpp>
#include <omp.h>

// Custom
#include "benchmarker/benchmarker.hpp"
#include "common.hpp"
#include "util/misc.hpp"
#include "util/timer.hpp"

// Standard Library
#include <fstream>
#include <iostream>
#include <time.h>
#include <vector>

// Namespaces
using namespace openvdb;

using OptionsT = boost::program_options::variables_map;

Timer global_timer;
json global_settings;

void verify_cli_options(OptionsT const &options)
{
  BOOST_ASSERT_MSG(0 < options["r0"].as<double>(), "Sphere radius must be > 0");
  BOOST_ASSERT_MSG(options["r0"].as<double>() < options["r1"].as<double>(),
                   "Inner radius (r0) must be smaller than Outer radius (r1)");
  BOOST_ASSERT_MSG(options["p_rays_start"].as<int>() < options["p_rays_end"].as<int>(),
                   "Inner radius (r0) must be smaller than Outer radius (r1)");

  BOOST_ASSERT(0 < options["voxel_size"].as<double>());
  BOOST_ASSERT(0 < options["half_width"].as<double>());
  BOOST_ASSERT(0 < options["n_bench"].as<int>());
  BOOST_ASSERT(0 < options["p_rays_start"].as<int>());
  BOOST_ASSERT(0 < options["p_rays_end"].as<int>());
  BOOST_ASSERT(0 < options["n_bench"].as<int>());
  BOOST_ASSERT(0 < options["gpu_grid_size"].as<int>());
  BOOST_ASSERT(0 < options["gpu_block_size"].as<int>());


  int ray_dim = options["ray_dim"].as<int>();
  BOOST_ASSERT_MSG((ray_dim == DIM2) || (ray_dim == DIM3), "Only 2D or 3D possible");


  if(options.count("shuffle-rays") && !options.count("skip-checks"))
  {
    PLOG_WARNING << "Shuffling rays prevents result verification. It is advised to disable checks (use --skip-checks)" << std::endl;
  }

  if((options["inner_sphere_offset"].as<double>() > 0) && !options.count("skip-checks"))
  {
    PLOG_WARNING << "Offsetting the inner sphere rpevents result verification. It is advised to disable checks (use --skip-checks)" << std::endl;
  }
}

OptionsT parse_options(int ac, char **av)
{
  namespace po = boost::program_options;

  po::options_description desc("Allowed options");

  // clang-format off
  desc.add_options()
    // General
    ("help,h", "produce help message")

    ("loglevel,l", po::value<int>()->default_value(DEFAULT_LOG_LEVEL), "Log Level 0=none, fatal, error, warning, info, debug, 6=verbose")

    ("gpu_price", 
    po::value<double>()->default_value(global_settings["defaults"]["gpu_price"]), 
    "Price of GPU in EUR")

    ("cpu_price", 
    po::value<double>()->default_value(global_settings["defaults"]["gpu_price"]), 
    "Price of CPU in EUR")

    ("gpu_power", 
    po::value<double>()->default_value(global_settings["defaults"]["gpu_power"]), 
    "GPU power consumption in Watts")

    ("cpu_power", 
    po::value<double>()->default_value(global_settings["defaults"]["cpu_power"]), 
    "CPU power consumption in Watts")

    ("outfile,o",
    po::value<std::vector<std::string>>(),
    "file for data output")

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

    ("inner_sphere_offset", 
    po::value<double>()->default_value(global_settings["defaults"]["inner_sphere_offset"]), 
    "offset in x-direction of the inner sphere to force different ray times. Note this prevents result verification")

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

    ("n_bench,nb",     
    po::value<int>()->default_value(global_settings["defaults"]["n_bench"]), 
    "number of benchmarks to perform. Ray counts are logarithmically spaced between 2^p0 and 2^p1")

    ("omp_n_threads",     
    po::value<int>()->default_value(global_settings["defaults"]["omp_n_threads"]), 
    "Number of threads used for OpenVDB and NanoVDB on CPU")

    ("skip-openvdb", "Skip OpenVDB Benchmark on CPU")

    ("skip-nanovdb-cpu", "Skip NanoVDB Benchmark on CPU")

    ("skip-nanovdb-gpu", "Skip NanoVDB Benchmark on GPU")

    ("skip-checks", "Skip verification of results")

    ("calculate-error", "Performa a detailed calculation of the error after each run. (currently only implemented for NanoVDB on CUDA.")

    ("gpu_load_factor",     
    po::value<int>()->default_value(global_settings["defaults"]["gpu_load_factor"]), 
    "Number of times the GPU calculation is repeated to simulate heavier load")

    ("shuffle-rays", "Shuffle rays before passing them to the kernel. Note: performance checks are no longer possible with that option")

    // GPU Settings
    ("gpu_grid_size",     
    po::value<int>()->default_value(global_settings["defaults"]["gpu_grid_size"]), 
    "number of blocks used on GPU")

    ("gpu_block_size",     
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

  verify_cli_options(vm);

  return vm;
}

json parse_globals()
{
  std::string proj_root = get_proj_root_dir();

  // parse global json file
  std::ifstream infile(proj_root + "globals.json");
  json j;
  infile >> j;

  return j;
}

int main(int ac, char **av)
{
  global_timer = Timer();
  global_settings = parse_globals();

  // Create output folder if it does not exists yet
  int exit_status = -1;
  std::string outdir = global_settings["paths"]["out_dir"];
  execCommand("mkdir -p " + misc::abs_path(outdir), exit_status);
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