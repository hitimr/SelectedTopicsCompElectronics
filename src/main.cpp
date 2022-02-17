// OpenVDB
#include <openvdb/Exceptions.h>
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
#include "common.hpp"
#include "util/misc.hpp"
#include "util/timer.hpp"
#include "benchmarker/benchmarker.hpp"

// Standard Library
#include <cassert>
#include <iostream>
#include <vector>

// Namespaces
using namespace openvdb;

using OptionsT = boost::program_options::variables_map;

void benchmark(RayIntersectorT &lsri, int n_rays, const OptionsT &options)
{
  // Extract parameters from optinos
  RealT voxel_size = options["voxel_size"].as<RealT>();
  RealT radius = options["radius"].as<RealT>();

  // generate a circular range of rays with origin at 0,0,0
  // all rays point along the x-y-Plane. z is kept at 0 for now
  Vec3T eye({0, 0, 0}); // all rays have a common origin
  std::vector<RayT> rays(n_rays);
  std::vector<Vec3T> reference_solutions(n_rays);
  std::vector<RealT> alpha_vals = linspace<RealT>(0.0, 2.0 * M_PI, n_rays);
  for (size_t i = 0; i < n_rays; i++)
  {
    Vec3T direction({
        math::Cos(alpha_vals[i]), // x = Cos(α)
        math::Sin(alpha_vals[i]), // y = Sin(α)
        0                         // z = 0
    });

    direction.normalize();
    rays[i] = RayT(eye, direction);

    Vec3T solution({radius * math::Cos(alpha_vals[i]), radius * math::Sin(alpha_vals[i]), 0});
    reference_solutions[i] = solution;
  }

  std::vector<Vec3T> calculated(n_rays, Vec3T(0, 0, 0)); // results

  // Run Benchmark
  Timer timer;
  timer.reset();
  for (size_t i = 0; i < n_rays; i++)
  {
    // generate ray
    lsri.intersectsWS(rays[i], calculated[i]);
  }
  double time = timer.get();
  PLOG_INFO << "Finished in " << time << "s (" << (double) n_rays/(1000*time) <<" kRays/s)" << std::endl;

  // Verify Solution
  RealT eps = voxel_size / 2;
  Vec3T vec_eps(eps, eps, eps);
  for (size_t i = 0; i < n_rays; i++)
  {
    assert(math::isApproxEqual(calculated[i], reference_solutions[i], vec_eps));
  }

  // results for each ray
  PLOG_DEBUG << "Voxel size: " << voxel_size << std::endl;
  PLOG_DEBUG << "Calculated | reference" << std::endl;
  for (size_t i = 0; i < n_rays; i++)
  {
    PLOG_DEBUG << "Ray " << i << std::endl;
    PLOG_DEBUG << "x: " << calculated[i].x() << "|" << reference_solutions[i].x() << std::endl;
    PLOG_DEBUG << "y: " << calculated[i].y() << "|" << reference_solutions[i].y() << std::endl;
    PLOG_DEBUG << "z: " << calculated[i].z() << "|" << reference_solutions[i].z() << std::endl;
    PLOG_DEBUG << std::endl;
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
    
    // Colume Settings
    ("voxel_size", po::value<RealT>()->default_value(DEFAULT_VOXEL_SIZE), "voxel size in world units")
    ("radius", po::value<RealT>()->default_value(DEFAULT_RADIUS), "sphere radius")

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
  // Parse CLI options
  OptionsT options = parse_options(ac, av);

  // Set up Logging
  static plog::ColorConsoleAppender<CustomPlogFormatter> consoleAppender;
  int loglevel = options["loglevel"].as<int>();
  plog::init(plog::Severity(loglevel), &consoleAppender);

  // Init OpenVBD
  openvdb::initialize();


  Benchmarker<double> benchmarker();
/*
  // Create Level Set sphere
  // for details see:
  // https://www.openvdb.org/documentation/doxygen/namespaceopenvdb_1_1v8__0_1_1tools.html#a47e7b3c363d0d3a15b5859c4b06e9d8b
  const Vec3f center(0, 0, 0);
  const float half_width = 2;
  openvdb::FloatGrid::Ptr ls = tools::createLevelSetSphere<FloatGrid>(
      options["radius"].as<RealT>(),     // radius of the sphere in world units
      center,                            // center of the sphere in world units
      options["voxel_size"].as<RealT>(), // voxel size in world units
      half_width                         // half the width of the narrow band, in voxel units
  );

  // intersector
  RayIntersectorT lsri(*ls);


  std::vector<int> ray_vals =
      logspace(options["nrays_min"].as<int>(), options["nrays_max"].as<int>(), BASE2,
               options["nbench"].as<int>());

  // Run Benchmark
  for(int nrays : ray_vals)
  {
    PLOG_INFO << "\nRunning benchmark for " << nrays << " Rays" << std::endl;
    benchmark(lsri, nrays, options);
  }
  

  PLOG_INFO << "Benchmark finished" << std::endl;
  */

  return EXIT_SUCCESS;
}