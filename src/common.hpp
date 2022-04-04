#pragma once

// Default CLI Params
// TODO: migrate to json file
#define DEFAULT_LOG_LEVEL plog::info
#define DEFAULT_RADIUS 5.0

// MISC
#define BASE2 2
#define BASE10 10
#define PROJ_ROOT_FOLDER_NAME "SelectedTopicsCompElectronics"
#define DIM3 3
#define DIM2 2
#define EXIT_SUCCESS 0

#include <mmintrin.h>

#include "util/timer.hpp"
#include <nlohmann/json.hpp>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Initializers/RollingFileInitializer.h>
#include <plog/Log.h>

#include <cmath>
#include <iomanip>
#include <vector>

// Constants
#ifdef M_PI
#undef M_PI
#endif // M_PI

using json = nlohmann::json;
using FP_Type = float;

// Globals
extern Timer global_timer;
extern json global_settings; //  uses https://github.com/nlohmann/json
const FP_Type M_PI = std::acos(-1);

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
    ss << std::fixed << std::setprecision(3) << global_timer.get() << ": " << record.getMessage();

    return ss.str();
  }
};
