#pragma once

using FP_Type = float;

// Default CLI Params
// TODO: migrate to json file
#define DEFAULT_NBENCH 16
#define DEFAULT_NRAYS_MIN 1
#define DEFAULT_NRAYS_MAX 24 
#define DEFAULT_LOG_LEVEL plog::info
#define DEFAULT_RADIUS 5.0

// MISC
#define BASE2 2
#define BASE10 10
#define PROJ_ROOT_FOLDER_NAME "SelectedTopicsCompElectronics"

// Exit Codes
#define EXIT_SUCCESS 0

#include "util/timer.hpp"

// Logging
#include <plog/Appenders/ColorConsoleAppender.h>
#include <plog/Initializers/RollingFileInitializer.h>
#include <plog/Log.h>

// JSON
#include <nlohmann/json.hpp>
using json = nlohmann::json;

#include <cmath>
#include <iomanip>
#include <vector>

// Constants
#ifdef M_PI
#undef M_PI
#endif // M_PI

const double M_PI = std::acos(-1); // TODO: replace with FP_type

// Globals
extern Timer global_timer;
extern json global_settings;  //  uses https://github.com/nlohmann/json

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
    ss << std::fixed << std::setprecision(3) << global_timer.get() << ": "
       << record.getMessage();

    return ss.str();
  }
};
