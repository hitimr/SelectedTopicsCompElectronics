#pragma once
#ifndef _WIN32
#include <sys/time.h>
#else
#include <chrono>
#endif
#include <iostream>

/** 
 * @brief Simple timer class based on gettimeofday (POSIX).
 * Original File provided by Karl Rupp
 * 
 * Wiondwos compatibility added by Mario Hiti
 *
 */
class Timer
{
public:
  Timer() : ts(0) {}

  void reset()
  {
#ifndef _WIN32
    struct timeval tval;
    gettimeofday(&tval, NULL);
    ts = static_cast<double>(tval.tv_sec * 1000000 + tval.tv_usec);
#else
    chrono_ts = std::chrono::high_resolution_clock::now();
#endif
  }

  double get() const
  {
#ifndef _WIN32
    struct timeval tval;
    gettimeofday(&tval, NULL);
    double end_time = static_cast<double>(tval.tv_sec * 1000000 + tval.tv_usec);

    return static_cast<double>(end_time - ts) / 1000000.0;

#else
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - chrono_ts);
    return duration.count() * 1e-9;
#endif
  }

private:
  double ts;
  std::chrono::time_point<std::chrono::high_resolution_clock> chrono_ts;
};