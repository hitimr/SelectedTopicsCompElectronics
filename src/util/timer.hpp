#pragma once
#include <chrono>
#include <iostream>
 

class Timer
{
public:
  Timer() : ts(0) {}

  void reset() { chrono_ts = std::chrono::high_resolution_clock::now(); }

  double get() const
  {
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - chrono_ts);
    return duration.count() * 1e-9;
  }

private:
  double ts;
  std::chrono::time_point<std::chrono::high_resolution_clock> chrono_ts;
};