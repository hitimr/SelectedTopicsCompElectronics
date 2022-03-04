#pragma once

#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

class Timer
{
public:
  Timer(bool store_timings = false) : m_store_timings(store_timings) { reset(); }

  void reset() { chrono_ts = std::chrono::high_resolution_clock::now(); }

  double get()
  {
    auto end_time = std::chrono::high_resolution_clock::now();
    double duration =
        std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - chrono_ts).count() * 1e-9;

    if (m_store_timings)
    {
      m_timings.push_back(duration);
    }

    return duration;
  }

  double median()
  {
    assert(m_timings.size() > 0);

    return m_timings[m_timings.size() / 2];
  }

  double average()
  {
    assert(m_timings.size() > 0);

    double sum = 0;
    for (auto timing : m_timings)
    {
      sum += timing;
    }
    return sum / (double)m_timings.size();
  }

  double std_dev()
  {
    double avg = average();
    double sigma2 = 0;
    for (size_t i = 0; i < m_timings.size(); i++)
    {
      sigma2 += (m_timings[i] - avg) * (m_timings[i] - avg);
    }

    return sqrt(sigma2) / (double)m_timings.size();
  }

  double m_ts;
  bool m_store_timings;
  std::vector<double> m_timings;
  std::chrono::time_point<std::chrono::high_resolution_clock> chrono_ts;
};


