#pragma once
#include "../common.hpp"

#ifdef __linux__
#include <libgen.h>       // dirname
#include <linux/limits.h> // PATH_MAX
#include <string>
#include <unistd.h> // readlink
#else
#error Platform not supported
#endif // linux

#include <stdexcept>

std::string get_proj_root_dir()
{
#ifdef __linux__
  // Get path to exe
  char result[PATH_MAX];
  ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
  std::string path;
  if (count != -1)
  {
    path = dirname(result);
  }

#else
#error Platform not supported
#endif // linux

  std::string root_folder_name(PROJ_ROOT_FOLDER_NAME);
  size_t pos = path.find(root_folder_name);
  if (pos == std::string::npos)
  {
    throw std::runtime_error("Could not locate project root dir");
  }

  path = path.erase(pos + root_folder_name.size()) + "/";

  return path;
}

template <typename T> std::vector<T> linspace(T start, T end, size_t count)
{
  assert(end - start != 0.0);
  T step = (end - start) / count;
  std::vector<T> ret_vals(count);

  for (size_t i = 0; i < count; i++)
  {
    ret_vals[i] = start + i * step;
  }
  return ret_vals;
}

// TODO: this function is probably buggy -> fix asap
std::vector<int> logspace(int start, int end, int base, size_t sample_cnt)
{
  std::vector<int> n_vals;
  double spacing = (end - start) / sample_cnt;
  for (size_t i = 0; i < sample_cnt; i++)
  {
    double exp = start + i * spacing;
    int n = (int)pow(base, exp);
    n_vals.push_back(n);
  }
  return n_vals;
}