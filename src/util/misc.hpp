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

#include <array>
#include <stdexcept>

static std::string get_proj_root_dir()
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

namespace misc
{
  static std::string abs_path(std::string rel_path)
  {
    return get_proj_root_dir() + rel_path;
  }
}

/**
 * @brief execute a shell command
 * taken from https://dev.to/aggsol/calling-shell-commands-from-c-8ej
 * 
 * @param cmd shell command
 * @param out_exitStatus return status of 
 * @return std::string output of shell command
 */
static std::string execCommand(const std::string cmd, int &out_exitStatus)
{
  out_exitStatus = 0;
  auto pPipe = ::popen(cmd.c_str(), "r");
  if (pPipe == nullptr)
  {
    throw std::runtime_error("Cannot open pipe");
  }

  std::array<char, 256> buffer;

  std::string output;

  while (not std::feof(pPipe))
  {
    auto bytes = std::fread(buffer.data(), 1, buffer.size(), pPipe);
    output.append(buffer.data(), bytes);
  }

  auto rc = ::pclose(pPipe);

  if (WIFEXITED(rc))
  {
    out_exitStatus = WEXITSTATUS(rc);
  }

  return output;
}

template <typename T> static std::vector<T> linspace(T start, T end, size_t count)
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

template<typename T = int>
static std::vector<T> logspace(double start, double end, double base, size_t sample_cnt)
{
  std::vector<T> vals;
  double spacing = (end - start) / (double) sample_cnt;
  for (size_t i = 0; i < sample_cnt; i++)
  {
    double exp = start + i * spacing;
    T n = (T)pow(base, exp);
    vals.push_back(n);
  }
  return vals;
}