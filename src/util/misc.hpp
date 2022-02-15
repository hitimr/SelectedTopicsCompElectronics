#include "../common.hpp"

#include <vector>

template <typename T = RealT> std::vector<T> linspace(T start, T end, size_t count)
{
  assert(end - start != 0.0);
  T step = (end - start) / count;
  std::vector<T> ret_vals(count);

  for (int i = 0; i < count; i++)
  {
    ret_vals[i] = start + i * step;
  }
  return ret_vals;
}

std::vector<int> logspace(int start, int end, int base, int sample_cnt)
{
  std::vector<int> n_vals;
  double spacing = (end - start) / sample_cnt;
  for (int i = 0; i < sample_cnt; i++)
  {
    double exp = start + i * spacing;
    int n = (int)pow(base, exp);
    n_vals.push_back(n);
  }
  return n_vals;
}