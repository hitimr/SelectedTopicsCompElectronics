#include <vector>


template<typename T>
std::vector<T> linspace(T start, T end, size_t count)
{
  assert(end - start != 0);
  T step = (end - start) / count;
  std::vector<T> ret_vals(count);

  for (int i = 0; i < count; i++)
  {
    ret_vals[i] = start + i * step;
  }
  return ret_vals;
}