#include "MathUtils.h"

std::vector<double> MathUtils::linespace(double start, double end, int number)
{
  std::vector<double> array;
  double step = (end - start) / (number - 1);

  while (start <= end)
  {
    array.push_back(start);
    start += step;  // could recode to better handle rounding errors
  }
  return array;
}