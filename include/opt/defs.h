
#ifndef DEFS_H
#define DEFS_H

/********************************* Includes **********************************/

/* From standard C library */
#include <math.h>
#include <stdio.h>
#include <errno.h>
#include <stdarg.h>
#include <stdlib.h>
#include <unistd.h>
#include <random>
#include <iostream>
#include <memory>
#include <vector>

#include <float.h>
#include <stddef.h>
#include <string.h>

/******************************* Defs and macros *****************************/

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif
#ifndef MIN
#define MIN(x, y) ((x < y) ? x : y)
#endif
#ifndef MAX
#define MAX(x, y) ((x > y) ? x : y)
#endif
#ifndef ABS
#define ABS(x) ((x < 0) ? -x : x)
#endif

typedef std::vector<float> VectorX;

/********************************** Structures *******************************/
template <typename T>
bool is_infinite(const T& value)
{
  // Since we're a template, it's wise to use std::numeric_limits<T>
  //
  // Note: std::numeric_limits<T>::min() behaves like DBL_MIN, and is the smallest absolute value possible.
  //

  T max_value = std::numeric_limits<T>::max();
  T min_value = -max_value;

  return !(min_value <= value && value <= max_value);
}

template <typename T>
bool is_nan(const T& value)
{
  // True if NAN
  return value != value;
}

template <typename T>
bool is_valid(const T& value)
{
  return !is_infinite(value) && !is_nan(value);
}
#endif
