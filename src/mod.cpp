//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mod.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 05-Nov-2021 00:16:17
//

// Include Files
#include "mod.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
//
// Arguments    : double x
//                double y
// Return Type  : double
//
namespace coder {
double b_mod(double x, double y)
{
  double r;
  r = x;
  if (y == 0.0) {
    if (x == 0.0) {
      r = y;
    }
  } else if (std::isnan(x) || std::isnan(y) || std::isinf(x)) {
    r = rtNaN;
  } else if (x == 0.0) {
    r = 0.0 / y;
  } else if (std::isinf(y)) {
    if ((y < 0.0) != (x < 0.0)) {
      r = y;
    }
  } else {
    bool rEQ0;
    r = std::fmod(x, y);
    rEQ0 = (r == 0.0);
    if ((!rEQ0) && (y > std::floor(y))) {
      double q;
      q = std::abs(x / y);
      rEQ0 = !(std::abs(q - std::floor(q + 0.5)) > 2.2204460492503131E-16 * q);
    }
    if (rEQ0) {
      r = y * 0.0;
    } else if ((x < 0.0) != (y < 0.0)) {
      r += y;
    }
  }
  return r;
}

//
// Arguments    : double x
// Return Type  : double
//
double b_mod(double x)
{
  double r;
  if (std::isnan(x) || std::isinf(x)) {
    r = rtNaN;
  } else if (x == 0.0) {
    r = 0.0;
  } else {
    r = std::fmod(x, 2.0);
    if (r == 0.0) {
      r = 0.0;
    } else if (x < 0.0) {
      r += 2.0;
    }
  }
  return r;
}

} // namespace coder

//
// File trailer for mod.cpp
//
// [EOF]
//
