//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Foot_planning.h
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 05-Nov-2021 00:16:17
//

#ifndef FOOT_PLANNING_H
#define FOOT_PLANNING_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void Foot_planning(double b_time, double t_step,
                          coder::array<double, 1U> &x_amp,
                          const coder::array<double, 1U> &y_amp,
                          const coder::array<double, 1U> &com_x_out,
                          const coder::array<double, 1U> &com_y_out,
                          double ds_time, double n_step, double z_h, double z0,
                          double x0, double b_y0, double *x_RF, double *y_RF,
                          double *z_RF, double *x_LF, double *y_LF,
                          double *z_LF);
// Function Declarations
namespace coder
{
    double sum(const ::coder::array<double, 2U> &x);

}
#endif
//
// File trailer for Foot_planning.h
//
// [EOF]
//
