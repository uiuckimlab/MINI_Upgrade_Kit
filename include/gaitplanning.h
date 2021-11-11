//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: gaitplanning.h
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 04-Nov-2021 23:56:10
//

#ifndef GAITPLANNING_H
#define GAITPLANNING_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void gaitplanning(double x_size, double y_size, double t_step,
                         double n_step, double ds_time,
                         coder::array<double, 1U> &com_x,
                         coder::array<double, 1U> &com_y,
                         coder::array<double, 1U> &x_amp,
                         coder::array<double, 1U> &y_amp);
namespace coder
{
    double sum(const ::coder::array<double, 1U> &x);

}

#endif
//
// File trailer for gaitplanning.h
//
// [EOF]
//
