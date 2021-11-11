//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Foot_planning.cpp
//
// MATLAB Coder version            : 5.3
// C/C++ source code generated on  : 05-Nov-2021 00:16:17
//

// Include Files
#include "Foot_planning.h"
#include "mod.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>

namespace coder
{
  double sum(const ::coder::array<double, 2U> &x)
  {
    double y;
    if (x.size(1) == 0)
    {
      y = 0.0;
    }
    else
    {
      int firstBlockLength;
      int k;
      int lastBlockLength;
      int nblocks;
      if (x.size(1) <= 1024)
      {
        firstBlockLength = x.size(1);
        lastBlockLength = 0;
        nblocks = 1;
      }
      else
      {
        firstBlockLength = 1024;
        nblocks = x.size(1) / 1024;
        lastBlockLength = x.size(1) - (nblocks << 10);
        if (lastBlockLength > 0)
        {
          nblocks++;
        }
        else
        {
          lastBlockLength = 1024;
        }
      }
      y = x[0];
      for (k = 2; k <= firstBlockLength; k++)
      {
        y += x[k - 1];
      }
      for (int ib{2}; ib <= nblocks; ib++)
      {
        double bsum;
        int hi;
        firstBlockLength = (ib - 1) << 10;
        bsum = x[firstBlockLength];
        if (ib == nblocks)
        {
          hi = lastBlockLength;
        }
        else
        {
          hi = 1024;
        }
        for (k = 2; k <= hi; k++)
        {
          bsum += x[(firstBlockLength + k) - 1];
        }
        y += bsum;
      }
    }
    return y;
  }

} // namespace coder

// Function Declarations
static int div_s32_floor(int numerator, int denominator);

// Function Definitions
//
// Arguments    : int numerator
//                int denominator
// Return Type  : int
//
static int div_s32_floor(int numerator, int denominator)
{
  int quotient;
  if (denominator == 0)
  {
    if (numerator >= 0)
    {
      quotient = MAX_int32_T;
    }
    else
    {
      quotient = MIN_int32_T;
    }
  }
  else
  {
    unsigned int absDenominator;
    unsigned int absNumerator;
    unsigned int tempAbsQuotient;
    bool quotientNeedsNegation;
    if (numerator < 0)
    {
      absNumerator = ~static_cast<unsigned int>(numerator) + 1U;
    }
    else
    {
      absNumerator = static_cast<unsigned int>(numerator);
    }
    if (denominator < 0)
    {
      absDenominator = ~static_cast<unsigned int>(denominator) + 1U;
    }
    else
    {
      absDenominator = static_cast<unsigned int>(denominator);
    }
    quotientNeedsNegation = ((numerator < 0) != (denominator < 0));
    tempAbsQuotient = absNumerator / absDenominator;
    if (quotientNeedsNegation)
    {
      absNumerator %= absDenominator;
      if (absNumerator > 0U)
      {
        tempAbsQuotient++;
      }
      quotient = -static_cast<int>(tempAbsQuotient);
    }
    else
    {
      quotient = static_cast<int>(tempAbsQuotient);
    }
  }
  return quotient;
}

//
// Arguments    : double b_time
//                double t_step
//                coder::array<double, 2U> &x_amp
//                const coder::array<double, 2U> &y_amp
//                const coder::array<double, 2U> &com_x_out
//                const coder::array<double, 2U> &com_y_out
//                double ds_time
//                double n_step
//                double z_h
//                double z0
//                double x0
//                double b_y0
//                double *x_RF
//                double *y_RF
//                double *z_RF
//                double *x_LF
//                double *y_LF
//                double *z_LF
// Return Type  : void
//
void Foot_planning(double b_time, double t_step,
                   coder::array<double, 1U> &x_amp,
                   const coder::array<double, 1U> &y_amp,
                   const coder::array<double, 1U> &com_x_out,
                   const coder::array<double, 1U> &com_y_out, double ds_time,
                   double n_step, double z_h, double z0, double x0, double b_y0,
                   double *x_RF, double *y_RF, double *z_RF, double *x_LF,
                   double *y_LF, double *z_LF)
{
  coder::array<double, 2U> b_x_amp;
  double COM_ref_idx_0;
  double COM_ref_idx_1;
  double LF_flag;
  double t_cycle;
  double x;
  x = std::floor(b_time / 0.001);
  n_step = n_step + 4;
  if (x > (n_step - 1) * t_step / 0.001)
  {
    x = (n_step - 1) * t_step / 0.001;
  }

  x_amp[0] = x_amp[0] / 2.0;
  COM_ref_idx_0 = com_x_out[static_cast<int>(x + 1.0) - 1];
  COM_ref_idx_1 = com_y_out[static_cast<int>(x + 1.0) - 1];
  LF_flag = std::floor(b_time / t_step + 1.0E-6);
  if (LF_flag > n_step - 2.0)
  {
    LF_flag = n_step - 2.0;
  }
  t_cycle = coder::b_mod(b_time, t_step);
  *x_RF = 0.0;
  *x_LF = 0.0;
  *y_RF = 0.0;
  *y_LF = 0.0;

  if (LF_flag == 0.0)
  {
    *y_RF = -b_y0 - COM_ref_idx_1;
    *y_LF = b_y0 - COM_ref_idx_1;
    *x_LF = x0 - COM_ref_idx_0;
    *x_RF = *x_LF;
    *z_RF = z0;
    *z_LF = z0;
  }
  else if (LF_flag == 1.0)
  {
    double d;
    d = t_step * ds_time;
    if (b_time - t_step < d)
    {
      *x_LF = x0 - COM_ref_idx_0;
      *x_RF = *x_LF;
      *y_RF = -b_y0 - COM_ref_idx_1;
      *y_LF = b_y0 - COM_ref_idx_1;
      *z_LF = z0;
      *z_RF = z0;
    }
    else
    {
      double x_RF_tmp;
      LF_flag = t_step * (1.0 - ds_time);
      x = t_cycle - d;
      x_RF_tmp = 1.0 - std::cos(3.1415926535897931 * x / LF_flag);
      *x_RF = (x0 - com_x_out[static_cast<int>(std::floor(
                                  (t_step / 0.001 + d / 0.001) + 1.0)) -
                              1]) +
              x_amp[0] * (ds_time + 1.0) * x_RF_tmp / 2.0;
      *x_LF = x0 - COM_ref_idx_0;
      *y_RF = (-b_y0 - COM_ref_idx_1) + (b_y0 + y_amp[1]) * x_RF_tmp / 2.0;
      *y_LF = b_y0 - COM_ref_idx_1;
      *z_LF = z0;
      *z_RF = z0 + z_h * std::abs(std::sin(3.1415926535897931 / LF_flag * x));
    }
  }
  else if (LF_flag == 2.0)
  {
    if (b_time - t_step * 2.0 < t_step * ds_time)
    {
      *x_RF =
          ((x0 + x_amp[0] * (ds_time + 1.0)) -
           com_x_out[static_cast<int>(std::floor(
                         (t_step / 0.001 + ds_time * t_step / 0.001) + 1.0)) -
                     1]) -
          (COM_ref_idx_0 -
           com_x_out[static_cast<int>(std::floor(2.0 * t_step / 0.001) + 1.0) -
                     1]);
      *x_LF = x0 - COM_ref_idx_0;
      *y_RF = (-b_y0 - COM_ref_idx_1) + (b_y0 + y_amp[1]);
      *y_LF = b_y0 - COM_ref_idx_1;
      *z_RF = z0;
      *z_LF = z0;
    }
    else
    {
      double x_RF_tmp;
      x_RF_tmp = 2.0 * t_step / 0.001;
      *x_RF =
          ((x0 + x_amp[0] * (ds_time + 1.0)) -
           com_x_out[static_cast<int>(std::floor(
                         (t_step / 0.001 + ds_time * t_step / 0.001) + 1.0)) -
                     1]) -
          (COM_ref_idx_0 -
           com_x_out[static_cast<int>(std::floor(x_RF_tmp) + 1.0) - 1]);
      LF_flag = 3.1415926535897931 * (t_cycle - t_step * ds_time) /
                (t_step * (1.0 - ds_time));
      x = 1.0 - std::cos(LF_flag);
      *x_LF =
          (x0 - com_x_out[static_cast<int>(std::floor(
                              (x_RF_tmp + ds_time * t_step / 0.001) + 1.0)) -
                          1]) +
          x_amp[1] * (ds_time + 1.0) * x / 2.0;
      *y_RF = (-b_y0 - COM_ref_idx_1) + (b_y0 + y_amp[1]);
      *y_LF = (b_y0 - COM_ref_idx_1) + (-b_y0 + y_amp[2]) * x / 2.0;
      *z_RF = z0;
      *z_LF = z0 + z_h * std::abs(std::sin(LF_flag));
    }
  }
  else if (LF_flag < n_step - 3.0)
  {
    double x_RF_tmp;
    int i;
    int kk;
    if (coder::b_mod(LF_flag) == 1.0)
    {
      i = static_cast<int>(LF_flag - 1.0);
      for (kk = 0; kk < i; kk++)
      {
        if (coder::b_mod(static_cast<double>(kk) + 1.0) == 1.0)
        {
          x_RF_tmp = ds_time * t_step / 0.001;
          *x_RF -= com_x_out[static_cast<int>(
                                 std::floor((static_cast<double>(kk) + 1.0) *
                                                t_step / 0.001 +
                                            x_RF_tmp) +
                                 1.0) -
                             1];
          *x_LF -=
              com_x_out[static_cast<int>(
                            std::floor(((static_cast<double>(kk) + 1.0) + 1.0) *
                                           t_step / 0.001 +
                                       x_RF_tmp) +
                            1.0) -
                        1];
        }
        else
        {
          *x_RF += com_x_out[static_cast<int>(
                                 std::floor((static_cast<double>(kk) + 1.0) *
                                            t_step / 0.001) +
                                 1.0) -
                             1];
          *x_LF +=
              com_x_out[static_cast<int>(
                            std::floor(((static_cast<double>(kk) + 1.0) + 1.0) *
                                       t_step / 0.001) +
                            1.0) -
                        1];
        }
      }
    }
    else
    {
      i = static_cast<int>(LF_flag - 2.0);
      for (kk = 0; kk < i; kk++)
      {
        if (coder::b_mod(static_cast<double>(kk) + 1.0) == 1.0)
        {
          x_RF_tmp = ds_time * t_step / 0.001;
          *x_RF -= com_x_out[static_cast<int>(
                                 std::floor((static_cast<double>(kk) + 1.0) *
                                                t_step / 0.001 +
                                            x_RF_tmp) +
                                 1.0) -
                             1];
          *x_LF -=
              com_x_out[static_cast<int>(
                            std::floor(((static_cast<double>(kk) + 1.0) + 1.0) *
                                           t_step / 0.001 +
                                       x_RF_tmp) +
                            1.0) -
                        1];
        }
        else
        {
          *x_RF += com_x_out[static_cast<int>(
                                 std::floor((static_cast<double>(kk) + 1.0) *
                                            t_step / 0.001) +
                                 1.0) -
                             1];
          *x_LF +=
              com_x_out[static_cast<int>(
                            std::floor(((static_cast<double>(kk) + 1.0) + 1.0) *
                                       t_step / 0.001) +
                            1.0) -
                        1];
        }
      }
    }
    i = static_cast<int>(LF_flag - 2.0);
    for (kk = 0; kk < i; kk++)
    {
      if (kk + 1U == 1U)
      {
        *y_RF += y_amp[1];
        *y_LF += y_amp[2];
      }
      else if (coder::b_mod(static_cast<double>(kk) + 1.0) == 0.0)
      {
        *y_RF -= y_amp[kk] - y_amp[kk + 2];
      }
      else
      {
        *y_LF -= y_amp[kk] - y_amp[kk + 2];
      }
    }
    if (coder::b_mod(LF_flag) == 1.0)
    {
      if (b_time - t_step * LF_flag < t_step * ds_time)
      {
        int i1;
        int i2;
        //  Right Swing
        if (1.0 > LF_flag - 1.0)
        {
          i = 1;
          i1 = -1;
        }
        else
        {
          i = 2;
          i1 = static_cast<int>(LF_flag - 1.0) - 1;
        }
        kk = div_s32_floor(i1, i);
        b_x_amp.set_size(1, kk + 1);
        for (i1 = 0; i1 <= kk; i1++)
        {
          b_x_amp[i1] = x_amp[i * i1] * (ds_time + 1.0);
        }
        *x_RF = ((x0 + *x_RF) + coder::sum(b_x_amp)) - COM_ref_idx_0;
        *y_RF -= COM_ref_idx_1;
        *z_RF = z0;
        //  Left Stand
        if (2.0 > LF_flag - 1.0)
        {
          i = 0;
          i1 = 1;
          i2 = -1;
        }
        else
        {
          i = 1;
          i1 = 2;
          i2 = static_cast<int>(LF_flag - 1.0) - 1;
        }
        kk = div_s32_floor(i2 - i, i1);
        b_x_amp.set_size(1, kk + 1);
        for (i2 = 0; i2 <= kk; i2++)
        {
          b_x_amp[i2] = x_amp[i + i1 * i2] * (ds_time + 1.0);
        }
        *x_LF = ((x0 + *x_LF) + coder::sum(b_x_amp)) - COM_ref_idx_0;
        *y_LF -= COM_ref_idx_1;
        *z_LF = z0;
      }
      else
      {
        int i1;
        int i2;
        //  Right Swing
        if (1.0 > LF_flag - 1.0)
        {
          i = 1;
          i1 = -1;
        }
        else
        {
          i = 2;
          i1 = static_cast<int>(LF_flag - 1.0) - 1;
        }
        kk = div_s32_floor(i1, i);
        b_x_amp.set_size(1, kk + 1);
        for (i1 = 0; i1 <= kk; i1++)
        {
          b_x_amp[i1] = x_amp[i * i1] * (ds_time + 1.0);
        }
        x_RF_tmp =
            1.0 - std::cos(3.1415926535897931 * (t_cycle - t_step * ds_time) /
                           (t_step * (1.0 - ds_time)));
        *x_RF =
            (((x0 + *x_RF) + coder::sum(b_x_amp)) -
             com_x_out[static_cast<int>(std::floor((LF_flag * t_step / 0.001 +
                                                    ds_time * t_step / 0.001) +
                                                   1.0)) -
                       1]) +
            x_amp[static_cast<int>(LF_flag) - 1] * (ds_time + 1.0) * x_RF_tmp /
                2.0;
        *y_RF = (*y_RF - COM_ref_idx_1) +
                (y_amp[static_cast<int>(static_cast<unsigned int>(LF_flag))] -
                 y_amp[static_cast<int>(LF_flag) - 2]) *
                    x_RF_tmp / 2.0;
        *z_RF = z0 + z_h * std::abs(std::sin(3.1415926535897931 *
                                             (t_cycle - t_step * ds_time) /
                                             (t_step * (1.0 - ds_time))));
        //  Left Stand
        if (2.0 > LF_flag - 1.0)
        {
          i = 0;
          i1 = 1;
          i2 = -1;
        }
        else
        {
          i = 1;
          i1 = 2;
          i2 = static_cast<int>(LF_flag) - 2;
        }
        kk = div_s32_floor(i2 - i, i1);
        b_x_amp.set_size(1, kk + 1);
        for (i2 = 0; i2 <= kk; i2++)
        {
          b_x_amp[i2] = x_amp[i + i1 * i2] * (ds_time + 1.0);
        }
        *x_LF = ((x0 + *x_LF) + coder::sum(b_x_amp)) - COM_ref_idx_0;
        *y_LF -= COM_ref_idx_1;
        *z_LF = z0;
      }
    }
    else if (b_time - t_step * LF_flag < t_step * ds_time)
    {
      int i1;
      int i2;
      //  Right Swing
      if (1.0 > LF_flag - 1.0)
      {
        i = 1;
        i1 = -1;
      }
      else
      {
        i = 2;
        i1 = static_cast<int>(LF_flag - 1.0) - 1;
      }
      kk = div_s32_floor(i1, i);
      b_x_amp.set_size(1, kk + 1);
      for (i1 = 0; i1 <= kk; i1++)
      {
        b_x_amp[i1] = x_amp[i * i1] * (ds_time + 1.0);
      }
      *x_RF = ((((x0 + *x_RF) + coder::sum(b_x_amp)) - COM_ref_idx_0) -
               com_x_out[static_cast<int>(
                             std::floor((LF_flag - 1.0) * t_step / 0.001 +
                                        t_step * ds_time / 0.001) +
                             1.0) -
                         1]) +
              com_x_out[static_cast<int>(std::floor(LF_flag * t_step / 0.001) +
                                         1.0) -
                        1];
      *y_RF -= COM_ref_idx_1;
      *z_RF = z0;
      //  Left Stand
      if (2.0 > LF_flag - 1.0)
      {
        i = 0;
        i1 = 1;
        i2 = -1;
      }
      else
      {
        i = 1;
        i1 = 2;
        i2 = static_cast<int>(LF_flag - 1.0) - 1;
      }
      kk = div_s32_floor(i2 - i, i1);
      b_x_amp.set_size(1, kk + 1);
      for (i2 = 0; i2 <= kk; i2++)
      {
        b_x_amp[i2] = x_amp[i + i1 * i2] * (ds_time + 1.0);
      }
      *x_LF = ((x0 + *x_LF) + coder::sum(b_x_amp)) - COM_ref_idx_0;
      *y_LF -= COM_ref_idx_1;
      *z_LF = z0;
    }
    else
    {
      int i1;
      int i2;
      //  Right Swing
      if (1.0 > LF_flag - 1.0)
      {
        i = 1;
        i1 = -1;
      }
      else
      {
        i = 2;
        i1 = static_cast<int>(LF_flag - 1.0) - 1;
      }
      kk = div_s32_floor(i1, i);
      b_x_amp.set_size(1, kk + 1);
      for (i1 = 0; i1 <= kk; i1++)
      {
        b_x_amp[i1] = x_amp[i * i1] * (ds_time + 1.0);
      }
      *x_RF = ((((x0 + *x_RF) + coder::sum(b_x_amp)) - COM_ref_idx_0) -
               com_x_out[static_cast<int>(
                             std::floor((LF_flag - 1.0) * t_step / 0.001 +
                                        t_step * ds_time / 0.001) +
                             1.0) -
                         1]) +
              com_x_out[static_cast<int>(std::floor(LF_flag * t_step / 0.001) +
                                         1.0) -
                        1];
      *y_RF -= COM_ref_idx_1;
      *z_RF = z0;
      //  Left Stand
      if (2.0 > LF_flag - 1.0)
      {
        i = 0;
        i1 = 1;
        i2 = -1;
      }
      else
      {
        i = 1;
        i1 = 2;
        i2 = static_cast<int>(LF_flag - 1.0) - 1;
      }
      kk = div_s32_floor(i2 - i, i1);
      b_x_amp.set_size(1, kk + 1);
      for (i2 = 0; i2 <= kk; i2++)
      {
        b_x_amp[i2] = x_amp[i + i1 * i2] * (ds_time + 1.0);
      }
      *x_LF =
          (((x0 + *x_LF) + coder::sum(b_x_amp)) -
           com_x_out[static_cast<int>(std::floor(
                         (LF_flag * t_step / 0.001 + ds_time * t_step / 0.001) +
                         1.0)) -
                     1]) +
          x_amp[static_cast<int>(LF_flag) - 1] * (ds_time + 1.0) *
              (1.0 -
               std::cos(3.1415926535897931 * (t_cycle - t_step * ds_time) /
                        (t_step * (1.0 - ds_time)))) /
              2.0;
      *y_LF = (*y_LF - COM_ref_idx_1) +
              (y_amp[static_cast<int>(static_cast<unsigned int>(LF_flag))] -
               y_amp[static_cast<int>(LF_flag) - 2]) *
                  (1.0 -
                   std::cos(3.1415926535897931 * (t_cycle - t_step * ds_time) /
                            (t_step * (1.0 - ds_time)))) /
                  2.0;
      *z_LF = z0 + z_h * std::abs(std::sin(3.1415926535897931 *
                                           (t_cycle - t_step * ds_time) /
                                           (t_step * (1.0 - ds_time))));
    }
  }
  else if (LF_flag == n_step - 3.0)
  {
    double d;
    double x_RF_tmp;
    int i;
    int kk;
    i = static_cast<int>(LF_flag - 1.0);
    for (kk = 0; kk < i; kk++)
    {
      if (coder::b_mod(static_cast<double>(kk) + 1.0) == 1.0)
      {
        x_RF_tmp = ds_time * t_step / 0.001;
        *x_RF -= com_x_out[static_cast<int>(
                               std::floor((static_cast<double>(kk) + 1.0) *
                                              t_step / 0.001 +
                                          x_RF_tmp) +
                               1.0) -
                           1];
        *x_LF -=
            com_x_out[static_cast<int>(
                          std::floor(((static_cast<double>(kk) + 1.0) + 1.0) *
                                         t_step / 0.001 +
                                     x_RF_tmp) +
                          1.0) -
                      1];
      }
      else
      {
        *x_RF += com_x_out[static_cast<int>(
                               std::floor((static_cast<double>(kk) + 1.0) *
                                          t_step / 0.001) +
                               1.0) -
                           1];
        *x_LF +=
            com_x_out[static_cast<int>(
                          std::floor(((static_cast<double>(kk) + 1.0) + 1.0) *
                                     t_step / 0.001) +
                          1.0) -
                      1];
      }
    }
    i = static_cast<int>(LF_flag - 2.0);
    for (kk = 0; kk < i; kk++)
    {
      if (kk + 1U == 1U)
      {
        *y_RF += y_amp[1];
        *y_LF += y_amp[2];
      }
      else if (coder::b_mod(static_cast<double>(kk) + 1.0) == 0.0)
      {
        *y_RF -= y_amp[kk] - y_amp[kk + 2];
      }
      else
      {
        *y_LF -= y_amp[kk] - y_amp[kk + 2];
      }
    }
    d = t_step * LF_flag;
    if (b_time - d < t_step * ds_time)
    {
      int i1;
      int i2;
      //  Right Swing
      if (1.0 > LF_flag - 1.0)
      {
        i = 1;
        i1 = -1;
      }
      else
      {
        i = 2;
        i1 = static_cast<int>(LF_flag - 1.0) - 1;
      }
      kk = div_s32_floor(i1, i);
      b_x_amp.set_size(1, kk + 1);
      for (i1 = 0; i1 <= kk; i1++)
      {
        b_x_amp[i1] = x_amp[i * i1] * (ds_time + 1.0);
      }
      *x_RF = ((x0 + *x_RF) + coder::sum(b_x_amp)) - COM_ref_idx_0;
      *y_RF -= COM_ref_idx_1;
      *z_RF = z0;
      //  Left Stand
      if (2.0 > LF_flag - 1.0)
      {
        i = 0;
        i1 = 1;
        i2 = -1;
      }
      else
      {
        i = 1;
        i1 = 2;
        i2 = static_cast<int>(LF_flag - 1.0) - 1;
      }
      kk = div_s32_floor(i2 - i, i1);
      b_x_amp.set_size(1, kk + 1);
      for (i2 = 0; i2 <= kk; i2++)
      {
        b_x_amp[i2] = x_amp[i + i1 * i2] * (ds_time + 1.0);
      }
      *x_LF = ((x0 + *x_LF) + coder::sum(b_x_amp)) - COM_ref_idx_0;
      *y_LF -= COM_ref_idx_1;
      *z_LF = z0;
    }
    else
    {
      int i1;
      int i2;
      //  Right Swing
      if (1.0 > LF_flag - 1.0)
      {
        i = 1;
        i1 = -1;
      }
      else
      {
        i = 2;
        i1 = static_cast<int>(LF_flag - 1.0) - 1;
      }
      kk = div_s32_floor(i1, i);
      b_x_amp.set_size(1, kk + 1);
      for (i1 = 0; i1 <= kk; i1++)
      {
        b_x_amp[i1] = x_amp[i * i1] * (ds_time + 1.0);
      }
      x_RF_tmp = com_x_out[static_cast<int>(std::floor(
                               (d / 0.001 + ds_time * t_step / 0.001) + 1.0)) -
                           1];
      x = 1.0 - std::cos(3.1415926535897931 * (t_cycle - t_step * ds_time) /
                             (t_step * (1.0 - ds_time)) +
                         3.1415926535897931);
      *x_RF = x0 + ((*x_RF + coder::sum(b_x_amp)) - x_RF_tmp) * x / 2.0;
      *y_RF = (*y_RF - COM_ref_idx_1) +
              (y_amp[static_cast<int>(LF_flag + 1.0) - 1] -
               y_amp[static_cast<int>(LF_flag - 1.0) - 1]) *
                  (1.0 -
                   std::cos(3.1415926535897931 * (t_cycle - t_step * ds_time) /
                            (t_step * (1.0 - ds_time)))) /
                  2.0;
      *z_RF = z0 + z_h * std::abs(std::sin(3.1415926535897931 *
                                           (t_cycle - t_step * ds_time) /
                                           (t_step * (1.0 - ds_time))));
      //  Left Stand
      if (2.0 > LF_flag - 1.0)
      {
        i = 0;
        i1 = 1;
        i2 = -1;
      }
      else
      {
        i = 1;
        i1 = 2;
        i2 = static_cast<int>(LF_flag - 1.0) - 1;
      }
      kk = div_s32_floor(i2 - i, i1);
      b_x_amp.set_size(1, kk + 1);
      for (i2 = 0; i2 <= kk; i2++)
      {
        b_x_amp[i2] = x_amp[i + i1 * i2] * (ds_time + 1.0);
      }
      *x_LF = x0 + ((*x_LF + coder::sum(b_x_amp)) - x_RF_tmp) * x / 2.0;
      *y_LF -= COM_ref_idx_1;
      *z_LF = z0;
    }
  }
  else
  {
    int i;
    if (LF_flag > n_step - 2.0)
    {
      LF_flag = n_step - 2.0;
    }
    i = static_cast<int>(LF_flag - 2.0);
    for (int kk{0}; kk < i; kk++)
    {
      if (kk + 1U == 1U)
      {
        *y_RF += y_amp[1];
        *y_LF += y_amp[2];
      }
      else if (coder::b_mod(static_cast<double>(kk) + 1.0) == 0.0)
      {
        *y_RF -= y_amp[kk] - y_amp[kk + 2];
      }
      else
      {
        *y_LF -= y_amp[kk] - y_amp[kk + 2];
      }
    }
    //  Right Swing
    *x_RF = x0;
    *y_RF -= COM_ref_idx_1;
    *z_RF = z0;
    //  Left Stand
    *x_LF = x0;
    *y_LF -= COM_ref_idx_1;
    *z_LF = z0;
  }
}

//
// File trailer for Foot_planning.cpp
//
// [EOF]
//
