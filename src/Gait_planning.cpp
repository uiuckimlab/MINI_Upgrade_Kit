

// Include Files
#include "gaitplanning.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>

namespace coder
{
    double sum(const ::coder::array<double, 1U> &x)
    {
        double y;
        if (x.size(0) == 0)
        {
            y = 0.0;
        }
        else
        {
            int firstBlockLength;
            int k;
            int lastBlockLength;
            int nblocks;
            if (x.size(0) <= 1024)
            {
                firstBlockLength = x.size(0);
                lastBlockLength = 0;
                nblocks = 1;
            }
            else
            {
                firstBlockLength = 1024;
                nblocks = x.size(0) / 1024;
                lastBlockLength = x.size(0) - (nblocks << 10);
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
static double rt_powd_snf(double u0, double u1);

// Function Definitions
//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_powd_snf(double u0, double u1)
{
    double y;
    if (std::isnan(u0) || std::isnan(u1))
    {
        y = rtNaN;
    }
    else
    {
        double d;
        double d1;
        d = std::abs(u0);
        d1 = std::abs(u1);
        if (std::isinf(u1))
        {
            if (d == 1.0)
            {
                y = 1.0;
            }
            else if (d > 1.0)
            {
                if (u1 > 0.0)
                {
                    y = rtInf;
                }
                else
                {
                    y = 0.0;
                }
            }
            else if (u1 > 0.0)
            {
                y = 0.0;
            }
            else
            {
                y = rtInf;
            }
        }
        else if (d1 == 0.0)
        {
            y = 1.0;
        }
        else if (d1 == 1.0)
        {
            if (u1 > 0.0)
            {
                y = u0;
            }
            else
            {
                y = 1.0 / u0;
            }
        }
        else if (u1 == 2.0)
        {
            y = u0 * u0;
        }
        else if ((u1 == 0.5) && (u0 >= 0.0))
        {
            y = std::sqrt(u0);
        }
        else if ((u0 < 0.0) && (u1 > std::floor(u1)))
        {
            y = rtNaN;
        }
        else
        {
            y = std::pow(u0, u1);
        }
    }
    return y;
}

//
// t_step = 1; % timing for one step (s)
//  n_step = 6; % Always even number!!!!
//  ds_time = 0.4;
//
// Arguments    : double x_size
//                double y_size
//                double t_step
//                double n_step
//                double ds_time
//                coder::array<double, 1U> &com_x
//                coder::array<double, 1U> &com_y
//                coder::array<double, 1U> &x_amp
//                coder::array<double, 1U> &y_amp
// Return Type  : void
//
void gaitplanning(double x_size, double y_size, double t_step, double n_step,
                  double ds_time, coder::array<double, 1U> &com_x,
                  coder::array<double, 1U> &com_y,
                  coder::array<double, 1U> &x_amp,
                  coder::array<double, 1U> &y_amp)
{
    static const double dv1[16]{
        1.2476410448674278, 56.7058241861882, 6.2942497597623728,
        -0.0018141917486719054, -3.3326904209055882E-6, 0.9992503251359105,
        0.00091530703966317723, 3.577527520227077E-7, -0.0099980712627168455,
        -2.249024592268376, 0.745921118989531, 0.00057325825606812286,
        -19.996142525433584, -4498.0491845367542, -508.15776202093957,
        0.14651651213624217};
    static const double b_a[9]{1.0, 0.0, 0.0, 0.001, 1.0,
                               0.0, 5.0E-7, 0.001, 1.0};
    static const double c_a[3]{1.6666666666666669E-10, 5.0E-7, 0.001};
    coder::array<double, 2U> Gd;
    coder::array<double, 2U> zmp_x_ref;
    coder::array<double, 2U> zmp_y_ref;
    coder::array<double, 1U> b_x_amp;
    coder::array<double, 1U> c_x_amp;
    coder::array<double, 1U> d_x_amp;
    double dv[4];
    double a[3];
    double x_x[3];
    double c_i;
    double d;
    double j;
    double k;
    double ndbl;
    double q;
    double t_calc_tmp;
    double t_preview;
    double x_y_idx_0;
    double x_y_idx_1;
    double x_y_idx_2;
    int b_i;
    int i;
    int i1;
    int loop_ub;
    int loop_ub_tmp;
    int nm1d2;
    double y0 = 0.033;
    //  LIPM height
    //  delta time (s)
    n_step += 4.0;
    t_preview = t_step * 2.0;
    //  timing for preview (s)
    t_calc_tmp = n_step * t_step;
    //  Automatic calculation for simulation time
    loop_ub_tmp = static_cast<int>(n_step);
    x_amp.set_size(loop_ub_tmp);
    y_amp.set_size(loop_ub_tmp);
    for (i = 0; i < loop_ub_tmp; i++)
    {
        x_amp[i] = 0.0;
        y_amp[i] = 0.0;
    }
    i = static_cast<int>(n_step - 2.0);
    for (b_i = 0; b_i < i; b_i++)
    {
        if (static_cast<double>(b_i) + 1.0 < n_step - 4.0)
        {
            x_amp[b_i] = x_size * 0.001;
            //  Step stride
            y_amp[b_i] = y_size * 0.001 *
                         rt_powd_snf(-1.0, (static_cast<double>(b_i) + 1.0) - 1.0);
            //  Lateral Step size
        }
        else if (static_cast<double>(b_i) + 1.0 == n_step - 4.0)
        {
            x_amp[b_i] = x_amp[b_i - 1] / 2.0;
            //  Half step for the last step
            y_amp[b_i] =
                0.03 * rt_powd_snf(-1.0, (static_cast<double>(b_i) + 1.0) - 1.0);
        }
        else
        {
            x_amp[b_i] = 0.0;
            y_amp[b_i] =
                y0 * rt_powd_snf(-1.0, (static_cast<double>(b_i) + 1.0) - 1.0);
            // Go back to the initial pose
        }
    }
    y_amp[0] = y0;
    k = 0.0;
    d = t_calc_tmp + t_preview * 2.0;
    loop_ub = static_cast<int>(d / 0.001 + 1.0);
    zmp_x_ref.set_size(1, loop_ub);
    zmp_y_ref.set_size(1, loop_ub);
    for (i = 0; i < loop_ub; i++)
    {
        zmp_x_ref[i] = 0.0;
        zmp_y_ref[i] = 0.0;
    }
    j = t_step / 0.001;
    i = static_cast<int>((d + (0.001 - t_step)) / 0.001);
    for (b_i = 0; b_i < i; b_i++)
    {
        c_i = t_step + static_cast<double>(b_i) * 0.001;
        if (c_i < (n_step - 2.0) * t_step)
        {
            if (k == 0.0)
            {
                loop_ub_tmp =
                    static_cast<int>(j + (static_cast<double>(b_i) + 1.0)) - 1;
                zmp_x_ref[loop_ub_tmp] = 0.0;
                zmp_y_ref[loop_ub_tmp] = y_amp[0];
            }
            else
            {
                if (1.0 > k - 1.0)
                {
                    loop_ub = 0;
                }
                else
                {
                    loop_ub = static_cast<int>(k - 1.0);
                }
                if (1.0 > k - 2.0)
                {
                    loop_ub_tmp = 0;
                    nm1d2 = 0;
                }
                else
                {
                    loop_ub_tmp = static_cast<int>(k - 2.0);
                    nm1d2 = static_cast<int>(k - 2.0);
                }
                b_x_amp.set_size(loop_ub);
                for (i1 = 0; i1 < loop_ub; i1++)
                {
                    b_x_amp[i1] = x_amp[i1];
                }
                c_x_amp.set_size(loop_ub_tmp);
                for (i1 = 0; i1 < loop_ub_tmp; i1++)
                {
                    c_x_amp[i1] = x_amp[i1];
                }
                d_x_amp.set_size(nm1d2);
                for (i1 = 0; i1 < nm1d2; i1++)
                {
                    d_x_amp[i1] = x_amp[i1];
                }
                loop_ub_tmp =
                    static_cast<int>(j + (static_cast<double>(b_i) + 1.0)) - 1;
                zmp_x_ref[loop_ub_tmp] = (coder::sum(b_x_amp) - coder::sum(c_x_amp)) /
                                             (t_step / ds_time) * (c_i - t_step * k) +
                                         coder::sum(d_x_amp);
                zmp_y_ref[loop_ub_tmp] = y_amp[static_cast<int>(k) - 1];
            }
        }
        else
        {
            if (1.0 > n_step - 4.0)
            {
                loop_ub = 0;
            }
            else
            {
                loop_ub = static_cast<int>(n_step - 4.0);
            }
            b_x_amp.set_size(loop_ub);
            for (i1 = 0; i1 < loop_ub; i1++)
            {
                b_x_amp[i1] = x_amp[i1];
            }
            loop_ub_tmp = static_cast<int>(j + (static_cast<double>(b_i) + 1.0)) - 1;
            zmp_x_ref[loop_ub_tmp] = coder::sum(b_x_amp);
            zmp_y_ref[loop_ub_tmp] = 0.0;
        }
        if (c_i != 0.0)
        {
            ndbl = c_i;
            if (t_step == 0.0)
            {
                if (c_i == 0.0)
                {
                    ndbl = t_step;
                }
            }
            else if (std::isnan(c_i) || std::isnan(t_step) || std::isinf(c_i))
            {
                ndbl = rtNaN;
            }
            else if (c_i == 0.0)
            {
                ndbl = 0.0 / t_step;
            }
            else if (std::isinf(t_step))
            {
                if ((t_step < 0.0) != (c_i < 0.0))
                {
                    ndbl = t_step;
                }
            }
            else
            {
                bool rEQ0;
                ndbl = std::fmod(c_i, t_step);
                rEQ0 = (ndbl == 0.0);
                if ((!rEQ0) && (t_step > std::floor(t_step)))
                {
                    q = std::abs(c_i / t_step);
                    rEQ0 =
                        !(std::abs(q - std::floor(q + 0.5)) > 2.2204460492503131E-16 * q);
                }
                if (rEQ0)
                {
                    ndbl = t_step * 0.0;
                }
                else if ((c_i < 0.0) != (t_step < 0.0))
                {
                    ndbl += t_step;
                }
            }
            if (ndbl == 0.0)
            {
                k++;
            }
        }
    }
    loop_ub = x_amp.size(0);
    for (i = 0; i < loop_ub; i++)
    {
        x_amp[i] = x_amp[i] * 1000.0;
    }
    loop_ub = y_amp.size(0);
    for (i = 0; i < loop_ub; i++)
    {
        y_amp[i] = y_amp[i] * 1000.0;
    }
    //
    //  Calculating preview gain
    if (std::isnan(t_preview))
    {
        nm1d2 = 1;
    }
    else if (t_preview < 0.0)
    {
        nm1d2 = 0;
    }
    else if (std::isinf(t_preview) && (0.0 == t_preview))
    {
        nm1d2 = 1;
    }
    else
    {
        ndbl = std::floor(t_preview / 0.001 + 0.5);
        j = ndbl * 0.001;
        q = j - t_preview;
        if (std::abs(q) < 4.4408920985006262E-16 * std::fmax(0.0, t_preview))
        {
            ndbl++;
            j = t_preview;
        }
        else if (q > 0.0)
        {
            j = (ndbl - 1.0) * 0.001;
        }
        else
        {
            ndbl++;
        }
        if (ndbl >= 0.0)
        {
            loop_ub_tmp = static_cast<int>(ndbl);
        }
        else
        {
            loop_ub_tmp = 0;
        }
        Gd.set_size(1, loop_ub_tmp);
        if ((loop_ub_tmp > 0) && (loop_ub_tmp > 1))
        {
            Gd[loop_ub_tmp - 1] = j;
            nm1d2 = (loop_ub_tmp - 1) / 2;
            for (loop_ub = 0; loop_ub <= nm1d2 - 2; loop_ub++)
            {
                ndbl = (static_cast<double>(loop_ub) + 1.0) * 0.001;
                Gd[loop_ub + 1] = ndbl;
                Gd[(loop_ub_tmp - loop_ub) - 2] = j - ndbl;
            }
            if (nm1d2 << 1 == loop_ub_tmp - 1)
            {
                Gd[nm1d2] = j / 2.0;
            }
            else
            {
                ndbl = static_cast<double>(nm1d2) * 0.001;
                Gd[nm1d2] = ndbl;
                Gd[nm1d2 + 1] = j - ndbl;
            }
        }
        nm1d2 = Gd.size(1);
    }
    Gd.set_size(1, nm1d2);
    for (i = 0; i < nm1d2; i++)
    {
        Gd[i] = 0.0;
    }
    Gd[0] = -19996.142524325;
    ndbl = -22394.584470965594;
    q = -2.518784364034222E+6;
    j = -281584.20169819141;
    k = -142.37191770763343;
    for (b_i = 0; b_i <= nm1d2 - 2; b_i++)
    {
        Gd[b_i + 1] = ((-49.518656578554072 * ndbl + 0.00066640953047104141 * q) +
                       1.9992285914131238 * j) +
                      3998.4571828262483 * k;
        for (i = 0; i < 4; i++)
        {
            dv[i] =
                ((dv1[i] * ndbl + dv1[i + 4] * q) + dv1[i + 8] * j) + dv1[i + 12] * k;
        }
        ndbl = dv[0];
        q = dv[1];
        j = dv[2];
        k = dv[3];
    }
    //  CoM x, CoM y
    x_x[0] = 0.0;
    x_y_idx_0 = 0.0;
    x_x[1] = 0.0;
    x_y_idx_1 = 0.0;
    x_x[2] = 0.0;
    x_y_idx_2 = 0.0;
    //  Variable for plotting
    c_i = 1.0;
    loop_ub = static_cast<int>((t_calc_tmp - 0.001) / 0.001 + 1.0);
    com_x.set_size(loop_ub);
    com_y.set_size(loop_ub);
    for (i = 0; i < loop_ub; i++)
    {
        com_x[i] = 0.0;
        com_y[i] = 0.0;
    }
    i = static_cast<int>(((t_calc_tmp - 0.001) + 0.001) / 0.001);
    for (b_i = 0; b_i < i; b_i++)
    {
        if (static_cast<double>(b_i) * 0.001 == 0.0)
        {
            //  Output ZMP
            ndbl = 0.0;
            q = 0.0;
            for (loop_ub = 0; loop_ub < nm1d2; loop_ub++)
            {
                d = Gd[loop_ub];
                loop_ub_tmp =
                    static_cast<int>(static_cast<unsigned int>(c_i) + loop_ub);
                ndbl += d * zmp_x_ref[loop_ub_tmp];
                q += d * zmp_y_ref[loop_ub_tmp];
            }
            d = x_x[0];
            j = x_x[1];
            t_preview = x_x[2];
            k = (-19996.142524325 *
                     (zmp_x_ref[static_cast<int>(c_i) - 1] -
                      ((x_x[0] + 0.0 * x_x[1]) + -0.012384607543323139 * x_x[2])) -
                 ((4.49804918441121E+6 * x_x[0] + 508157.762006947 * x_x[1]) +
                  853.483487856589 * x_x[2])) -
                ndbl;
            ndbl =
                (-19996.142524325 * (zmp_y_ref[static_cast<int>(c_i) - 1] -
                                     ((x_y_idx_0 + 0.0 * x_y_idx_1) +
                                      -0.012384607543323139 * x_y_idx_2)) -
                 ((4.49804918441121E+6 * x_y_idx_0 + 508157.762006947 * x_y_idx_1) +
                  853.483487856589 * x_y_idx_2)) -
                q;
            //  update state
            for (i1 = 0; i1 < 3; i1++)
            {
                a[i1] = ((b_a[i1] * d + b_a[i1 + 3] * j) + b_a[i1 + 6] * t_preview) +
                        c_a[i1] * k;
            }
            for (i1 = 0; i1 < 3; i1++)
            {
                x_x[i1] = a[i1];
                a[i1] = ((b_a[i1] * x_y_idx_0 + b_a[i1 + 3] * x_y_idx_1) +
                         b_a[i1 + 6] * x_y_idx_2) +
                        c_a[i1] * ndbl;
            }
            x_y_idx_0 = a[0];
            x_y_idx_1 = a[1];
            x_y_idx_2 = a[2];
            //  save current state to array
            com_x[static_cast<int>(c_i) - 1] = x_x[0] * 1000.0;
            com_y[static_cast<int>(c_i) - 1] = a[0] * 1000.0;
            c_i++;
        }
        else
        {
            //  Output ZMP
            ndbl = 0.0;
            q = 0.0;
            for (loop_ub = 0; loop_ub < nm1d2; loop_ub++)
            {
                d = Gd[loop_ub];
                loop_ub_tmp =
                    static_cast<int>(static_cast<unsigned int>(c_i) + loop_ub);
                ndbl += d * zmp_x_ref[loop_ub_tmp];
                q += d * zmp_y_ref[loop_ub_tmp];
            }
            d = x_x[0];
            j = x_x[1];
            t_preview = x_x[2];
            k = (-19996.142524325 *
                     (zmp_x_ref[static_cast<int>(c_i) - 1] -
                      ((x_x[0] + 0.0 * x_x[1]) + -0.012384607543323139 * x_x[2])) -
                 ((4.49804918441121E+6 * x_x[0] + 508157.762006947 * x_x[1]) +
                  853.483487856589 * x_x[2])) -
                ndbl;
            ndbl =
                (-19996.142524325 * (zmp_y_ref[static_cast<int>(c_i) - 1] -
                                     ((x_y_idx_0 + 0.0 * x_y_idx_1) +
                                      -0.012384607543323139 * x_y_idx_2)) -
                 ((4.49804918441121E+6 * x_y_idx_0 + 508157.762006947 * x_y_idx_1) +
                  853.483487856589 * x_y_idx_2)) -
                q;
            //  update state
            for (i1 = 0; i1 < 3; i1++)
            {
                a[i1] = ((b_a[i1] * d + b_a[i1 + 3] * j) + b_a[i1 + 6] * t_preview) +
                        c_a[i1] * k;
            }
            for (i1 = 0; i1 < 3; i1++)
            {
                x_x[i1] = a[i1];
                a[i1] = ((b_a[i1] * x_y_idx_0 + b_a[i1 + 3] * x_y_idx_1) +
                         b_a[i1 + 6] * x_y_idx_2) +
                        c_a[i1] * ndbl;
            }
            x_y_idx_0 = a[0];
            x_y_idx_1 = a[1];
            x_y_idx_2 = a[2];
            //  save current state to array
            com_x[static_cast<int>(c_i) - 1] = x_x[0] * 1000.0 * 0.95;
            com_y[static_cast<int>(c_i) - 1] = a[0] * 1000.0 * 0.95;
            c_i++;
        }
    }
}
