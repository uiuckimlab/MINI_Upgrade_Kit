#include <ros/ros.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include <cstddef>
#include <cstdlib>
#include <cmath>

double joint_angle[16];

void angleCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);

int main(int argc, char **argv)
{

     ros::init(argc, argv, "com_calc");
     ros::NodeHandle nh;
     ros::Publisher chatter_pub = nh.advertise<std_msgs::Float32MultiArray>("COM_pos", 100);
     ros::Subscriber sub = nh.subscribe("joint_angles", 100, angleCallback);
     ros::Rate loop_rate(100);
     double P_CoM_part_tmp;
     double P_CoM_part_tmp_tmp;
     double ab_P_CoM_part_tmp;
     double b_P_CoM_part_tmp;
     double b_P_CoM_part_tmp_tmp;
     double bb_P_CoM_part_tmp;
     double c_P_CoM_part_tmp;
     double c_P_CoM_part_tmp_tmp;
     double cb_P_CoM_part_tmp;
     double d_P_CoM_part_tmp;
     double d_P_CoM_part_tmp_tmp;
     double db_P_CoM_part_tmp;
     double e_P_CoM_part_tmp;
     double e_P_CoM_part_tmp_tmp;
     double eb_P_CoM_part_tmp;
     double f_P_CoM_part_tmp;
     double f_P_CoM_part_tmp_tmp;
     double fb_P_CoM_part_tmp;
     double g_P_CoM_part_tmp;
     double g_P_CoM_part_tmp_tmp;
     double gb_P_CoM_part_tmp;
     double h_P_CoM_part_tmp;
     double h_P_CoM_part_tmp_tmp;
     double hb_P_CoM_part_tmp;
     double i_P_CoM_part_tmp;
     double i_P_CoM_part_tmp_tmp;
     double ib_P_CoM_part_tmp;
     double j_P_CoM_part_tmp;
     double j_P_CoM_part_tmp_tmp;
     double jb_P_CoM_part_tmp;
     double k_P_CoM_part_tmp;
     double kb_P_CoM_part_tmp;
     double l_P_CoM_part_tmp;
     double lb_P_CoM_part_tmp;
     double m_P_CoM_part_tmp;
     double n_P_CoM_part_tmp;
     double o_P_CoM_part_tmp;
     double p_P_CoM_part_tmp;
     double q_P_CoM_part_tmp;
     double r_P_CoM_part_tmp;
     double s_P_CoM_part_tmp;
     double t_P_CoM_part_tmp;
     double u_P_CoM_part_tmp;
     double v_P_CoM_part_tmp;
     double w_P_CoM_part_tmp;
     double x_P_CoM_part_tmp;
     double y_P_CoM_part_tmp;
     double P_CoM[3];

     while (ros::ok())
     {
          P_CoM_part_tmp = std::cos(joint_angle[0]);
          b_P_CoM_part_tmp = std::sin(joint_angle[0]);
          c_P_CoM_part_tmp = std::sin(joint_angle[1]);
          d_P_CoM_part_tmp = std::cos(joint_angle[1]);
          P_CoM_part_tmp_tmp = joint_angle[1] + joint_angle[2];
          e_P_CoM_part_tmp = std::sin(P_CoM_part_tmp_tmp);
          f_P_CoM_part_tmp = std::cos(P_CoM_part_tmp_tmp);
          //  Left Hand
          g_P_CoM_part_tmp = std::sin(joint_angle[3]);
          h_P_CoM_part_tmp = std::cos(joint_angle[4]);
          i_P_CoM_part_tmp = std::sin(joint_angle[4]);
          j_P_CoM_part_tmp = std::cos(joint_angle[3]);
          k_P_CoM_part_tmp = std::cos(joint_angle[5]);
          l_P_CoM_part_tmp = std::sin(joint_angle[5]);
          //  Right Foot
          P_CoM_part_tmp_tmp = std::cos(joint_angle[6]);
          b_P_CoM_part_tmp_tmp = std::sin(joint_angle[6]);
          c_P_CoM_part_tmp_tmp = std::cos(joint_angle[7]);
          m_P_CoM_part_tmp = std::sin(joint_angle[7]);
          n_P_CoM_part_tmp = std::sin(joint_angle[8]);
          o_P_CoM_part_tmp = std::cos(joint_angle[8]);
          p_P_CoM_part_tmp = joint_angle[7] + joint_angle[8];
          q_P_CoM_part_tmp = std::cos(p_P_CoM_part_tmp);
          r_P_CoM_part_tmp = std::sin(p_P_CoM_part_tmp);
          s_P_CoM_part_tmp = std::cos(joint_angle[9]);
          t_P_CoM_part_tmp = std::sin(joint_angle[9]);
          u_P_CoM_part_tmp = p_P_CoM_part_tmp - joint_angle[9];
          d_P_CoM_part_tmp_tmp = 45.0 * c_P_CoM_part_tmp_tmp * b_P_CoM_part_tmp_tmp;
          e_P_CoM_part_tmp_tmp = 45.0 * P_CoM_part_tmp_tmp * c_P_CoM_part_tmp_tmp;
          v_P_CoM_part_tmp = std::sin(u_P_CoM_part_tmp);
          w_P_CoM_part_tmp = std::cos(joint_angle[10]);
          x_P_CoM_part_tmp = std::sin(joint_angle[10]);
          y_P_CoM_part_tmp = std::cos(u_P_CoM_part_tmp);
          //  Left Foot
          f_P_CoM_part_tmp_tmp = std::cos(joint_angle[11]);
          g_P_CoM_part_tmp_tmp = std::sin(joint_angle[11]);
          h_P_CoM_part_tmp_tmp = std::cos(joint_angle[12]);
          ab_P_CoM_part_tmp = std::sin(joint_angle[12]);
          bb_P_CoM_part_tmp = std::cos(joint_angle[13]);
          cb_P_CoM_part_tmp = std::sin(joint_angle[13]);
          i_P_CoM_part_tmp_tmp = joint_angle[12] + joint_angle[13];
          db_P_CoM_part_tmp = std::cos(i_P_CoM_part_tmp_tmp);
          eb_P_CoM_part_tmp = std::sin(i_P_CoM_part_tmp_tmp);
          fb_P_CoM_part_tmp = std::cos(joint_angle[14]);
          gb_P_CoM_part_tmp = std::sin(joint_angle[14]);
          hb_P_CoM_part_tmp = i_P_CoM_part_tmp_tmp - joint_angle[14];
          i_P_CoM_part_tmp_tmp = 45.0 * h_P_CoM_part_tmp_tmp * g_P_CoM_part_tmp_tmp;
          j_P_CoM_part_tmp_tmp = 45.0 * f_P_CoM_part_tmp_tmp * h_P_CoM_part_tmp_tmp;
          ib_P_CoM_part_tmp = std::sin(hb_P_CoM_part_tmp);
          jb_P_CoM_part_tmp = std::cos(joint_angle[15]);
          kb_P_CoM_part_tmp = std::sin(joint_angle[15]);
          lb_P_CoM_part_tmp = std::cos(hb_P_CoM_part_tmp);
          //  Calculate Wholebody COM
          P_CoM[0] =
              ((((((((((((((((457.0 * b_P_CoM_part_tmp / 100.0 * 4.26 +
                              -3442.1477999999997) +
                             (((P_CoM_part_tmp / 20.0 -
                                97.0 * d_P_CoM_part_tmp * b_P_CoM_part_tmp / 100.0) -
                               241.0 * b_P_CoM_part_tmp * c_P_CoM_part_tmp / 20.0) +
                              12.0 * b_P_CoM_part_tmp) *
                                 23.0) +
                            (((b_P_CoM_part_tmp * (12.0 - 45.0 * c_P_CoM_part_tmp) -
                               20.0 * e_P_CoM_part_tmp * b_P_CoM_part_tmp) -
                              9.0 * P_CoM_part_tmp / 50.0) -
                             129.0 * f_P_CoM_part_tmp * b_P_CoM_part_tmp / 100.0) *
                                26.16) +
                           457.0 * std::cos(joint_angle[3] + 1.5707963267948966) /
                               100.0 * 4.26) +
                          (((j_P_CoM_part_tmp / 20.0 +
                             97.0 * h_P_CoM_part_tmp * g_P_CoM_part_tmp / 100.0) -
                            241.0 * g_P_CoM_part_tmp * i_P_CoM_part_tmp / 20.0) -
                           12.0 * g_P_CoM_part_tmp) *
                              23.0) +
                         ((((((129.0 * h_P_CoM_part_tmp * k_P_CoM_part_tmp *
                                   g_P_CoM_part_tmp / 100.0 -
                               12.0 * std::sin(joint_angle[3])) -
                              20.0 * h_P_CoM_part_tmp * g_P_CoM_part_tmp *
                                  l_P_CoM_part_tmp) -
                             20.0 * k_P_CoM_part_tmp * g_P_CoM_part_tmp *
                                 i_P_CoM_part_tmp) -
                            129.0 * g_P_CoM_part_tmp * i_P_CoM_part_tmp *
                                l_P_CoM_part_tmp / 100.0) -
                           45.0 * g_P_CoM_part_tmp * i_P_CoM_part_tmp) -
                          9.0 * j_P_CoM_part_tmp / 50.0) *
                             26.16) +
                        269.24699999999996) +
                       ((97.0 * c_P_CoM_part_tmp_tmp / 100.0 + 15.0) -
                        659.0 * m_P_CoM_part_tmp / 20.0) *
                           23.0) +
                      ((15.0 - 45.0 * m_P_CoM_part_tmp) +
                       958.83314502576513 *
                           std::cos(p_P_CoM_part_tmp + 1.2038471279410756) / 50.0) *
                          8.99) +
                     (((15.0 - 42.0 * r_P_CoM_part_tmp) -
                       45.0 * std::sin(joint_angle[7])) -
                      647.38319409759163 *
                          std::cos(u_P_CoM_part_tmp + 1.2954671661606658) / 100.0) *
                         19.57) +
                    (((((15.0 - 1053.0 * y_P_CoM_part_tmp / 50.0) -
                        42.0 * std::sin(joint_angle[7] + joint_angle[8])) -
                       45.0 * std::sin(joint_angle[7])) -
                      49.0 * v_P_CoM_part_tmp * w_P_CoM_part_tmp / 4.0) +
                     801.0 * v_P_CoM_part_tmp * x_P_CoM_part_tmp / 100.0) *
                        36.53) +
                   269.24699999999996) +
                  ((97.0 * h_P_CoM_part_tmp_tmp / 100.0 + 15.0) +
                   659.0 * ab_P_CoM_part_tmp / 20.0) *
                      23.0) +
                 (((((172.0 * h_P_CoM_part_tmp_tmp * bb_P_CoM_part_tmp / 25.0 + 15.0) +
                     179.0 * h_P_CoM_part_tmp_tmp * cb_P_CoM_part_tmp / 10.0) +
                    179.0 * bb_P_CoM_part_tmp * ab_P_CoM_part_tmp / 10.0) -
                   172.0 * ab_P_CoM_part_tmp * cb_P_CoM_part_tmp / 25.0) +
                  45.0 * ab_P_CoM_part_tmp) *
                     8.99) +
                (((42.0 * eb_P_CoM_part_tmp + 15.0) +
                  45.0 * std::sin(joint_angle[12])) -
                 647.38319409759163 * std::cos(hb_P_CoM_part_tmp - 1.2954671661606658) /
                     100.0) *
                    19.57) +
               (((((15.0 - 1053.0 * lb_P_CoM_part_tmp / 50.0) +
                   42.0 * std::sin(joint_angle[12] + joint_angle[13])) +
                  45.0 * std::sin(joint_angle[12])) +
                 49.0 * ib_P_CoM_part_tmp * jb_P_CoM_part_tmp / 4.0) +
                801.0 * ib_P_CoM_part_tmp * kb_P_CoM_part_tmp / 100.0) *
                   36.53) /
              679.15;
          P_CoM[1] =
              ((((((((((((((((((97.0 * c_P_CoM_part_tmp / 100.0 - 39.0) -
                               241.0 * d_P_CoM_part_tmp / 20.0) -
                              18.0) *
                                 23.0 +
                             -493.74629999999996) +
                            ((((129.0 * e_P_CoM_part_tmp / 100.0 - 39.0) -
                               20.0 * f_P_CoM_part_tmp) -
                              18.0) -
                             45.0 * d_P_CoM_part_tmp) *
                                26.16) +
                           205.11899999999997) +
                          ((241.0 * h_P_CoM_part_tmp / 20.0 + 57.0) +
                           97.0 * i_P_CoM_part_tmp / 100.0) *
                              23.0) +
                         ((2004.1559320571839 *
                               std::cos((joint_angle[4] + joint_angle[5]) -
                                        0.064410777232744368) /
                               100.0 +
                           57.0) +
                          45.0 * h_P_CoM_part_tmp) *
                             26.16) +
                        ((-24.0 - 3.0 * P_CoM_part_tmp_tmp / 50.0) -
                         61.0 * b_P_CoM_part_tmp_tmp / 50.0) *
                            19.9) +
                       ((((659.0 * c_P_CoM_part_tmp_tmp * b_P_CoM_part_tmp_tmp / 20.0 -
                           P_CoM_part_tmp_tmp / 20.0) -
                          24.0) +
                         97.0 * b_P_CoM_part_tmp_tmp * m_P_CoM_part_tmp / 100.0) +
                        6.0 * b_P_CoM_part_tmp_tmp) *
                           23.0) +
                      ((((((6.0 * std::sin(joint_angle[6]) - 24.0) +
                           172.0 * c_P_CoM_part_tmp_tmp * b_P_CoM_part_tmp_tmp *
                               n_P_CoM_part_tmp / 25.0) +
                          172.0 * o_P_CoM_part_tmp * b_P_CoM_part_tmp_tmp *
                              m_P_CoM_part_tmp / 25.0) -
                         179.0 * b_P_CoM_part_tmp_tmp * m_P_CoM_part_tmp *
                             n_P_CoM_part_tmp / 10.0) +
                        d_P_CoM_part_tmp_tmp) +
                       179.0 * c_P_CoM_part_tmp_tmp * o_P_CoM_part_tmp *
                           b_P_CoM_part_tmp_tmp / 10.0) *
                          8.99) +
                     ((((((((6.0 * std::sin(joint_angle[6]) -
                             3.0 * std::cos(joint_angle[6]) / 50.0) -
                            24.0) +
                           42.0 * q_P_CoM_part_tmp * b_P_CoM_part_tmp_tmp) +
                          d_P_CoM_part_tmp_tmp) -
                         623.0 * q_P_CoM_part_tmp * s_P_CoM_part_tmp *
                             b_P_CoM_part_tmp_tmp / 100.0) +
                        44.0 * q_P_CoM_part_tmp * b_P_CoM_part_tmp_tmp *
                            t_P_CoM_part_tmp / 25.0) -
                       44.0 * r_P_CoM_part_tmp * s_P_CoM_part_tmp *
                           b_P_CoM_part_tmp_tmp / 25.0) -
                      623.0 * r_P_CoM_part_tmp * b_P_CoM_part_tmp_tmp *
                          t_P_CoM_part_tmp / 100.0) *
                         19.57) +
                    (((((((((6.0 * std::sin(joint_angle[6]) -
                             801.0 * P_CoM_part_tmp_tmp * w_P_CoM_part_tmp / 100.0) -
                            49.0 * P_CoM_part_tmp_tmp * x_P_CoM_part_tmp / 4.0) -
                           24.0) +
                          42.0 * std::cos(joint_angle[7] + joint_angle[8]) *
                              std::sin(joint_angle[6])) +
                         d_P_CoM_part_tmp_tmp) +
                        49.0 * y_P_CoM_part_tmp * w_P_CoM_part_tmp *
                            b_P_CoM_part_tmp_tmp / 4.0) -
                       801.0 * y_P_CoM_part_tmp * b_P_CoM_part_tmp_tmp *
                           x_P_CoM_part_tmp / 100.0) +
                      1053.0 * q_P_CoM_part_tmp * b_P_CoM_part_tmp_tmp *
                          t_P_CoM_part_tmp / 50.0) -
                     1053.0 * r_P_CoM_part_tmp * s_P_CoM_part_tmp *
                         b_P_CoM_part_tmp_tmp / 50.0) *
                        36.53) +
                   ((3.0 * f_P_CoM_part_tmp_tmp / 50.0 + 24.0) -
                    61.0 * g_P_CoM_part_tmp_tmp / 50.0) *
                       19.9) +
                  ((((f_P_CoM_part_tmp_tmp / 20.0 + 24.0) +
                     659.0 * h_P_CoM_part_tmp_tmp * g_P_CoM_part_tmp_tmp / 20.0) -
                    97.0 * g_P_CoM_part_tmp_tmp * ab_P_CoM_part_tmp / 100.0) +
                   6.0 * g_P_CoM_part_tmp_tmp) *
                      23.0) +
                 ((((((6.0 * std::sin(joint_angle[11]) + 24.0) -
                      172.0 * std::cos(joint_angle[12]) * g_P_CoM_part_tmp_tmp *
                          cb_P_CoM_part_tmp / 25.0) -
                     172.0 * bb_P_CoM_part_tmp * g_P_CoM_part_tmp_tmp *
                         ab_P_CoM_part_tmp / 25.0) -
                    179.0 * g_P_CoM_part_tmp_tmp * ab_P_CoM_part_tmp *
                        cb_P_CoM_part_tmp / 10.0) +
                   i_P_CoM_part_tmp_tmp) +
                  179.0 * std::cos(joint_angle[12]) * bb_P_CoM_part_tmp *
                      g_P_CoM_part_tmp_tmp / 10.0) *
                     8.99) +
                ((((((((3.0 * std::cos(joint_angle[11]) / 50.0 + 24.0) +
                       6.0 * std::sin(joint_angle[11])) +
                      42.0 * db_P_CoM_part_tmp * g_P_CoM_part_tmp_tmp) +
                     i_P_CoM_part_tmp_tmp) -
                    623.0 * db_P_CoM_part_tmp * fb_P_CoM_part_tmp *
                        g_P_CoM_part_tmp_tmp / 100.0) -
                   44.0 * db_P_CoM_part_tmp * g_P_CoM_part_tmp_tmp * gb_P_CoM_part_tmp /
                       25.0) +
                  44.0 * eb_P_CoM_part_tmp * fb_P_CoM_part_tmp * g_P_CoM_part_tmp_tmp /
                      25.0) -
                 623.0 * eb_P_CoM_part_tmp * g_P_CoM_part_tmp_tmp * gb_P_CoM_part_tmp /
                     100.0) *
                    19.57) +
               (((((((((801.0 * f_P_CoM_part_tmp_tmp * jb_P_CoM_part_tmp / 100.0 +
                        24.0) -
                       49.0 * f_P_CoM_part_tmp_tmp * kb_P_CoM_part_tmp / 4.0) +
                      6.0 * std::sin(joint_angle[11])) +
                     42.0 * std::cos(joint_angle[12] + joint_angle[13]) *
                         std::sin(joint_angle[11])) +
                    i_P_CoM_part_tmp_tmp) +
                   49.0 * lb_P_CoM_part_tmp * jb_P_CoM_part_tmp * g_P_CoM_part_tmp_tmp /
                       4.0) +
                  801.0 * lb_P_CoM_part_tmp * g_P_CoM_part_tmp_tmp * kb_P_CoM_part_tmp /
                      100.0) -
                 1053.0 * db_P_CoM_part_tmp * g_P_CoM_part_tmp_tmp * gb_P_CoM_part_tmp /
                     50.0) +
                1053.0 * eb_P_CoM_part_tmp * fb_P_CoM_part_tmp * g_P_CoM_part_tmp_tmp /
                    50.0) *
                   36.53) /
              679.15;
          P_CoM[2] =
              ((((((((((((((((-(457.0 * P_CoM_part_tmp) / 100.0 * 4.26 + -5220.2345) +
                             (((b_P_CoM_part_tmp / 20.0 +
                                97.0 * P_CoM_part_tmp * d_P_CoM_part_tmp / 100.0) +
                               241.0 * P_CoM_part_tmp * c_P_CoM_part_tmp / 20.0) -
                              12.0 * P_CoM_part_tmp) *
                                 23.0) +
                            (((129.0 * std::cos(joint_angle[1] + joint_angle[2]) *
                                   P_CoM_part_tmp / 100.0 -
                               P_CoM_part_tmp *
                                   (12.0 - 45.0 * std::sin(joint_angle[1]))) -
                              9.0 * b_P_CoM_part_tmp / 50.0) +
                             20.0 * std::sin(joint_angle[1] + joint_angle[2]) *
                                 P_CoM_part_tmp) *
                                26.16) +
                           -(457.0 * std::sin(joint_angle[3] + 1.5707963267948966)) /
                               100.0 * 4.26) +
                          (((97.0 * j_P_CoM_part_tmp * h_P_CoM_part_tmp / 100.0 -
                             g_P_CoM_part_tmp / 20.0) -
                            241.0 * j_P_CoM_part_tmp * i_P_CoM_part_tmp / 20.0) -
                           12.0 * j_P_CoM_part_tmp) *
                              23.0) +
                         ((((((9.0 * g_P_CoM_part_tmp / 50.0 -
                               12.0 * std::cos(joint_angle[3])) -
                              129.0 * j_P_CoM_part_tmp * i_P_CoM_part_tmp *
                                  l_P_CoM_part_tmp / 100.0) -
                             45.0 * j_P_CoM_part_tmp * i_P_CoM_part_tmp) +
                            129.0 * std::cos(joint_angle[3]) * h_P_CoM_part_tmp *
                                k_P_CoM_part_tmp / 100.0) -
                           20.0 * j_P_CoM_part_tmp * h_P_CoM_part_tmp *
                               l_P_CoM_part_tmp) -
                          20.0 * std::cos(joint_angle[3]) * k_P_CoM_part_tmp *
                              i_P_CoM_part_tmp) *
                             26.16) +
                        ((61.0 * P_CoM_part_tmp_tmp / 50.0 - 72.0) -
                         3.0 * b_P_CoM_part_tmp_tmp / 50.0) *
                            19.9) +
                       ((((-72.0 - b_P_CoM_part_tmp_tmp / 20.0) -
                          659.0 * P_CoM_part_tmp_tmp * c_P_CoM_part_tmp_tmp / 20.0) -
                         97.0 * P_CoM_part_tmp_tmp * m_P_CoM_part_tmp / 100.0) -
                        6.0 * P_CoM_part_tmp_tmp) *
                           23.0) +
                      ((((((179.0 * P_CoM_part_tmp_tmp * m_P_CoM_part_tmp *
                                n_P_CoM_part_tmp / 10.0 -
                            6.0 * std::cos(joint_angle[6])) -
                           72.0) -
                          e_P_CoM_part_tmp_tmp) -
                         179.0 * std::cos(joint_angle[6]) * c_P_CoM_part_tmp_tmp *
                             o_P_CoM_part_tmp / 10.0) -
                        172.0 * P_CoM_part_tmp_tmp * c_P_CoM_part_tmp_tmp *
                            n_P_CoM_part_tmp / 25.0) -
                       172.0 * std::cos(joint_angle[6]) * o_P_CoM_part_tmp *
                           m_P_CoM_part_tmp / 25.0) *
                          8.99) +
                     ((((((((623.0 * std::cos(joint_angle[7] + joint_angle[8]) *
                                 P_CoM_part_tmp_tmp * s_P_CoM_part_tmp / 100.0 -
                             3.0 * std::sin(joint_angle[6]) / 50.0) -
                            6.0 * std::cos(joint_angle[6])) -
                           42.0 * std::cos(joint_angle[7] + joint_angle[8]) *
                               P_CoM_part_tmp_tmp) -
                          e_P_CoM_part_tmp_tmp) -
                         72.0) -
                        44.0 * std::cos(joint_angle[7] + joint_angle[8]) *
                            P_CoM_part_tmp_tmp * t_P_CoM_part_tmp / 25.0) +
                       44.0 * std::sin(joint_angle[7] + joint_angle[8]) *
                           P_CoM_part_tmp_tmp * s_P_CoM_part_tmp / 25.0) +
                      623.0 * std::sin(joint_angle[7] + joint_angle[8]) *
                          P_CoM_part_tmp_tmp * t_P_CoM_part_tmp / 100.0) *
                         19.57) +
                    (((((((((801.0 *
                                 std::cos((joint_angle[7] + joint_angle[8]) -
                                          joint_angle[9]) *
                                 P_CoM_part_tmp_tmp * x_P_CoM_part_tmp / 100.0 -
                             801.0 * w_P_CoM_part_tmp * b_P_CoM_part_tmp_tmp / 100.0) -
                            49.0 * b_P_CoM_part_tmp_tmp * x_P_CoM_part_tmp / 4.0) -
                           6.0 * std::cos(joint_angle[6])) -
                          42.0 * std::cos(joint_angle[7] + joint_angle[8]) *
                              std::cos(joint_angle[6])) -
                         e_P_CoM_part_tmp_tmp) -
                        49.0 *
                            std::cos((joint_angle[7] + joint_angle[8]) -
                                     joint_angle[9]) *
                            P_CoM_part_tmp_tmp * w_P_CoM_part_tmp / 4.0) -
                       72.0) -
                      1053.0 * std::cos(joint_angle[7] + joint_angle[8]) *
                          P_CoM_part_tmp_tmp * t_P_CoM_part_tmp / 50.0) +
                     1053.0 * std::sin(joint_angle[7] + joint_angle[8]) *
                         P_CoM_part_tmp_tmp * s_P_CoM_part_tmp / 50.0) *
                        36.53) +
                   ((61.0 * f_P_CoM_part_tmp_tmp / 50.0 - 72.0) +
                    3.0 * g_P_CoM_part_tmp_tmp / 50.0) *
                       19.9) +
                  ((((g_P_CoM_part_tmp_tmp / 20.0 - 72.0) -
                     659.0 * f_P_CoM_part_tmp_tmp * h_P_CoM_part_tmp_tmp / 20.0) +
                    97.0 * f_P_CoM_part_tmp_tmp * ab_P_CoM_part_tmp / 100.0) -
                   6.0 * f_P_CoM_part_tmp_tmp) *
                      23.0) +
                 ((((((179.0 * f_P_CoM_part_tmp_tmp * ab_P_CoM_part_tmp *
                           cb_P_CoM_part_tmp / 10.0 -
                       6.0 * std::cos(joint_angle[11])) -
                      72.0) -
                     j_P_CoM_part_tmp_tmp) -
                    179.0 * std::cos(joint_angle[11]) * h_P_CoM_part_tmp_tmp *
                        bb_P_CoM_part_tmp / 10.0) +
                   172.0 * f_P_CoM_part_tmp_tmp * h_P_CoM_part_tmp_tmp *
                       cb_P_CoM_part_tmp / 25.0) +
                  172.0 * std::cos(joint_angle[11]) * bb_P_CoM_part_tmp *
                      ab_P_CoM_part_tmp / 25.0) *
                     8.99) +
                ((((((((3.0 * std::sin(joint_angle[11]) / 50.0 - 72.0) -
                       6.0 * std::cos(joint_angle[11])) -
                      42.0 * std::cos(joint_angle[12] + joint_angle[13]) *
                          f_P_CoM_part_tmp_tmp) -
                     j_P_CoM_part_tmp_tmp) +
                    623.0 * std::cos(joint_angle[12] + joint_angle[13]) *
                        f_P_CoM_part_tmp_tmp * fb_P_CoM_part_tmp / 100.0) +
                   44.0 * std::cos(joint_angle[12] + joint_angle[13]) *
                       f_P_CoM_part_tmp_tmp * gb_P_CoM_part_tmp / 25.0) -
                  44.0 * std::sin(joint_angle[12] + joint_angle[13]) *
                      f_P_CoM_part_tmp_tmp * fb_P_CoM_part_tmp / 25.0) +
                 623.0 * std::sin(joint_angle[12] + joint_angle[13]) *
                     f_P_CoM_part_tmp_tmp * gb_P_CoM_part_tmp / 100.0) *
                    19.57) +
               (((((((((801.0 * jb_P_CoM_part_tmp * g_P_CoM_part_tmp_tmp / 100.0 -
                        72.0) -
                       49.0 * g_P_CoM_part_tmp_tmp * kb_P_CoM_part_tmp / 4.0) -
                      6.0 * std::cos(joint_angle[11])) -
                     42.0 * std::cos(joint_angle[12] + joint_angle[13]) *
                         std::cos(joint_angle[11])) -
                    j_P_CoM_part_tmp_tmp) -
                   49.0 *
                       std::cos((joint_angle[12] + joint_angle[13]) - joint_angle[14]) *
                       f_P_CoM_part_tmp_tmp * jb_P_CoM_part_tmp / 4.0) -
                  801.0 *
                      std::cos((joint_angle[12] + joint_angle[13]) - joint_angle[14]) *
                      f_P_CoM_part_tmp_tmp * kb_P_CoM_part_tmp / 100.0) +
                 1053.0 * std::cos(joint_angle[12] + joint_angle[13]) *
                     f_P_CoM_part_tmp_tmp * gb_P_CoM_part_tmp / 50.0) -
                1053.0 * std::sin(joint_angle[12] + joint_angle[13]) *
                    f_P_CoM_part_tmp_tmp * fb_P_CoM_part_tmp / 50.0) *
                   36.53) /
              679.15;
          loop_rate.sleep();
          ros::spinOnce();
     }
}

void angleCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
     for (int i = 0; i < 16; i++)
     {
          joint_angle[i] = msg->data[i];
          printf("j: %f   ", joint_angle[i]);
     }
     printf("\n");
}

//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// COM_calc.cpp
//
// Code generation for function 'COM_calc'
//

// Include files
#include "COM_calc.h"
#include <cmath>

// Function Definitions
void COM_calc(const double joint_angle[16], double P_CoM[3])
{
     double P_CoM_part_tmp;
     double P_CoM_part_tmp_tmp;
     double ab_P_CoM_part_tmp;
     double b_P_CoM_part_tmp;
     double b_P_CoM_part_tmp_tmp;
     double bb_P_CoM_part_tmp;
     double c_P_CoM_part_tmp;
     double c_P_CoM_part_tmp_tmp;
     double cb_P_CoM_part_tmp;
     double d_P_CoM_part_tmp;
     double d_P_CoM_part_tmp_tmp;
     double db_P_CoM_part_tmp;
     double e_P_CoM_part_tmp;
     double e_P_CoM_part_tmp_tmp;
     double eb_P_CoM_part_tmp;
     double f_P_CoM_part_tmp;
     double f_P_CoM_part_tmp_tmp;
     double fb_P_CoM_part_tmp;
     double g_P_CoM_part_tmp;
     double g_P_CoM_part_tmp_tmp;
     double gb_P_CoM_part_tmp;
     double h_P_CoM_part_tmp;
     double h_P_CoM_part_tmp_tmp;
     double hb_P_CoM_part_tmp;
     double i_P_CoM_part_tmp;
     double i_P_CoM_part_tmp_tmp;
     double ib_P_CoM_part_tmp;
     double j_P_CoM_part_tmp;
     double j_P_CoM_part_tmp_tmp;
     double jb_P_CoM_part_tmp;
     double k_P_CoM_part_tmp;
     double kb_P_CoM_part_tmp;
     double l_P_CoM_part_tmp;
     double lb_P_CoM_part_tmp;
     double m_P_CoM_part_tmp;
     double n_P_CoM_part_tmp;
     double o_P_CoM_part_tmp;
     double p_P_CoM_part_tmp;
     double q_P_CoM_part_tmp;
     double r_P_CoM_part_tmp;
     double s_P_CoM_part_tmp;
     double t_P_CoM_part_tmp;
     double u_P_CoM_part_tmp;
     double v_P_CoM_part_tmp;
     double w_P_CoM_part_tmp;
     double x_P_CoM_part_tmp;
     double y_P_CoM_part_tmp;
     //  Origin to arm roll joint
     //  Shoulder bracket horizontal distance
     //  Shoulder bracket vertical distance
     //  Upper arm length
     //  Lower arm length
     //  Origin to pelvis vertical length
     //  Pelvis horizontal length
     //  Shoulder joint axis to Leg center (On Sagittal Plane)
     //  Pelvis Roll axis to pitch axis
     //  Thigh Length
     //  Shank Length
     //  Ankle Length
     //  Right Hand
     P_CoM_part_tmp = std::cos(joint_angle[0]);
     b_P_CoM_part_tmp = std::sin(joint_angle[0]);
     c_P_CoM_part_tmp = std::sin(joint_angle[1]);
     d_P_CoM_part_tmp = std::cos(joint_angle[1]);
     P_CoM_part_tmp_tmp = joint_angle[1] + joint_angle[2];
     e_P_CoM_part_tmp = std::sin(P_CoM_part_tmp_tmp);
     f_P_CoM_part_tmp = std::cos(P_CoM_part_tmp_tmp);
     //  Left Hand
     g_P_CoM_part_tmp = std::sin(joint_angle[3]);
     h_P_CoM_part_tmp = std::cos(joint_angle[4]);
     i_P_CoM_part_tmp = std::sin(joint_angle[4]);
     j_P_CoM_part_tmp = std::cos(joint_angle[3]);
     k_P_CoM_part_tmp = std::cos(joint_angle[5]);
     l_P_CoM_part_tmp = std::sin(joint_angle[5]);
     //  Right Foot
     P_CoM_part_tmp_tmp = std::cos(joint_angle[6]);
     b_P_CoM_part_tmp_tmp = std::sin(joint_angle[6]);
     c_P_CoM_part_tmp_tmp = std::cos(joint_angle[7]);
     m_P_CoM_part_tmp = std::sin(joint_angle[7]);
     n_P_CoM_part_tmp = std::sin(joint_angle[8]);
     o_P_CoM_part_tmp = std::cos(joint_angle[8]);
     p_P_CoM_part_tmp = joint_angle[7] + joint_angle[8];
     q_P_CoM_part_tmp = std::cos(p_P_CoM_part_tmp);
     r_P_CoM_part_tmp = std::sin(p_P_CoM_part_tmp);
     s_P_CoM_part_tmp = std::cos(joint_angle[9]);
     t_P_CoM_part_tmp = std::sin(joint_angle[9]);
     u_P_CoM_part_tmp = p_P_CoM_part_tmp - joint_angle[9];
     d_P_CoM_part_tmp_tmp = 45.0 * c_P_CoM_part_tmp_tmp * b_P_CoM_part_tmp_tmp;
     e_P_CoM_part_tmp_tmp = 45.0 * P_CoM_part_tmp_tmp * c_P_CoM_part_tmp_tmp;
     v_P_CoM_part_tmp = std::sin(u_P_CoM_part_tmp);
     w_P_CoM_part_tmp = std::cos(joint_angle[10]);
     x_P_CoM_part_tmp = std::sin(joint_angle[10]);
     y_P_CoM_part_tmp = std::cos(u_P_CoM_part_tmp);
     //  Left Foot
     f_P_CoM_part_tmp_tmp = std::cos(joint_angle[11]);
     g_P_CoM_part_tmp_tmp = std::sin(joint_angle[11]);
     h_P_CoM_part_tmp_tmp = std::cos(joint_angle[12]);
     ab_P_CoM_part_tmp = std::sin(joint_angle[12]);
     bb_P_CoM_part_tmp = std::cos(joint_angle[13]);
     cb_P_CoM_part_tmp = std::sin(joint_angle[13]);
     i_P_CoM_part_tmp_tmp = joint_angle[12] + joint_angle[13];
     db_P_CoM_part_tmp = std::cos(i_P_CoM_part_tmp_tmp);
     eb_P_CoM_part_tmp = std::sin(i_P_CoM_part_tmp_tmp);
     fb_P_CoM_part_tmp = std::cos(joint_angle[14]);
     gb_P_CoM_part_tmp = std::sin(joint_angle[14]);
     hb_P_CoM_part_tmp = i_P_CoM_part_tmp_tmp - joint_angle[14];
     i_P_CoM_part_tmp_tmp = 45.0 * h_P_CoM_part_tmp_tmp * g_P_CoM_part_tmp_tmp;
     j_P_CoM_part_tmp_tmp = 45.0 * f_P_CoM_part_tmp_tmp * h_P_CoM_part_tmp_tmp;
     ib_P_CoM_part_tmp = std::sin(hb_P_CoM_part_tmp);
     jb_P_CoM_part_tmp = std::cos(joint_angle[15]);
     kb_P_CoM_part_tmp = std::sin(joint_angle[15]);
     lb_P_CoM_part_tmp = std::cos(hb_P_CoM_part_tmp);
     //  Calculate Wholebody COM
     P_CoM[0] =
         ((((((((((((((((457.0 * b_P_CoM_part_tmp / 100.0 * 4.26 +
                         -3442.1477999999997) +
                        (((P_CoM_part_tmp / 20.0 -
                           97.0 * d_P_CoM_part_tmp * b_P_CoM_part_tmp / 100.0) -
                          241.0 * b_P_CoM_part_tmp * c_P_CoM_part_tmp / 20.0) +
                         12.0 * b_P_CoM_part_tmp) *
                            23.0) +
                       (((b_P_CoM_part_tmp * (12.0 - 45.0 * c_P_CoM_part_tmp) -
                          20.0 * e_P_CoM_part_tmp * b_P_CoM_part_tmp) -
                         9.0 * P_CoM_part_tmp / 50.0) -
                        129.0 * f_P_CoM_part_tmp * b_P_CoM_part_tmp / 100.0) *
                           26.16) +
                      457.0 * std::cos(joint_angle[3] + 1.5707963267948966) /
                          100.0 * 4.26) +
                     (((j_P_CoM_part_tmp / 20.0 +
                        97.0 * h_P_CoM_part_tmp * g_P_CoM_part_tmp / 100.0) -
                       241.0 * g_P_CoM_part_tmp * i_P_CoM_part_tmp / 20.0) -
                      12.0 * g_P_CoM_part_tmp) *
                         23.0) +
                    ((((((129.0 * h_P_CoM_part_tmp * k_P_CoM_part_tmp *
                              g_P_CoM_part_tmp / 100.0 -
                          12.0 * std::sin(joint_angle[3])) -
                         20.0 * h_P_CoM_part_tmp * g_P_CoM_part_tmp *
                             l_P_CoM_part_tmp) -
                        20.0 * k_P_CoM_part_tmp * g_P_CoM_part_tmp *
                            i_P_CoM_part_tmp) -
                       129.0 * g_P_CoM_part_tmp * i_P_CoM_part_tmp *
                           l_P_CoM_part_tmp / 100.0) -
                      45.0 * g_P_CoM_part_tmp * i_P_CoM_part_tmp) -
                     9.0 * j_P_CoM_part_tmp / 50.0) *
                        26.16) +
                   269.24699999999996) +
                  ((97.0 * c_P_CoM_part_tmp_tmp / 100.0 + 15.0) -
                   659.0 * m_P_CoM_part_tmp / 20.0) *
                      23.0) +
                 ((15.0 - 45.0 * m_P_CoM_part_tmp) +
                  958.83314502576513 *
                      std::cos(p_P_CoM_part_tmp + 1.2038471279410756) / 50.0) *
                     8.99) +
                (((15.0 - 42.0 * r_P_CoM_part_tmp) -
                  45.0 * std::sin(joint_angle[7])) -
                 647.38319409759163 *
                     std::cos(u_P_CoM_part_tmp + 1.2954671661606658) / 100.0) *
                    19.57) +
               (((((15.0 - 1053.0 * y_P_CoM_part_tmp / 50.0) -
                   42.0 * std::sin(joint_angle[7] + joint_angle[8])) -
                  45.0 * std::sin(joint_angle[7])) -
                 49.0 * v_P_CoM_part_tmp * w_P_CoM_part_tmp / 4.0) +
                801.0 * v_P_CoM_part_tmp * x_P_CoM_part_tmp / 100.0) *
                   36.53) +
              269.24699999999996) +
             ((97.0 * h_P_CoM_part_tmp_tmp / 100.0 + 15.0) +
              659.0 * ab_P_CoM_part_tmp / 20.0) *
                 23.0) +
            (((((172.0 * h_P_CoM_part_tmp_tmp * bb_P_CoM_part_tmp / 25.0 + 15.0) +
                179.0 * h_P_CoM_part_tmp_tmp * cb_P_CoM_part_tmp / 10.0) +
               179.0 * bb_P_CoM_part_tmp * ab_P_CoM_part_tmp / 10.0) -
              172.0 * ab_P_CoM_part_tmp * cb_P_CoM_part_tmp / 25.0) +
             45.0 * ab_P_CoM_part_tmp) *
                8.99) +
           (((42.0 * eb_P_CoM_part_tmp + 15.0) +
             45.0 * std::sin(joint_angle[12])) -
            647.38319409759163 * std::cos(hb_P_CoM_part_tmp - 1.2954671661606658) /
                100.0) *
               19.57) +
          (((((15.0 - 1053.0 * lb_P_CoM_part_tmp / 50.0) +
              42.0 * std::sin(joint_angle[12] + joint_angle[13])) +
             45.0 * std::sin(joint_angle[12])) +
            49.0 * ib_P_CoM_part_tmp * jb_P_CoM_part_tmp / 4.0) +
           801.0 * ib_P_CoM_part_tmp * kb_P_CoM_part_tmp / 100.0) *
              36.53) /
         679.15;
     P_CoM[1] =
         ((((((((((((((((((97.0 * c_P_CoM_part_tmp / 100.0 - 39.0) -
                          241.0 * d_P_CoM_part_tmp / 20.0) -
                         18.0) *
                            23.0 +
                        -493.74629999999996) +
                       ((((129.0 * e_P_CoM_part_tmp / 100.0 - 39.0) -
                          20.0 * f_P_CoM_part_tmp) -
                         18.0) -
                        45.0 * d_P_CoM_part_tmp) *
                           26.16) +
                      205.11899999999997) +
                     ((241.0 * h_P_CoM_part_tmp / 20.0 + 57.0) +
                      97.0 * i_P_CoM_part_tmp / 100.0) *
                         23.0) +
                    ((2004.1559320571839 *
                          std::cos((joint_angle[4] + joint_angle[5]) -
                                   0.064410777232744368) /
                          100.0 +
                      57.0) +
                     45.0 * h_P_CoM_part_tmp) *
                        26.16) +
                   ((-24.0 - 3.0 * P_CoM_part_tmp_tmp / 50.0) -
                    61.0 * b_P_CoM_part_tmp_tmp / 50.0) *
                       19.9) +
                  ((((659.0 * c_P_CoM_part_tmp_tmp * b_P_CoM_part_tmp_tmp / 20.0 -
                      P_CoM_part_tmp_tmp / 20.0) -
                     24.0) +
                    97.0 * b_P_CoM_part_tmp_tmp * m_P_CoM_part_tmp / 100.0) +
                   6.0 * b_P_CoM_part_tmp_tmp) *
                      23.0) +
                 ((((((6.0 * std::sin(joint_angle[6]) - 24.0) +
                      172.0 * c_P_CoM_part_tmp_tmp * b_P_CoM_part_tmp_tmp *
                          n_P_CoM_part_tmp / 25.0) +
                     172.0 * o_P_CoM_part_tmp * b_P_CoM_part_tmp_tmp *
                         m_P_CoM_part_tmp / 25.0) -
                    179.0 * b_P_CoM_part_tmp_tmp * m_P_CoM_part_tmp *
                        n_P_CoM_part_tmp / 10.0) +
                   d_P_CoM_part_tmp_tmp) +
                  179.0 * c_P_CoM_part_tmp_tmp * o_P_CoM_part_tmp *
                      b_P_CoM_part_tmp_tmp / 10.0) *
                     8.99) +
                ((((((((6.0 * std::sin(joint_angle[6]) -
                        3.0 * std::cos(joint_angle[6]) / 50.0) -
                       24.0) +
                      42.0 * q_P_CoM_part_tmp * b_P_CoM_part_tmp_tmp) +
                     d_P_CoM_part_tmp_tmp) -
                    623.0 * q_P_CoM_part_tmp * s_P_CoM_part_tmp *
                        b_P_CoM_part_tmp_tmp / 100.0) +
                   44.0 * q_P_CoM_part_tmp * b_P_CoM_part_tmp_tmp *
                       t_P_CoM_part_tmp / 25.0) -
                  44.0 * r_P_CoM_part_tmp * s_P_CoM_part_tmp *
                      b_P_CoM_part_tmp_tmp / 25.0) -
                 623.0 * r_P_CoM_part_tmp * b_P_CoM_part_tmp_tmp *
                     t_P_CoM_part_tmp / 100.0) *
                    19.57) +
               (((((((((6.0 * std::sin(joint_angle[6]) -
                        801.0 * P_CoM_part_tmp_tmp * w_P_CoM_part_tmp / 100.0) -
                       49.0 * P_CoM_part_tmp_tmp * x_P_CoM_part_tmp / 4.0) -
                      24.0) +
                     42.0 * std::cos(joint_angle[7] + joint_angle[8]) *
                         std::sin(joint_angle[6])) +
                    d_P_CoM_part_tmp_tmp) +
                   49.0 * y_P_CoM_part_tmp * w_P_CoM_part_tmp *
                       b_P_CoM_part_tmp_tmp / 4.0) -
                  801.0 * y_P_CoM_part_tmp * b_P_CoM_part_tmp_tmp *
                      x_P_CoM_part_tmp / 100.0) +
                 1053.0 * q_P_CoM_part_tmp * b_P_CoM_part_tmp_tmp *
                     t_P_CoM_part_tmp / 50.0) -
                1053.0 * r_P_CoM_part_tmp * s_P_CoM_part_tmp *
                    b_P_CoM_part_tmp_tmp / 50.0) *
                   36.53) +
              ((3.0 * f_P_CoM_part_tmp_tmp / 50.0 + 24.0) -
               61.0 * g_P_CoM_part_tmp_tmp / 50.0) *
                  19.9) +
             ((((f_P_CoM_part_tmp_tmp / 20.0 + 24.0) +
                659.0 * h_P_CoM_part_tmp_tmp * g_P_CoM_part_tmp_tmp / 20.0) -
               97.0 * g_P_CoM_part_tmp_tmp * ab_P_CoM_part_tmp / 100.0) +
              6.0 * g_P_CoM_part_tmp_tmp) *
                 23.0) +
            ((((((6.0 * std::sin(joint_angle[11]) + 24.0) -
                 172.0 * std::cos(joint_angle[12]) * g_P_CoM_part_tmp_tmp *
                     cb_P_CoM_part_tmp / 25.0) -
                172.0 * bb_P_CoM_part_tmp * g_P_CoM_part_tmp_tmp *
                    ab_P_CoM_part_tmp / 25.0) -
               179.0 * g_P_CoM_part_tmp_tmp * ab_P_CoM_part_tmp *
                   cb_P_CoM_part_tmp / 10.0) +
              i_P_CoM_part_tmp_tmp) +
             179.0 * std::cos(joint_angle[12]) * bb_P_CoM_part_tmp *
                 g_P_CoM_part_tmp_tmp / 10.0) *
                8.99) +
           ((((((((3.0 * std::cos(joint_angle[11]) / 50.0 + 24.0) +
                  6.0 * std::sin(joint_angle[11])) +
                 42.0 * db_P_CoM_part_tmp * g_P_CoM_part_tmp_tmp) +
                i_P_CoM_part_tmp_tmp) -
               623.0 * db_P_CoM_part_tmp * fb_P_CoM_part_tmp *
                   g_P_CoM_part_tmp_tmp / 100.0) -
              44.0 * db_P_CoM_part_tmp * g_P_CoM_part_tmp_tmp * gb_P_CoM_part_tmp /
                  25.0) +
             44.0 * eb_P_CoM_part_tmp * fb_P_CoM_part_tmp * g_P_CoM_part_tmp_tmp /
                 25.0) -
            623.0 * eb_P_CoM_part_tmp * g_P_CoM_part_tmp_tmp * gb_P_CoM_part_tmp /
                100.0) *
               19.57) +
          (((((((((801.0 * f_P_CoM_part_tmp_tmp * jb_P_CoM_part_tmp / 100.0 +
                   24.0) -
                  49.0 * f_P_CoM_part_tmp_tmp * kb_P_CoM_part_tmp / 4.0) +
                 6.0 * std::sin(joint_angle[11])) +
                42.0 * std::cos(joint_angle[12] + joint_angle[13]) *
                    std::sin(joint_angle[11])) +
               i_P_CoM_part_tmp_tmp) +
              49.0 * lb_P_CoM_part_tmp * jb_P_CoM_part_tmp * g_P_CoM_part_tmp_tmp /
                  4.0) +
             801.0 * lb_P_CoM_part_tmp * g_P_CoM_part_tmp_tmp * kb_P_CoM_part_tmp /
                 100.0) -
            1053.0 * db_P_CoM_part_tmp * g_P_CoM_part_tmp_tmp * gb_P_CoM_part_tmp /
                50.0) +
           1053.0 * eb_P_CoM_part_tmp * fb_P_CoM_part_tmp * g_P_CoM_part_tmp_tmp /
               50.0) *
              36.53) /
         679.15;
     P_CoM[2] =
         ((((((((((((((((-(457.0 * P_CoM_part_tmp) / 100.0 * 4.26 + -5220.2345) +
                        (((b_P_CoM_part_tmp / 20.0 +
                           97.0 * P_CoM_part_tmp * d_P_CoM_part_tmp / 100.0) +
                          241.0 * P_CoM_part_tmp * c_P_CoM_part_tmp / 20.0) -
                         12.0 * P_CoM_part_tmp) *
                            23.0) +
                       (((129.0 * std::cos(joint_angle[1] + joint_angle[2]) *
                              P_CoM_part_tmp / 100.0 -
                          P_CoM_part_tmp *
                              (12.0 - 45.0 * std::sin(joint_angle[1]))) -
                         9.0 * b_P_CoM_part_tmp / 50.0) +
                        20.0 * std::sin(joint_angle[1] + joint_angle[2]) *
                            P_CoM_part_tmp) *
                           26.16) +
                      -(457.0 * std::sin(joint_angle[3] + 1.5707963267948966)) /
                          100.0 * 4.26) +
                     (((97.0 * j_P_CoM_part_tmp * h_P_CoM_part_tmp / 100.0 -
                        g_P_CoM_part_tmp / 20.0) -
                       241.0 * j_P_CoM_part_tmp * i_P_CoM_part_tmp / 20.0) -
                      12.0 * j_P_CoM_part_tmp) *
                         23.0) +
                    ((((((9.0 * g_P_CoM_part_tmp / 50.0 -
                          12.0 * std::cos(joint_angle[3])) -
                         129.0 * j_P_CoM_part_tmp * i_P_CoM_part_tmp *
                             l_P_CoM_part_tmp / 100.0) -
                        45.0 * j_P_CoM_part_tmp * i_P_CoM_part_tmp) +
                       129.0 * std::cos(joint_angle[3]) * h_P_CoM_part_tmp *
                           k_P_CoM_part_tmp / 100.0) -
                      20.0 * j_P_CoM_part_tmp * h_P_CoM_part_tmp *
                          l_P_CoM_part_tmp) -
                     20.0 * std::cos(joint_angle[3]) * k_P_CoM_part_tmp *
                         i_P_CoM_part_tmp) *
                        26.16) +
                   ((61.0 * P_CoM_part_tmp_tmp / 50.0 - 72.0) -
                    3.0 * b_P_CoM_part_tmp_tmp / 50.0) *
                       19.9) +
                  ((((-72.0 - b_P_CoM_part_tmp_tmp / 20.0) -
                     659.0 * P_CoM_part_tmp_tmp * c_P_CoM_part_tmp_tmp / 20.0) -
                    97.0 * P_CoM_part_tmp_tmp * m_P_CoM_part_tmp / 100.0) -
                   6.0 * P_CoM_part_tmp_tmp) *
                      23.0) +
                 ((((((179.0 * P_CoM_part_tmp_tmp * m_P_CoM_part_tmp *
                           n_P_CoM_part_tmp / 10.0 -
                       6.0 * std::cos(joint_angle[6])) -
                      72.0) -
                     e_P_CoM_part_tmp_tmp) -
                    179.0 * std::cos(joint_angle[6]) * c_P_CoM_part_tmp_tmp *
                        o_P_CoM_part_tmp / 10.0) -
                   172.0 * P_CoM_part_tmp_tmp * c_P_CoM_part_tmp_tmp *
                       n_P_CoM_part_tmp / 25.0) -
                  172.0 * std::cos(joint_angle[6]) * o_P_CoM_part_tmp *
                      m_P_CoM_part_tmp / 25.0) *
                     8.99) +
                ((((((((623.0 * std::cos(joint_angle[7] + joint_angle[8]) *
                            P_CoM_part_tmp_tmp * s_P_CoM_part_tmp / 100.0 -
                        3.0 * std::sin(joint_angle[6]) / 50.0) -
                       6.0 * std::cos(joint_angle[6])) -
                      42.0 * std::cos(joint_angle[7] + joint_angle[8]) *
                          P_CoM_part_tmp_tmp) -
                     e_P_CoM_part_tmp_tmp) -
                    72.0) -
                   44.0 * std::cos(joint_angle[7] + joint_angle[8]) *
                       P_CoM_part_tmp_tmp * t_P_CoM_part_tmp / 25.0) +
                  44.0 * std::sin(joint_angle[7] + joint_angle[8]) *
                      P_CoM_part_tmp_tmp * s_P_CoM_part_tmp / 25.0) +
                 623.0 * std::sin(joint_angle[7] + joint_angle[8]) *
                     P_CoM_part_tmp_tmp * t_P_CoM_part_tmp / 100.0) *
                    19.57) +
               (((((((((801.0 *
                            std::cos((joint_angle[7] + joint_angle[8]) -
                                     joint_angle[9]) *
                            P_CoM_part_tmp_tmp * x_P_CoM_part_tmp / 100.0 -
                        801.0 * w_P_CoM_part_tmp * b_P_CoM_part_tmp_tmp / 100.0) -
                       49.0 * b_P_CoM_part_tmp_tmp * x_P_CoM_part_tmp / 4.0) -
                      6.0 * std::cos(joint_angle[6])) -
                     42.0 * std::cos(joint_angle[7] + joint_angle[8]) *
                         std::cos(joint_angle[6])) -
                    e_P_CoM_part_tmp_tmp) -
                   49.0 *
                       std::cos((joint_angle[7] + joint_angle[8]) -
                                joint_angle[9]) *
                       P_CoM_part_tmp_tmp * w_P_CoM_part_tmp / 4.0) -
                  72.0) -
                 1053.0 * std::cos(joint_angle[7] + joint_angle[8]) *
                     P_CoM_part_tmp_tmp * t_P_CoM_part_tmp / 50.0) +
                1053.0 * std::sin(joint_angle[7] + joint_angle[8]) *
                    P_CoM_part_tmp_tmp * s_P_CoM_part_tmp / 50.0) *
                   36.53) +
              ((61.0 * f_P_CoM_part_tmp_tmp / 50.0 - 72.0) +
               3.0 * g_P_CoM_part_tmp_tmp / 50.0) *
                  19.9) +
             ((((g_P_CoM_part_tmp_tmp / 20.0 - 72.0) -
                659.0 * f_P_CoM_part_tmp_tmp * h_P_CoM_part_tmp_tmp / 20.0) +
               97.0 * f_P_CoM_part_tmp_tmp * ab_P_CoM_part_tmp / 100.0) -
              6.0 * f_P_CoM_part_tmp_tmp) *
                 23.0) +
            ((((((179.0 * f_P_CoM_part_tmp_tmp * ab_P_CoM_part_tmp *
                      cb_P_CoM_part_tmp / 10.0 -
                  6.0 * std::cos(joint_angle[11])) -
                 72.0) -
                j_P_CoM_part_tmp_tmp) -
               179.0 * std::cos(joint_angle[11]) * h_P_CoM_part_tmp_tmp *
                   bb_P_CoM_part_tmp / 10.0) +
              172.0 * f_P_CoM_part_tmp_tmp * h_P_CoM_part_tmp_tmp *
                  cb_P_CoM_part_tmp / 25.0) +
             172.0 * std::cos(joint_angle[11]) * bb_P_CoM_part_tmp *
                 ab_P_CoM_part_tmp / 25.0) *
                8.99) +
           ((((((((3.0 * std::sin(joint_angle[11]) / 50.0 - 72.0) -
                  6.0 * std::cos(joint_angle[11])) -
                 42.0 * std::cos(joint_angle[12] + joint_angle[13]) *
                     f_P_CoM_part_tmp_tmp) -
                j_P_CoM_part_tmp_tmp) +
               623.0 * std::cos(joint_angle[12] + joint_angle[13]) *
                   f_P_CoM_part_tmp_tmp * fb_P_CoM_part_tmp / 100.0) +
              44.0 * std::cos(joint_angle[12] + joint_angle[13]) *
                  f_P_CoM_part_tmp_tmp * gb_P_CoM_part_tmp / 25.0) -
             44.0 * std::sin(joint_angle[12] + joint_angle[13]) *
                 f_P_CoM_part_tmp_tmp * fb_P_CoM_part_tmp / 25.0) +
            623.0 * std::sin(joint_angle[12] + joint_angle[13]) *
                f_P_CoM_part_tmp_tmp * gb_P_CoM_part_tmp / 100.0) *
               19.57) +
          (((((((((801.0 * jb_P_CoM_part_tmp * g_P_CoM_part_tmp_tmp / 100.0 -
                   72.0) -
                  49.0 * g_P_CoM_part_tmp_tmp * kb_P_CoM_part_tmp / 4.0) -
                 6.0 * std::cos(joint_angle[11])) -
                42.0 * std::cos(joint_angle[12] + joint_angle[13]) *
                    std::cos(joint_angle[11])) -
               j_P_CoM_part_tmp_tmp) -
              49.0 *
                  std::cos((joint_angle[12] + joint_angle[13]) - joint_angle[14]) *
                  f_P_CoM_part_tmp_tmp * jb_P_CoM_part_tmp / 4.0) -
             801.0 *
                 std::cos((joint_angle[12] + joint_angle[13]) - joint_angle[14]) *
                 f_P_CoM_part_tmp_tmp * kb_P_CoM_part_tmp / 100.0) +
            1053.0 * std::cos(joint_angle[12] + joint_angle[13]) *
                f_P_CoM_part_tmp_tmp * gb_P_CoM_part_tmp / 50.0) -
           1053.0 * std::sin(joint_angle[12] + joint_angle[13]) *
               f_P_CoM_part_tmp_tmp * fb_P_CoM_part_tmp / 50.0) *
              36.53) /
         679.15;
}

// End of code generation (COM_calc.cpp)
