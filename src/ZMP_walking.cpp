/*******************************************************************************
 * This prgoram is written for upgraded ROBOTIS MINI with the Orange Pi. 
 * To run: 
 * 
 * in a terminal: 
 * $ roscore 
 * 
 * in a second terminal: 
 * $ rosrun dynamixel_sdk_examples sync_write_node 
*******************************************************************************/

#include "ros/ros.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <cmath>
#include "gaitplanning.h"
#include "Foot_planning.h"
#include <fcntl.h>
#include <termios.h>
#include <sensor_msgs/Imu.h>
#define STDIN_FILENO 0

using namespace dynamixel;

// Control table address
#define ADDR_TORQUE_ENABLE 24
#define ADDR_GOAL_POSITION 30
#define ADDR_PRESENT_POSITION 37
#define ADDR_P_GAIN 29
#define ADDR_I_GAIN 28
#define ADDR_D_GAIN 27
#define ADDR_MOVING_SPEED 32
#define ADDR_320_VOLT 45
#define ADDR_320_ReturnDelay 5
#define PROTOCOL_VERSION 2.0       // Default Protocol version of DYNAMIXEL X series.
#define BAUDRATE 1000000           // Default Baudrate of DYNAMIXEL XL 320
#define DEVICE_NAME "/dev/ttyUSB0" // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command
#define PI 3.1415
#define ESC_ASCII_VALUE 0x1b
#define MILLION 1000000L

double t = 0.0, t_elapsed = 0.0, t_loop = 0.0;
char input = 0;
int t_init = 5.0;
long t_sleep = 0;
uint64_t diff1, diff2;
long tick = 0;
struct timespec t0, t_start, t_end1, t_end2;
double x_RH0 = 0.0, y_RH0 = -174.00, z_RH0 = -12.00, x_LH0 = 0.0, y_LH0 = 174.0, z_LH0 = -12.0;
double x_RF0 = -5.0, y_RF0 = -(33.0 + 0.0), z_RF0 = -196.0, Roll_RF0 = 0.0, Pitch_RF0 = 0.0;
double x_LF0 = -5.0, y_LF0 = 33.0 + 0.0, z_LF0 = -196.0, Roll_LF0 = 0.0, Pitch_LF0 = 0.0;
double x_RH, y_RH = 0.0, z_RH = 0.0, x_LH = 0.0, y_LH = 0.0, z_LH = 0.0;
double x_RF = 0.0, y_RF = 0.0, z_RF = 0.0, Roll_RF = 0.0, Pitch_RF = -0.02, x_LF = 0.0, y_LF = 0.0, z_LF = 0.0, Roll_LF = 0.0, Pitch_LF = -0.02;
double z0 = -176.0, z_h = 0.0;
double t_step = 1.0, x_size = 0.0, y_size = 0.0;
int n_step = 0;
double ds_time = 0.5;
double imu_roll_raw = 0.0, imu_roll_ofst = 0.0, imu_pitch_raw = 0.0, ax_raw = 0.0, ay_raw = 0.0, az_raw = 0.0, ax = 0.0, ay = 0.0, az = 0.0;
double imu_pitch_old1 = 0.0, imu_pitch_old2 = 0.0, imu_roll_old1 = 0.0, imu_roll_old2 = 0.0, imu_roll_init = 0.0, imu_pitch_init = 0.0, imu_roll = 0.0, imu_pitch = 0.0;
double ax_old1 = 0.0, ay_old1 = 0.0, az_old1, ax_old2 = 0.0, ay_old2 = 0.0, az_old2 = 0.0, ay_old3 = 0.0, ay_old4 = 0.0;
double pitch_ankle_gain, roll_ankle_gain, pitch_disp_gain, roll_disp_gain, ay_gain, ax_gain;
std::vector<double> joint_angle(16);
std::vector<double> P_CoM(3);
coder::array<double, 1U> com_x;
coder::array<double, 1U> com_y;
coder::array<double, 1U> ZMP_x_amp;
coder::array<double, 1U> ZMP_y_amp;

PortHandler *portHandler = PortHandler::getPortHandler(DEVICE_NAME);
PacketHandler *packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, 4);
GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, 4);

void IK_RH(float x, float y, float z, std::vector<uint16_t> &positions);
void IK_LH(float x, float y, float z, std::vector<uint16_t> &positions);
void IK_RF(float x, float y, float z, float th_r, float th_p, std::vector<uint16_t> &positions);
void IK_LF(float x, float y, float z, float th_r, float th_p, std::vector<uint16_t> &positions);
void COM_calc(std::vector<double> joint_angle, std::vector<double> &P_CoM);
int getch();
int kbhit(void);
char getKey();
void chatterCallback_imu(const sensor_msgs::Imu::ConstPtr &msg);
// void positionCallback(geometry_msgs::Point msg);
void initPose();

int main(int argc, char **argv)
{
     ros::init(argc, argv, "inplace_node");
     ros::NodeHandle nh;
     uint8_t dxl_error = 0;
     int dxl_comm_result = COMM_TX_FAIL;
     ros::WallTime start_t, end_t, t0;
     ros::Rate loop_rate(200);
     if (!portHandler->openPort())
     {
          ROS_ERROR("Failed to open the port!");
          return -1;
     }

     if (!portHandler->setBaudRate(BAUDRATE))
     {
          ROS_ERROR("Failed to set the baudrate!");
          return -1;
     }

     //subscribes to ball_pos
     // ros::Subscriber sub = nh.subscribe("/ball_pos", 1, positionCallback);
     for (int i = 1; i < 17; i++)
     {
          packetHandler->write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, 0, &dxl_error);
          packetHandler->write1ByteTxRx(portHandler, i, ADDR_320_ReturnDelay, 50, &dxl_error);
          packetHandler->write1ByteTxRx(portHandler, i, ADDR_320_VOLT, 50, &dxl_error);
          packetHandler->write1ByteTxRx(portHandler, i, ADDR_P_GAIN, 15, &dxl_error);
          packetHandler->write1ByteTxRx(portHandler, i, ADDR_I_GAIN, 15, &dxl_error);
          packetHandler->write1ByteTxRx(portHandler, i, ADDR_D_GAIN, 0, &dxl_error);
          packetHandler->write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, 1, &dxl_error);
     }

     //arm motors pid value set
     for (int i = 1; i < 7; i++)
     {
          packetHandler->write1ByteTxRx(portHandler, i, ADDR_P_GAIN, 20, &dxl_error);
          packetHandler->write1ByteTxRx(portHandler, i, ADDR_I_GAIN, 5, &dxl_error);
          packetHandler->write1ByteTxRx(portHandler, i, ADDR_D_GAIN, 10, &dxl_error);
     }

     //leg motors pid value set
     for (int i = 7; i < 17; i++)
     {
          packetHandler->write1ByteTxRx(portHandler, i, ADDR_P_GAIN, 20, &dxl_error);
          packetHandler->write1ByteTxRx(portHandler, i, ADDR_I_GAIN, 0, &dxl_error);
          packetHandler->write1ByteTxRx(portHandler, i, ADDR_D_GAIN, 10, &dxl_error);
     }

     initPose();
     ros::Duration(0.5).sleep();
     //arm motors pid value set
     for (int i = 1; i < 7; i++)
     {
          packetHandler->write1ByteTxRx(portHandler, i, ADDR_P_GAIN, 50, &dxl_error);
          packetHandler->write1ByteTxRx(portHandler, i, ADDR_I_GAIN, 5, &dxl_error);
          packetHandler->write1ByteTxRx(portHandler, i, ADDR_D_GAIN, 1, &dxl_error);
     }

     // leg motors pid value set
     for (int i = 7; i < 17; i++)
     {
          packetHandler->write1ByteTxRx(portHandler, i, ADDR_P_GAIN, 140, &dxl_error);
          packetHandler->write1ByteTxRx(portHandler, i, ADDR_I_GAIN, 20, &dxl_error);
          packetHandler->write1ByteTxRx(portHandler, i, ADDR_D_GAIN, 10, &dxl_error);
     }
     // Step Parameters
     nh.getParam("/x_size", x_size);
     nh.getParam("/y_size", y_size);
     nh.getParam("/n_step", n_step);
     nh.getParam("/t_step", t_step);
     nh.getParam("/ds_time", ds_time);
     nh.getParam("/z_h", z_h);
     nh.getParam("/pitch_ankle_gain", pitch_ankle_gain);
     nh.getParam("/roll_ankle_gain", roll_ankle_gain);
     nh.getParam("/pitch_disp_gain", pitch_disp_gain);
     nh.getParam("/roll_disp_gain", roll_disp_gain);
     nh.getParam("/ay_gain", ay_gain);
     nh.getParam("/ax_gain", ax_gain);

     //  x_size = 15;
     // y_size = 33;
     // n_step = 4;
     // t_step = 5.0;
     // ds_time = 0.1;
     // z_h = 30.0;
     printf("Calculating ZMP and CoM trajectories...\n");
     gaitplanning(x_size, y_size, t_step, n_step, ds_time, com_x, com_y, ZMP_x_amp, ZMP_y_amp);

     ros::Subscriber sub = nh.subscribe("imu/data", 1000, chatterCallback_imu);

     while (ros::ok())
     {
          std::cout << "Press any key to start\n"
                    << std::endl;
          printf("Press any key to continue! (or press ESC to quit!)\n");
          if (getch() == ESC_ASCII_VALUE)
               break;

          t_elapsed = 0;
          t0 = ros::WallTime::now();

          while (t_elapsed < (n_step + 3) * t_step)
          {
               start_t = ros::WallTime::now();
               input = getKey();
               if ((input == ESC_ASCII_VALUE))
               {
                    break;
               }
               ROS_INFO("Loop: %2.2f ms  Elapsed Time: %4.2f s | ax: %2.2f  ay: %2.2f  az: %2.2f   | pitch: %2.2f    roll: %2.2f   ", t_loop, t_elapsed, ax, ay, az, imu_pitch, imu_roll);
               std::vector<uint16_t> posis(16);
               ros::NodeHandle nh;
               // ros::Publisher chatter_pub = nh.advertise<std_msgs::Float32MultiArray>("joint_angles", 1000);
               // std_msgs::Float32MultiArray msg_angle;
               // msg_angle.data.clear();

               // Motion Generation
               x_RH = x_RH0;
               x_LH = x_LH0;
               y_RH = y_RH0 + 100.0;
               y_LH = y_LH0 - 100.0;
               z_RH = z_RH0 - 90.0;
               z_LH = z_LH0 - 90.0;

               Foot_planning(t_elapsed, t_step, ZMP_x_amp, ZMP_y_amp, com_x, com_y, ds_time, n_step, z_h, z0, x_RF0, y_LF0, &x_RF, &y_RF, &z_RF, &x_LF, &y_LF, &z_LF);

               Pitch_RF = -0.00 + pitch_ankle_gain * imu_pitch;
               Pitch_LF = -0.00 + pitch_ankle_gain * imu_pitch;
               Roll_RF = roll_ankle_gain * imu_roll;
               Roll_LF = roll_ankle_gain * imu_roll;
               x_LF = x_LF + ax_gain * ax;
               x_RF = x_RF + ax_gain * ax;

               y_LF = y_LF + roll_disp_gain * (imu_roll+(0.4)) + ay_gain * ay ;
               y_RF = y_RF + roll_disp_gain * (imu_roll+(0.4)) + ay_gain * ay ;
               IK_RH(x_RH, y_RH, z_RH, posis);
               IK_LH(x_LH, y_LH, z_LH, posis);
               IK_RF(x_RF, y_RF, z_RF, Roll_RF, Pitch_RF, posis);
               IK_LF(x_LF, y_LF, z_LF, Roll_LF, Pitch_LF, posis);

               //write positions to motors
               for (int i = 0; i < 16; i++)
               {
                    uint8_t param_goal_position[4];
                    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD((uint16_t)posis[i]));
                    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD((uint16_t)posis[i]));
                    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD((uint16_t)posis[i]));
                    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD((uint16_t)posis[i]));
                    groupSyncWrite.addParam((uint8_t)(i + 1), param_goal_position);
               }

               for (int i = 0; i < 16; i++)
               {
                    joint_angle[i] = ((double)posis[i] * 0.00511826781 - 512 * 0.00511826781);
                    // msg_angle.data.push_back(joint_angle[i]);
               }
               COM_calc(joint_angle, P_CoM);

               groupSyncWrite.txPacket();
               groupSyncWrite.clearParam();

               tick++; // For general tick
                       //simple IMU filter
               imu_roll = (imu_roll_raw - imu_roll_init + imu_roll_old1 + imu_roll_old2) / 3.0;
               imu_roll_old2 = imu_roll_old1;
               imu_roll_old1 = imu_roll;

               imu_pitch = (imu_pitch_raw - imu_pitch_init + imu_pitch_old1 + imu_pitch_old2) / 3.0;
               imu_pitch_old2 = imu_pitch_old1;
               imu_pitch_old1 = imu_pitch;

               ax = (ax_raw + ax_old1 + ax_old2) / 3.0;
               ax_old2 = ax_old1;
               ax_old1 = ax;

               ay = (ay_raw + ay_old1 + ay_old2 + ay_old3 + ay_old4) / 5.0;
               ay_old4 = ay_old3;
               ay_old3 = ay_old2;
               ay_old2 = ay_old1;
               ay_old1 = ay;

               az = (az_raw + az_old1 + az_old2) / 3.0;
               az_old2 = az_old1;
               az_old1 = az;

               loop_rate.sleep();
               ros::spinOnce();

               // Measure Loop time
               end_t = ros::WallTime::now();
               t_elapsed = (end_t - t0).toNSec() * 1e-9;
               t_loop = (end_t - start_t).toNSec() * 1e-6;
          }
     }

     for (int i = 1; i < 17; i++)
     {
          packetHandler->write1ByteTxRx(portHandler, i, ADDR_P_GAIN, 20, &dxl_error);
          packetHandler->write1ByteTxRx(portHandler, i, ADDR_I_GAIN, 0, &dxl_error);
          packetHandler->write1ByteTxRx(portHandler, i, ADDR_D_GAIN, 10, &dxl_error);
     }

     initPose();

     // ros::spin();
     ros::shutdown();

     //disable torque
     for (int i = 1; i < 17; i++)
     {
          packetHandler->write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, 0, &dxl_error);
     }

     portHandler->closePort();
     return 0;
}

void chatterCallback_imu(const sensor_msgs::Imu::ConstPtr &msg)
{
     double qx = msg->orientation.x;
     double qy = msg->orientation.y;
     double qz = msg->orientation.z;
     double qw = msg->orientation.w;
     double ax_imu = msg->linear_acceleration.x;
     double ay_imu = msg->linear_acceleration.y;
     double az_imu = msg->linear_acceleration.z;

     double sinr_cosp = 2 * (qw * qx + qy * qz);
     double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);

     double sinp = 2 * (qw * qy - qz * qx);
     ax_raw = ax_imu;
     ay_raw = ay_imu;
     az_raw = az_imu;
     imu_roll_raw = -std::atan(sinr_cosp / cosr_cosp) - 0.1;
     imu_pitch_raw = std::asin(sinp) + 0.1;
}

void initPose()
{
     std::vector<uint16_t> posis(16, 512);
     //initial pose
     x_RH = x_RH0;
     x_LH = x_LH0;
     y_RH = y_RH0 + 100.0;
     y_LH = y_LH0 - 100.0;
     z_RH = z_RH0 - 100.0;
     z_LH = z_LH0 - 100.0;

     x_RF = x_RF0;
     y_RF = y_RF0;
     z_RF = z0;

     x_LF = x_LF0;
     y_LF = y_LF0;
     z_LF = z0;

     IK_RH(x_RH, y_RH, z_RH, posis);
     IK_LH(x_LH, y_LH, z_LH, posis);
     IK_RF(x_RF, y_RF, z_RF, Roll_RF, Pitch_RF, posis);
     IK_LF(x_LF, y_LF, z_LF, Roll_LF, Pitch_LF, posis);
     // printf("x_RF: %3.2f   x_LF: %3.2f   y_RF: %3.2f   y_LF: %3.2f   z_RF: %3.2f   z_LF: %3.2f\n", x_RF, x_LF, y_RF, y_LF, z_RF, z_LF);
     for (int i = 0; i < 16; i++)
     {
          uint8_t param_goal_position[4];
          param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD((uint16_t)posis[i]));
          param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD((uint16_t)posis[i]));
          param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD((uint16_t)posis[i]));
          param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD((uint16_t)posis[i]));
          groupSyncWrite.addParam((uint8_t)(i + 1), param_goal_position);
     }

     groupSyncWrite.txPacket();
     groupSyncWrite.clearParam();
}

void IK_RH(float x, float y, float z, std::vector<uint16_t> &positions)
{

     float L_sh = 39.0; // Origin to arm roll joint
     float L_a1 = 18.0; // Shoulder bracket horizontal distance
     float L_a2 = 12.0; // Shoulder bracket vertical distance
     float L_a3 = 45.0; // Upper arm length
     float L_a4 = 72.0; // Lower arm length

     float x_0 = 0.0;
     float y_0 = -174.0;
     float z_0 = -12.0;

     float th1 = 0.0, th3 = 0.0, th5 = 0.0;
     int DXL_POS_1 = 512, DXL_POS_3 = 512, DXL_POS_5 = 512;

     // Right Arm
     float x_RH = x;
     float y_RH = y;
     float z_RH = z;
     float x_RH0 = x_RH;
     float y_RH0 = y_RH + (L_sh + L_a1);
     float z_RH0 = z_RH;

     th1 = -atan(x_RH0 / (z_RH0 + 0.00001));

     float R1_RH = sqrt((x_RH0 - L_a2 * sin(th1)) * (x_RH0 - L_a2 * sin(th1)) + y_RH0 * y_RH0 + (z_RH0 + L_a2 * cos(th1)) * (z_RH0 + L_a2 * cos(th1)));

     if (R1_RH > 117)
     {
          R1_RH = 117;
     }
     else
     {
          R1_RH = R1_RH;
     }

     float alpha_RH = acos((L_a3 * L_a3 + L_a4 * L_a4 - R1_RH * R1_RH) / (2 * L_a3 * L_a4));

     // Elbow Joint angle
     th5 = -PI + alpha_RH;
     float R2_RH = sqrt((x_RH0 - L_a2 * sin(th1)) * (x_RH0 - L_a2 * sin(th1)) + (z_RH0 + L_a2 * cos(th1)) * (z_RH0 + L_a2 * cos(th1)));

     // Shoulder Joint angle
     if (z_RH > 0)
     {
          th3 = PI / 2 + (atan(y_RH0 / R2_RH) + acos((L_a3 * L_a3 + R1_RH * R1_RH - L_a4 * L_a4) / (2 * L_a3 * R1_RH)));
     }
     else
     {
          th3 = -PI / 2 + (-atan(y_RH0 / R2_RH) + acos((L_a3 * L_a3 + R1_RH * R1_RH - L_a4 * L_a4) / (2 * L_a3 * R1_RH)));
     }

     DXL_POS_1 = (512 + (int)(th1 * 195.3786));
     DXL_POS_3 = (512 + (int)(th3 * 195.3786));
     DXL_POS_5 = (512 + (int)(th5 * 195.3786));

     positions[0] = (uint16_t)DXL_POS_1;
     positions[2] = (uint16_t)DXL_POS_3;
     positions[4] = (uint16_t)DXL_POS_5;
}

void IK_LH(float x, float y, float z, std::vector<uint16_t> &positions)
{
     float L_sh = 39.0; // Origin to arm roll joint
     float L_a1 = 18.0; // Shoulder bracket horizontal distance
     float L_a2 = 12.0; // Shoulder bracket vertical distance
     float L_a3 = 45.0; // Upper arm length
     float L_a4 = 72.0; // Lower arm length

     // Position of Left hand in Initial Pose (all the left arm motor angle == 0)

     float x_0 = 0.0;
     float y_0 = 174.0;
     float z_0 = -12.0;

     float th2 = 0.0, th4 = 0.0, th6 = 0.0;
     int DXL_POS_2 = 0, DXL_POS_4 = 0, DXL_POS_6 = 0;
     // Left Arm
     float x_LH = x;
     float y_LH = y;
     float z_LH = z;

     float x_LH0 = x_LH;
     float y_LH0 = y_LH - (L_sh + L_a1);
     float z_LH0 = z_LH;

     th2 = atan(x_LH0 / (z_LH0 + 0.00001));

     // Range of XL320 motor -150 to 150 deg

     float R1_LH = sqrt((x_LH0 - L_a2 * sin(th2)) * (x_LH0 - L_a2 * sin(th2)) + y_LH0 * y_LH0 + (z_LH0 + L_a2 * cos(th2)) * (z_LH0 + L_a2 * cos(th2)));

     if (R1_LH > 117)
     {
          R1_LH = 117;
     }
     else
     {
          R1_LH = R1_LH;
     }

     float alpha_LH = acos((L_a3 * L_a3 + L_a4 * L_a4 - R1_LH * R1_LH) / (2 * L_a3 * L_a4));

     // Elbow Joint angle
     th6 = PI - alpha_LH;

     float R2_LH = sqrt((x_LH0 - L_a2 * sin(th2)) * (x_LH0 - L_a2 * sin(th2)) + (z_LH0 + L_a2 * cos(th2)) * (z_LH0 + L_a2 * cos(th2)));

     // Shoulder Joint angle
     if (z_LH0 > 0)
     {
          th4 = -((atan(R2_LH / y_LH0) + acos((L_a3 * L_a3 + R1_LH * R1_LH - L_a4 * L_a4) / (2 * L_a3 * R1_LH))));
     }
     else
     {
          th4 = -(-PI / 2 + (atan(y_LH0 / R2_LH) + acos((L_a3 * L_a3 + R1_LH * R1_LH - L_a4 * L_a4) / (2 * L_a3 * R1_LH))));
     }

     DXL_POS_2 = (512 + (int)(th2 * 195.3786));
     DXL_POS_4 = (512 + (int)(th4 * 195.3786));
     DXL_POS_6 = (512 + (int)(th6 * 195.3786));

     positions[1] = (uint16_t)DXL_POS_2;
     positions[3] = (uint16_t)DXL_POS_4;
     positions[5] = (uint16_t)DXL_POS_6;
}

void IK_RF(float x, float y, float z, float th_r, float th_p, std::vector<uint16_t> &positions)
{
     float L_by = 24.0; // Origin to pelvis vertical length
     float L_bz = 72.0; // Pelvis horizontal length
     float L_bx = 15.0; // Shoulder joint axis to Leg center (On Sagittal Plane)
     float L_l1 = 6.0;  // Pelvis Roll axis to pitch axis
     float L_l2 = 45.0; // Thigh Length
     float L_l3 = 42.0; // Shank Length
     float L_l4 = 31.0; // Ankle Length
     float L_f = 9.0;   // Foot horizontal length

     float x_0 = 15.0;
     float y_0 = -24.0;
     float z_0 = -196.0;
     float x_RF = 0;
     float y_RF = 0;
     float z_RF = 0;

     float th7 = 0.0, th9 = 0.0, th11 = 0.0, th13 = 0.0, th15 = 0.0;
     int DXL_POS_7 = 512, DXL_POS_9 = 512, DXL_POS_11 = 512, DXL_POS_13 = 512, DXL_POS_15 = 512;

     x_RF = x;
     y_RF = y;
     z_RF = z;

     float pos_RF[3] = {
         -L_bz - z_RF - L_f * sin(th_r) - L_l4 * cos(th_p) * cos(th_r),
         L_by + y_RF + L_f * cos(th_r) - L_l4 * cos(th_p) * sin(th_r),
         x_RF - L_bx + L_l4 * sin(th_p)};

     th7 = atan(pos_RF[1] / pos_RF[0]);
     float R1_RF = sqrt((pos_RF[0] - L_l1 * cos(th7)) * (pos_RF[0] - L_l1 * cos(th7)) + (pos_RF[1] - L_l1 * sin(th7)) * (pos_RF[1] - L_l1 * sin(th7)) + pos_RF[2] * pos_RF[2]);
     float alpha_RF = acos((L_l2 * L_l2 + L_l3 * L_l3 - R1_RF * R1_RF) / (2 * L_l2 * L_l3));
     th11 = PI - alpha_RF;
     float R2_RF = sqrt((pos_RF[0] - L_l1 * cos(th7)) * (pos_RF[0] - L_l1 * cos(th7)) + (pos_RF[1] - L_l1 * sin(th7)) * (pos_RF[1] - L_l1 * sin(th7)));

     th9 = -((atan(pos_RF[2] / R2_RF) + acos((L_l2 * L_l2 + R1_RF * R1_RF - L_l3 * L_l3) / (2 * L_l2 * R1_RF))));

     th13 = -asin(cos(th9 + th11) * cos(th_r) * cos(th7) * sin(th_p) - sin(th9 + th11) * cos(th_p) + cos(th9 + th11) * sin(th_p) * sin(th_r) * sin(th7));
     th15 = -asin(sin(th_r - th7) * cos(th_p));

     DXL_POS_7 = (512 + (int)(th7 * 195.3786));
     DXL_POS_9 = (482 + (int)(th9 * 195.3786));
     DXL_POS_11 = (512 + (int)(th11 * 195.3786));
     DXL_POS_13 = (512 + (int)(th13 * 195.3786));
     DXL_POS_15 = (512 + (int)(th15 * 195.3786));

     positions[6] = (uint16_t)DXL_POS_7;
     positions[8] = (uint16_t)DXL_POS_9;
     positions[10] = (uint16_t)DXL_POS_11;
     positions[12] = (uint16_t)DXL_POS_13;
     positions[14] = (uint16_t)DXL_POS_15;
}

void IK_LF(float x, float y, float z, float th_r, float th_p, std::vector<uint16_t> &positions)
{

     float L_by = 24.0; // Origin to pelvis vertical length
     float L_bz = 72.0; // Pelvis horizontal length
     float L_bx = 15.0; // Shoulder joint axis to Leg center (On Sagittal Plane)
     float L_l1 = 6.0;  // Pelvis Roll axis to pitch axis
     float L_l2 = 45.0; // Thigh Length
     float L_l3 = 42.0; // Shank Length
     float L_l4 = 31.0; // Ankle Length
     float L_f = 9.0;   // Foot horizontal length

     float x_0 = 15.0;
     float y_0 = 24.0;
     float z_0 = -196.0;
     float x_LF = 0;
     float y_LF = 0;
     float z_LF = 0;

     float th8 = 0.0, th10 = 0.0, th12 = 0.0, th14 = 0.0, th16 = 0.0;
     int DXL_POS_8 = 512, DXL_POS_10 = 512, DXL_POS_12 = 512, DXL_POS_14 = 512, DXL_POS_16 = 512;

     x_LF = x;
     y_LF = y;
     z_LF = z;

     float pos_LF[3] = {
         L_f * sin(th_r) - z_LF - L_bz - L_l4 * cos(th_p) * cos(th_r),
         y_LF - L_by - L_f * cos(th_r) - L_l4 * cos(th_p) * sin(th_r),
         x_LF - L_bx + L_l4 * sin(th_p)};

     th8 = atan(pos_LF[1] / pos_LF[0]);
     float R1_LF = sqrt((pos_LF[0] - L_l1 * cos(th8)) * (pos_LF[0] - L_l1 * cos(th8)) + (pos_LF[1] - L_l1 * sin(th8)) * (pos_LF[1] - L_l1 * sin(th8)) + pos_LF[2] * pos_LF[2]);
     float alpha_LF = acos((L_l2 * L_l2 + L_l3 * L_l3 - R1_LF * R1_LF) / (2 * L_l2 * L_l3));

     th12 = -PI + alpha_LF;
     float R2_LF = sqrt((pos_LF[0] - L_l1 * cos(th8)) * (pos_LF[0] - L_l1 * cos(th8)) + (pos_LF[1] - L_l1 * sin(th8)) * (pos_LF[1] - L_l1 * sin(th8)));

     th10 = (atan(pos_LF[2] / R2_LF) + acos((L_l2 * L_l2 + R1_LF * R1_LF - L_l3 * L_l3) / (2 * L_l2 * R1_LF)));

     th14 = -acos(cos(th10 + th12) * cos(th_p) - sin(th10 + th12) * cos(th_r) * cos(th8) * sin(th_p) - sin(th10 + th12) * sin(th_p) * sin(th_r) * sin(th8));
     th16 = -asin(sin(th_r - th8) * cos(th_p));

     // printf("%f   %f   %f   %f   %f\n", th8, th10, th12, th14, th16);

     DXL_POS_8 = (512 + (int)(th8 * 195.3786));
     DXL_POS_10 = (542 + (int)(th10 * 195.3786));
     DXL_POS_12 = (512 + (int)(th12 * 195.3786));
     DXL_POS_14 = (512 + (int)(th14 * 195.3786));
     DXL_POS_16 = (512 + (int)(th16 * 195.3786));

     positions[7] = (uint16_t)DXL_POS_8;
     positions[9] = (uint16_t)DXL_POS_10;
     positions[11] = (uint16_t)DXL_POS_12;
     positions[13] = (uint16_t)DXL_POS_14;
     positions[15] = (uint16_t)DXL_POS_16;
}

void COM_calc(std::vector<double> joint_angle, std::vector<double> &P_CoM)
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

int getch()
{
     struct termios oldt, newt;
     int ch;
     tcgetattr(STDIN_FILENO, &oldt);
     newt = oldt;
     newt.c_lflag &= ~(ICANON | ECHO);
     tcsetattr(STDIN_FILENO, TCSANOW, &newt);
     ch = getchar();
     tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
     return ch;
}

int kbhit(void)
{
     struct termios oldt, newt;
     int ch;
     int oldf;

     tcgetattr(STDIN_FILENO, &oldt);
     newt = oldt;
     newt.c_lflag &= ~(ICANON | ECHO);
     tcsetattr(STDIN_FILENO, TCSANOW, &newt);
     oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
     fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

     ch = getchar();

     tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
     fcntl(STDIN_FILENO, F_SETFL, oldf);

     if (ch != EOF)
     {
          ungetc(ch, stdin);
          return 1;
     }
     return 0;
}

char getKey()
{
     if (kbhit())
     {
          return getch();
     }
     return '\0';
}
