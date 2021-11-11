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
#include <iostream>
#include "std_msgs/String.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Point.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <cmath>

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
#define MILLION 1000000L

double t = 0.0, t_elapsed = 0.0, t_cubic = 0.0;
int t_init = 5.0;
float t_loop1 = 0.0, t_loop2 = 0.0;
long t_sleep = 0;
uint64_t diff1, diff2;
long tick = 0;
struct timespec t0, t_start, t_end1, t_end2;
float x_RH0 = 0.0, y_RH0 = -174.00, z_RH0 = -12.00, x_LH0 = 0.0, y_LH0 = 174.0, z_LH0 = -12.0;
float x_RF0 = 15.0, y_RF0 = -33.0, z_RF0 = -196.0, Roll_RF0 = 0.0, Pitch_RF0 = 0.0;
float x_LF0 = 15.0, y_LF0 = 33.0, z_LF0 = -196.0, Roll_LF0 = 0.0, Pitch_LF0 = 0.0;
float x_RH, y_RH = 0.0, z_RH = 0.0, x_LH = 0.0, y_LH = 0.0, z_LH = 0.0, x_RF = 0.0, y_RF = 0.0, z_RF = 0.0, Roll_RF = 0.0, Pitch_RF = 0.0, x_LF = 0.0, y_LF = 0.0, z_LF = 0.0, Roll_LF = 0.0, Pitch_LF = 0.0;
float y_amp = 0.0;
float z_min = 10.0;
float z_height = 0.0;
float step_time = 0.0;
float tau = 0.0;
float duration = 0.0;
float t_lift_1 = 0.0;
float t_lift_2 = 0.0;
std::vector<float> joint_angle(16);

PortHandler *portHandler = PortHandler::getPortHandler(DEVICE_NAME);
PacketHandler *packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, 4);
GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, 4);

void IK_RH(float x, float y, float z, std::vector<uint16_t> &positions);
void IK_LH(float x, float y, float z, std::vector<uint16_t> &positions);
void IK_RF(float x, float y, float z, float th_r, float th_p, std::vector<uint16_t> &positions);
void IK_LF(float x, float y, float z, float th_r, float th_p, std::vector<uint16_t> &positions);
void walk(long loops);
void initPose();
void positionCallback(geometry_msgs::Point msg);

int main(int argc, char **argv)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

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
  ros::init(argc, argv, "inplace_node");
  ros::NodeHandle nh;
  //subscribes to ball_pos
  ros::Subscriber sub = nh.subscribe("/ball_pos", 1, positionCallback);
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

  initPose();

  //arm motors pid value set
  for (int i = 1; i < 7; i++)
  {
    packetHandler->write1ByteTxRx(portHandler, i, ADDR_P_GAIN, 55, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, i, ADDR_I_GAIN, 10, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, i, ADDR_D_GAIN, 1, &dxl_error);
  }

  //leg motors pid value set
  for (int i = 7; i < 17; i++)
  {
    packetHandler->write1ByteTxRx(portHandler, i, ADDR_P_GAIN, 120, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, i, ADDR_I_GAIN, 15, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, i, ADDR_D_GAIN, 50, &dxl_error);
  }
  x_LF0 = x_LF0 - 21;
  x_RF0 = x_RF0 - 21;
  initPose();
  ros::Duration(1).sleep();

  while (ros::ok())
  {
    clock_gettime(CLOCK_MONOTONIC, &t_start);
    // ROS_INFO("Step: %ld     t_loop: %2.3f   Elapsed Time: %3.3f   ", tick, t_loop2, t_elapsed);

    std::vector<uint16_t> posis(16, 512);
    ros::NodeHandle nh;
    ros::Publisher chatter_pub = nh.advertise<std_msgs::Float32MultiArray>("joint_angles", 1000);
    std_msgs::Float32MultiArray msg_angle;
    msg_angle.data.clear();

    // Motion Generation
    x_RH = x_RH0 + 50.0;
    x_LH = x_LH0 + 50.0;
    y_RH = y_RH0 + 100.0;
    y_LH = y_LH0 - 100.0;
    z_RH = z_RH0 - 90.0;
    z_LH = z_LH0 - 90.0;

    y_amp = 47.0;
    z_height = 0.0;
    step_time = 0.6;
    tau = 0.8;
    duration = step_time * tau;
    t_lift_1 = (step_time - duration) / 2.0;
    t_lift_2 = t_lift_1 + step_time;

    y_RF = y_RF0 - (y_amp / 2 * sin(2 * PI * t_elapsed / (2 * step_time)));
    y_LF = y_LF0 - (y_amp / 2 * sin(2 * PI * t_elapsed / (2 * step_time)));

    // x_RF = x_RF0 + (y_amp / 2 * sin(2 * PI * t_elapsed / (2 * step_time)));
    // x_LF = x_LF0 - (y_amp / 2 * sin(2 * PI * t_elapsed / (2 * step_time)));

    int step_num = ceil(t_elapsed / step_time);
    float time_cycle = fmod(t_elapsed, 2 * step_time);

    if ((time_cycle > t_lift_1) && (time_cycle < t_lift_1 + duration))
    {
      z_RF = z_RF0 + z_min + z_height / 2 * (1 - cos(2 * PI / (duration) * (time_cycle - t_lift_1)));
    }
    else
    {
      z_RF = z_RF0 + z_min;
    }
    if ((time_cycle > t_lift_2) && (time_cycle < t_lift_2 + duration))
    {
      z_LF = z_LF0 + z_min + z_height / 2 * (1 - cos(2 * PI / (duration) * (time_cycle - t_lift_2)));
    }
    else
    {
      z_LF = z_LF0 + z_min;
    }

    if ((time_cycle > t_lift_1) && (time_cycle < t_lift_1 + duration))
    {
    }

    IK_RH(x_RH, y_RH, z_RH, posis);
    IK_LH(x_LH, y_LH, z_LH, posis);
    IK_RF(x_RF, y_RF, z_RF, Roll_RF, Pitch_RF, posis);
    IK_LF(x_LF, y_LF, z_LF, Roll_LF, Pitch_LF, posis);

    // for (int i = 0; i < posis.size(); i++)
    // std::cout << posis[i] << " ";
    // std::cout << std::endl;

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
      msg_angle.data.push_back(joint_angle[i]);
    }

    chatter_pub.publish(msg_angle);

    groupSyncWrite.txPacket();
    groupSyncWrite.clearParam();

    tick++; // For general tick

    clock_gettime(CLOCK_MONOTONIC, &t_end1);

    diff1 = MILLION * (t_end1.tv_sec - t_start.tv_sec) + (t_end1.tv_nsec - t_start.tv_nsec) / 1000; //usec
    t_loop1 = diff1 * 1e-3;                                                                         // msec
    t_sleep = 10000.0 - (float)t_loop1 * 1000;                                                      // Loop time == 8ms

    if (t_sleep > 0 && t_sleep < 50000)
    {
      usleep(t_sleep - 65);
    }

    clock_gettime(CLOCK_MONOTONIC, &t_end2);
    t_elapsed = (double)((t_end2.tv_sec - t0.tv_sec) * 1000000 + (t_end2.tv_nsec - t0.tv_nsec) / 1000) / 1000000;
    diff2 = MILLION * (t_end2.tv_sec - t_start.tv_sec) + (t_end2.tv_nsec - t_start.tv_nsec) / 1000; //usec
    t_loop2 = diff2 * 1e-3;
  }

  for (int i = 1; i < 17; i++)
  {
    packetHandler->write1ByteTxRx(portHandler, i, ADDR_P_GAIN, 55, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, i, ADDR_I_GAIN, 10, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, i, ADDR_D_GAIN, 1, &dxl_error);
  }

  ros::Duration(1).sleep();
  initPose();
  ros::Duration(1).sleep();

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

// void walk(long loops)
// {
//   clock_gettime(CLOCK_MONOTONIC, &t0);
//   while (tick < loops)
//   {
//     clock_gettime(CLOCK_MONOTONIC, &t_start);
//     // ROS_INFO("Step: %ld     t_loop: %2.3f   Elapsed Time: %3.3f   ", tick, t_loop2, t_elapsed);

//     std::vector<uint16_t> posis(16, 512);
//     ros::NodeHandle nh;
//     ros::Publisher chatter_pub = nh.advertise<std_msgs::Float32MultiArray>("joint_angles", 10);
//     std_msgs::Float32MultiArray msg_angle;
//     msg_angle.data.clear();

//     // Motion Generation
//     x_RH = x_RH0 + 50.0;
//     x_LH = x_LH0 + 50.0;
//     y_RH = y_RH0 + 100.0;
//     y_LH = y_LH0 - 100.0;
//     z_RH = z_RH0 - 90.0;
//     z_LH = z_LH0 - 90.0;

//     y_amp = 47.0;
//     z_height = 0.0;
//     step_time = 0.6;
//     tau = 0.8;
//     duration = step_time * tau;
//     t_lift_1 = (step_time - duration) / 2.0;
//     t_lift_2 = t_lift_1 + step_time;

//     y_RF = y_RF0 - (y_amp / 2 * sin(2 * PI * t_elapsed / (2 * step_time)));
//     y_LF = y_LF0 - (y_amp / 2 * sin(2 * PI * t_elapsed / (2 * step_time)));

//     // x_RF = x_RF0 + (y_amp / 2 * sin(2 * PI * t_elapsed / (2 * step_time)));
//     // x_LF = x_LF0 - (y_amp / 2 * sin(2 * PI * t_elapsed / (2 * step_time)));

//     int step_num = ceil(t_elapsed / step_time);
//     float time_cycle = fmod(t_elapsed, 2 * step_time);

//     if ((time_cycle > t_lift_1) && (time_cycle < t_lift_1 + duration))
//     {
//       z_RF = z_RF0 + z_min + z_height / 2 * (1 - cos(2 * PI / (duration) * (time_cycle - t_lift_1)));
//     }
//     else
//     {
//       z_RF = z_RF0 + z_min;
//     }
//     if ((time_cycle > t_lift_2) && (time_cycle < t_lift_2 + duration))
//     {
//       z_LF = z_LF0 + z_min + z_height / 2 * (1 - cos(2 * PI / (duration) * (time_cycle - t_lift_2)));
//     }
//     else
//     {
//       z_LF = z_LF0 + z_min;
//     }

//     if ((time_cycle > t_lift_1) && (time_cycle < t_lift_1 + duration))
//     {
//     }

//     IK_RH(x_RH, y_RH, z_RH, posis);
//     IK_LH(x_LH, y_LH, z_LH, posis);
//     IK_RF(x_RF, y_RF, z_RF, Roll_RF, Pitch_RF, posis);
//     IK_LF(x_LF, y_LF, z_LF, Roll_LF, Pitch_LF, posis);

//     // for (int i = 0; i < posis.size(); i++)
//     // std::cout << posis[i] << " ";
//     // std::cout << std::endl;

//     //write positions to motors
//     for (int i = 0; i < 16; i++)
//     {
//       uint8_t param_goal_position[4];
//       param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD((uint16_t)posis[i]));
//       param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD((uint16_t)posis[i]));
//       param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD((uint16_t)posis[i]));
//       param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD((uint16_t)posis[i]));
//       groupSyncWrite.addParam((uint8_t)(i + 1), param_goal_position);
//     }

//     for (int i = 0; i < 16; i++)
//     {
//       joint_angle[i] = ((double)posis[i] * 0.00511826781 - 512 * 0.00511826781);
//       msg_angle.data.push_back(joint_angle[i]);
//     }

//      chatter_pub.publish(msg_angle);

//     groupSyncWrite.txPacket();
//     groupSyncWrite.clearParam();

//     tick++; // For general tick

//     clock_gettime(CLOCK_MONOTONIC, &t_end1);

//     diff1 = MILLION * (t_end1.tv_sec - t_start.tv_sec) + (t_end1.tv_nsec - t_start.tv_nsec) / 1000; //usec
//     t_loop1 = diff1 * 1e-3;                                                                         // msec
//     t_sleep = 10000.0 - (float)t_loop1 * 1000;                                                      // Loop time == 8ms

//     if (t_sleep > 0 && t_sleep < 50000)
//     {
//       usleep(t_sleep - 65);
//     }

//     clock_gettime(CLOCK_MONOTONIC, &t_end2);
//     t_elapsed = (double)((t_end2.tv_sec - t0.tv_sec) * 1000000 + (t_end2.tv_nsec - t0.tv_nsec) / 1000) / 1000000;
//     diff2 = MILLION * (t_end2.tv_sec - t_start.tv_sec) + (t_end2.tv_nsec - t_start.tv_nsec) / 1000; //usec
//     t_loop2 = diff2 * 1e-3;
//   }
// }

void initPose()
{
  std::vector<uint16_t> posis(16, 512);
  //initial pose
  x_RH = x_RH0 + 30.0;
  y_RH = y_RH0 + 90.0;
  z_RH = z_RH0 - 90.0;

  x_LH = x_LH0 + 30.0;
  y_LH = y_LH0 - 90.0;
  z_LH = z_LH0 + -90.0;

  x_RF = x_RF0 - 0;
  y_RF = y_RF0;
  z_RF = z_RF0 + z_min;

  x_LF = x_LF0 - 0;
  y_LF = y_LF0;
  z_LF = z_LF0 + z_min;

  IK_RH(x_RH, y_RH, z_RH, posis);
  IK_LH(x_LH, y_LH, z_LH, posis);
  IK_RF(x_RF, y_RF, z_RF, Roll_RF, Pitch_RF, posis);
  IK_LF(x_LF, y_LF, z_LF, Roll_LF, Pitch_LF, posis);

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

void positionCallback(geometry_msgs::Point msg)
{
  int x = msg.x;
  int y = msg.y;
  int size = msg.z;

  //just print the info that it recieved for now
  ROS_INFO("X: %d Y: %d Size: %d", x, y, size);
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
