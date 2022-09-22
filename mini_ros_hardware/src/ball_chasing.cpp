/*******************************************************************************
 * This prgoram is written for upgraded ROBOTIS MINI with the Orange Pi and a USB camera. 
 * It will track a blue ball on the floor and "attempt" to walk towards it and kick it. 
 * To run this program: 
 *
 * In a terminal:  
 * $ roslaunch dynamixel_sdk_examples mini.launch 
 *
 * In a second terminal:
 * $ cd /home/kevingim/
 * $ python perception.py   
 * 
 * attempting to run this with rosrun will cause the program to segfault, as it doesn't parameters from the .yaml file. 
 *  
 * If you get an error when running perception.py, make sure the camera device is properly connected with $v4l2-ctl --list-devices
 * Make sure that the device number for the usb camera matches the number in line 14 of percception.py 
 *       if camera is /dev/video1, then perception.py should have "cap = cv2.VideoCapture(1)
*******************************************************************************/

#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <time.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

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

// Protocol version
#define PROTOCOL_VERSION 2.0 // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define DXL1_ID 1                  // DXL1 ID
#define DXL2_ID 2                  // DXL2 ID
#define BAUDRATE 1000000           // Default Baudrate of DYNAMIXEL XL 320
#define DEVICE_NAME "/dev/ttyUSB0" // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

#define PI 3.1412
#define MILLION 1000000L

PortHandler *portHandler;
PacketHandler *packetHandler;

bool initJoints()
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  for (uint8_t i = 1; i < 17; i++)
  {
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      ROS_ERROR("Failed to enable torque for Dynamixel ID %d", i);
      return false;
    }
  }
  return true;
}

uint16_t degToPosConv(double degree)
{
  //position values range from 0-1023, one unit ≈ 0.2932 degrees.
  return (uint16_t)(512 + (degree / 0.293));
}

std::vector<uint16_t> allDegconv(const std::vector<double> &deg)
{
  std::vector<uint16_t> pos;
  for (int i = 0; i < deg.size(); i++)
  {
    pos.push_back(degToPosConv(deg[i]));
    if (pos[i] > 1023 || pos[i] < 0)
    { //if converted position values exceed motor range set it to -1 (invalid)
      pos[i] = -1;
    }
  }
  return pos;
}

void setPosition(uint16_t position, int id)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  if (position < 0)
    return; //invalid position value

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.

  // Write Goal Position (length : 4 bytes)
  // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
  dxl_comm_result = packetHandler->write2ByteTxRx(
      portHandler, (uint8_t)id, ADDR_GOAL_POSITION, position, &dxl_error);

  if (dxl_comm_result == COMM_SUCCESS)
  {
    // ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", id, position);
  }
  else
  {
    // ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
  }
}

//write to all 16 positions
void setJoints(std::vector<uint16_t> jointPos, double moveTime)
{
  //ros::Duration(moveTime).sleep();

  for (int i = 0; i < jointPos.size(); i++)
  {
    setPosition(jointPos[i], i + 1);
  }
}

void poseFrame(std::vector<double> command)
{
  std::vector<double> degrees(command.begin(), command.begin() + 16);
  double seconds = command[16] / 500.0;
  std::vector<uint16_t> positions = allDegconv(degrees);
  setJoints(positions, seconds);
}

void advance()
{
  poseFrame({0, 0, -73.24, 73.24, 0, 0, 0, 0, -42.23, 30.51, 29.3, -29.3, 10.25, -21.97, 2.93, 11.72, 5});
  poseFrame({0, 0, -73.24, 73.24, 0, 0, -2.93, 2.93, -36.37, 51.02, 29.3, -58.59, 16.11, -30.76, 2.93, 8.79, 5});
  poseFrame({0, 0, -73.24, 73.24, 0, 0, 0, 0, -30.51, 42.23, 29.3, -29.3, 21.97, -10.25, 0, 0, 5});
  poseFrame({0, 0, -73.54, 72.95, -0.29, -0.29, -0.29, -0.29, -30.8, 41.93, 29, -29.59, 21.68, -10.55, -12.01, -3.22, 5});
  poseFrame({0, 0, -73.54, 72.95, -0.29, -0.29, -3.22, 2.64, -51.31, 36.07, 58.3, -29.59, 30.47, -16.41, -9.08, -3.22, 5});
  poseFrame({0, 0, -73.24, 73.24, 0, 0, 0, 0, -42.23, 30.51, 29.3, -29.3, 10.25, -21.97, 0, 0, 5});

  // poseFrame({50, -50, -73.24, 73.24, 0, 0, 0, 0, -32.23, 20.51, 29.3, -29.3, 10.25, -21.97, 2.93, 11.72, 70});
  // poseFrame({50, -50, -73.24, 73.24, 0, 0, -2.93, 2.93, -26.37, 41.02, 29.3, -58.59, 16.11, -30.76, 2.93, 8.79, 70});
  //poseFrame({0, 0, -73.24, 73.24,0,0,0,0,-36.37,36.37,29.3,-29.3,13.18,-13.18,0,0, 5});
}

void lKick()
{
  poseFrame({0, 0, -73.24, 73.24, 0, 0, 0, 0, -36.66, 36.07, 29.3, -29.3, 15.82, -16.41, 8.79, 14.65, 10});
  poseFrame({0, 0, -73.24, 73.24, 0, 0, 0, 8.79, -36.66, 41.64, 29.3, -55.51, 15.82, -16.41, 14.65, 14.65, 10});
  poseFrame({0, 0, -44.24, 51.27, -20.8, 9.96, -6.15, 5.86, -32.52, 45.88, 29.3, 8.77, 9.96, 0, 14.65, 14.65, 10});
  poseFrame({0, 0, -73.24, 73.24, 0, 0, -6.15, 5.86, -26.66, 36.62, 29.3, -39.84, 15.82, -5.86, 8.79, 20.51, 10});
  poseFrame({0, 0, -73.24, 73.24, 0, 0, 0, 0, -36.37, 36.37, 29.3, -29.3, 13.18, -13.18, 0, 0, 10});
}

void positionCallback(geometry_msgs::Point msg)
{
  int x = msg.x;
  int y = msg.y;
  int size = msg.z;

  ROS_INFO("X: %d Y: %d Size: %d", x, y, size);

  if (size > 37)
    return;

  //turn left
  if (x > 125)
  {
    ROS_INFO("left");
    poseFrame({0, 0, -72.95, 73.54, 0.29, 0.29, -5.27, 5.27, -45.16, 27.58, 29.59, -29, 4.39, -21.97, -5.57, 5.86, 70});
    poseFrame({0, 0, -72.95, 73.54, 0.29, 0.29, -5.27, 5.27, -45.16, 27.58, 29.59, -29, -4.39, -30.76, -11.43, 11.72, 70});
    poseFrame({0, 0, -72.95, 73.54, 0.29, 0.29, 0.59, -0.59, -36.37, 36.37, 29.59, -29, 13.18, -13.18, 0.29, 0, 140});

    poseFrame({0, 0, -72.95, 73.54, 0.29, 0.29, -5.27, 5.27, -45.16, 27.58, 29.59, -29, 4.39, -21.97, -5.57, 5.86, 70});
    poseFrame({0, 0, -72.95, 73.54, 0.29, 0.29, -5.27, 5.27, -45.16, 27.58, 29.59, -29, -4.39, -30.76, -11.43, 11.72, 70});
    poseFrame({0, 0, -72.95, 73.54, 0.29, 0.29, 0.59, -0.59, -36.37, 36.37, 29.59, -29, 13.18, -13.18, 0.29, 0, 140});
    ros::Duration(0.5).sleep();
  }

  //turn right
  else if (x < 35)
  {
    ROS_INFO("right");
    poseFrame({0, 0, -73.54, 72.95, 0, 0, -5.27, 5.27, -27.58, 45.16, 29, -29.59, 21.97, -4.39, -5.86, 5.57, 70});
    poseFrame({0, 0, -73.54, 72.95, -0.29, -0.29, -5.27, 5.27, -27.58, 45.16, 29, -29.59, 30.76, 4.39, -11.72, 11.43, 70});
    poseFrame({0, 0, -73.54, 72.95, -0.29, -0.29, 0.59, -0.59, -36.37, 36.37, 29, -29.59, 13.18, -13.18, 0, -0.29, 140});

    poseFrame({0, 0, -73.54, 72.95, 0, 0, -5.27, 5.27, -27.58, 45.16, 29, -29.59, 21.97, -4.39, -5.86, 5.57, 70});
    poseFrame({0, 0, -73.54, 72.95, -0.29, -0.29, -5.27, 5.27, -27.58, 45.16, 29, -29.59, 30.76, 4.39, -11.72, 11.43, 70});
    poseFrame({0, 0, -73.54, 72.95, -0.29, -0.29, 0.59, -0.59, -36.37, 36.37, 29, -29.59, 13.18, -13.18, 0, -0.29, 140});

    ros::Duration(0.5).sleep();
  }

  //its centered, so move forward depending on size (aka distance)
  else
  {
    if (size >= 35 && size < 37)
    { //if close enough, just keep moving a hardcoded amt because ball will be out of camera's fov
      ROS_INFO("pushing to the end ");
      for (int i = 0; i < 14; i++)
        advance();

      lKick();
      ros::Duration(0.5).sleep();

      return;
    }
    else if (size < 35)
    {
      ROS_INFO("advancing");
      advance();
    }
  }
}

int main(int argc, char **argv)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  portHandler = PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

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

  ros::init(argc, argv, "ball_chasing_node");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/ball_pos", 1, positionCallback);

  //this turns on torque
  if (!initJoints())
    return -1; //motor initial ization failed

  //this section of code has robot move based on a list of instructions in the missionPlan.yaml file under dynamixel_sdk_examples/config

  std::vector<double> command;
  nh.getParam("/initial_pose", command);
  poseFrame(command);

  // std::vector<std::string> movesList;
  // nh.getParam("/demoMoves", movesList);

  // for(std::vector<std::string>::iterator it = std::begin(movesList); it != std::end(movesList); ++it){
  //   std::string move = *it;
  //   ROS_INFO("move: %s", move.c_str());
  //   std::vector<double> command;
  //   if(move == "initial_pose"){
  //     nh.getParam("/initial_pose", command);
  //     poseFrame(command);
  //   }

  //   else if(move == "advance"){
  //     nh.getParam("/advance_1", command);
  //     poseFrame(command);
  //     nh.getParam("/advance_2", command);
  //     poseFrame(command);
  //     nh.getParam("/advance_3", command);
  //     poseFrame(command);
  //     nh.getParam("/advance_4", command);
  //     poseFrame(command);
  //     nh.getParam("/advance_5", command);
  //     poseFrame(command);
  //     nh.getParam("/advance_6", command);
  //     poseFrame(command);
  //     /*
  //     nh.getParam("/advance_7", command);
  //     poseFrame(command);
  //     nh.getParam("/advance_8", command);
  //     poseFrame(command);
  //     nh.getParam("/advance_9", command);
  //     poseFrame(command);
  //     */
  //   }

  //   else if(move == "right_turn"){
  //     nh.getParam("right_turn_1", command);
  //     poseFrame(command);
  //     nh.getParam("right_turn_2", command);
  //     poseFrame(command);
  //     nh.getParam("right_turn_3", command);
  //     poseFrame(command);
  //   }

  //   else if(move == "left_turn"){
  //     nh.getParam("left_turn_1", command);
  //     poseFrame(command);
  //     nh.getParam("left_turn_2", command);
  //     poseFrame(command);
  //     nh.getParam("left_turn_3", command);
  //     poseFrame(command);
  //   }

  //   else if(move == "right_step"){
  //     nh.getParam("right_step_1", command);
  //     poseFrame(command);
  //     nh.getParam("right_step_2", command);
  //     poseFrame(command);
  //   }

  //   else if(move == "left_step"){
  //     nh.getParam("left_step_1", command);
  //     poseFrame(command);
  //     nh.getParam("left_step_2", command);
  //     poseFrame(command);
  //   }

  // }

  /*
  std::vector<double> deg = {0,0,-73.24,73.24,0,0,0,0,-26.37,26.37,29.3,-29.3,13.18,-13.18,0,0};
  std::vector<uint32_t> pos = allDegconv(deg);  

  setJoints(pos, 0.3); 
 */
  ros::spin();

  portHandler->closePort();
  return 0;
}