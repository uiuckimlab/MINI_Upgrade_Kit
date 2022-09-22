#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <vector>
#include "dynamixel_sdk/dynamixel_sdk.h"

//right leg 
void IK_RF(float x, float y, float z, float th_r, float th_p, std::vector<uint16_t> &positions); 

//left leg
void IK_LF(float x, float y, float z, float th_r, float th_p, std::vector<uint16_t> &positions);

//right arm 
void IK_RH(float x, float y, float z, std::vector<uint16_t> &positions); 

//left arm 
void IK_LH(float x, float y, float z, std::vector<uint16_t> &positions);

#endif