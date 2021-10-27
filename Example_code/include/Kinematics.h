#ifndef KINEMATICS_H
#define KINEMATICS_H

int* IK_RH(int groupwrite_num, float x, float y, float z);
int* IK_LH(int groupwrite_num, float x, float y, float z);
int* IK_RF(int groupwrite_num, float x, float y, float z, float th_r, float th_p);
int* IK_LF(int groupwrite_num, float x, float y, float z, float th_r, float th_p);
#endif
