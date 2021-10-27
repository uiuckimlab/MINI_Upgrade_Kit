#include "../include/kinematics.h"

#include <math.h>
#include<stdlib.h> 

# define PI 3.1415

float L_sh = 39.0; // Origin to arm roll joint
float L_a1 = 18.0; // Shoulder bracket horizontal distance
float L_a2 = 12.0; // Shoulder bracket vertical distance
float L_a3 = 45.0; // Upper arm length
float L_a4 = 72.0; // Lower arm length


float L_by = 24.0; // Origin to pelvis vertical length
float L_bz = 72.0; // Pelvis horizontal length
float L_bx = 15.0; // Shoulder joint axis to Leg center (On Sagittal Plane)
float L_l1 = 6.0;  // Pelvis Roll axis to pitch axis
float L_l2 = 45.0; // Thigh Length
float L_l3 = 42.0; // Shank Length
float L_l4 = 31.0; // Ankle Length
float L_f = 9.0;   // Foot horizontal length

void IK_RF(float x, float y, float z, float th_r, float th_p, std::vector<uint16_t> &positions){
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
    - L_bz - z_RF - L_f*sin(th_r) - L_l4*cos(th_p)*cos(th_r),
        L_by + y_RF + L_f*cos(th_r) - L_l4*cos(th_p)*sin(th_r),
                                        x_RF - L_bx + L_l4*sin(th_p)};


    th7 = atan(pos_RF[1]/pos_RF[0]);
    float R1_RF = sqrt((pos_RF[0]-L_l1*cos(th7))*(pos_RF[0]-L_l1*cos(th7)) + (pos_RF[1]-L_l1*sin(th7))*(pos_RF[1]-L_l1*sin(th7)) + pos_RF[2]*pos_RF[2]);
    float alpha_RF = acos((L_l2*L_l2 + L_l3*L_l3 - R1_RF*R1_RF)/(2*L_l2*L_l3));
    th11 = PI - alpha_RF;
    float R2_RF = sqrt((pos_RF[0]-L_l1*cos(th7))*(pos_RF[0]-L_l1*cos(th7)) + (pos_RF[1]-L_l1*sin(th7))*(pos_RF[1]-L_l1*sin(th7)));

    th9 = -((atan(pos_RF[2]/R2_RF) + acos((L_l2*L_l2 + R1_RF*R1_RF - L_l3*L_l3)/(2*L_l2*R1_RF))));


    th13 = -asin(cos(th9 + th11)*cos(th_r)*cos(th7)*sin(th_p) - sin(th9 + th11)*cos(th_p) + cos(th9 + th11)*sin(th_p)*sin(th_r)*sin(th7));
    th15 = -asin(sin(th_r - th7)*cos(th_p));

    // printf("%f   %f   %f   %f   %f\n", th7, th9, th11, th13, th15);


    DXL_POS_7 = (512 + (int)(th7* 195.3786));
    DXL_POS_9 = (512 + (int)(th9* 195.3786));
    DXL_POS_11 = (512 + (int)(th11*195.3786));
    DXL_POS_13 = (512 + (int)(th13*195.3786));
    DXL_POS_15 =(512 + (int)(th15*195.3786));

    positions[6] = (uint16_t)DXL_POS_7;
    positions[8] = (uint16_t)DXL_POS_9;
    positions[10] = (uint16_t)DXL_POS_11;
    positions[12] = (uint16_t)DXL_POS_13; 
    positions[14] = (uint16_t)DXL_POS_15; 
}

void IK_LF(float x, float y, float z, float th_r, float th_p, std::vector<uint16_t> &positions){
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
    L_f*sin(th_r) - z_LF - L_bz - L_l4*cos(th_p)*cos(th_r),
    y_LF - L_by - L_f*cos(th_r) - L_l4*cos(th_p)*sin(th_r),
                                    x_LF - L_bx + L_l4*sin(th_p)};


    th8 = atan(pos_LF[1]/pos_LF[0]);
    float R1_LF = sqrt((pos_LF[0]-L_l1*cos(th8))*(pos_LF[0]-L_l1*cos(th8)) + (pos_LF[1]-L_l1*sin(th8))*(pos_LF[1]-L_l1*sin(th8)) + pos_LF[2]*pos_LF[2]);
    float alpha_LF = acos((L_l2*L_l2 + L_l3*L_l3 - R1_LF*R1_LF)/(2*L_l2*L_l3));

    th12 = -PI + alpha_LF;
    float R2_LF = sqrt((pos_LF[0]-L_l1*cos(th8))*(pos_LF[0]-L_l1*cos(th8)) + (pos_LF[1]-L_l1*sin(th8))*(pos_LF[1]-L_l1*sin(th8)));

    th10 = (atan(pos_LF[2]/R2_LF) + acos((L_l2*L_l2 + R1_LF*R1_LF - L_l3*L_l3)/(2*L_l2*R1_LF)));

    th14 = -acos(cos(th10 + th12)*cos(th_p) - sin(th10 + th12)*cos(th_r)*cos(th8)*sin(th_p) - sin(th10 + th12)*sin(th_p)*sin(th_r)*sin(th8));
    th16 = -asin(sin(th_r - th8)*cos(th_p));

    // printf("%f   %f   %f   %f   %f\n", th8, th10, th12, th14, th16);


    DXL_POS_8 =  (512 + (int)(th8* 195.3786));
    DXL_POS_10 = (512 + (int)(th10* 195.3786));
    DXL_POS_12 = (512 + (int)(th12*195.3786));
    DXL_POS_14 = (512 + (int)(th14*195.3786));
    DXL_POS_16 = (512 + (int)(th16*195.3786));

    positions[7] = (uint16_t)DXL_POS_8;
    positions[9] = (uint16_t)DXL_POS_10;
    positions[11] = (uint16_t)DXL_POS_12;
    positions[13] = (uint16_t)DXL_POS_14; 
    positions[15] = (uint16_t)DXL_POS_16; 
}

void IK_RH(float x, float y, float z, std::vector<uint16_t> &positions){
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

    th1 = - atan(x_RH0/(z_RH0+0.00001));

    float R1_RH = sqrt((x_RH0 - L_a2*sin(th1))*(x_RH0 - L_a2*sin(th1)) + y_RH0*y_RH0 + (z_RH0 + L_a2*cos(th1))*(z_RH0 + L_a2*cos(th1)));

    if (R1_RH > 117){
        R1_RH = 117;
    } else{
        R1_RH = R1_RH;
    }

    float alpha_RH = acos((L_a3*L_a3 + L_a4*L_a4 - R1_RH*R1_RH)/(2*L_a3*L_a4));

    // Elbow Joint angle
    th5 = -PI + alpha_RH;
    float R2_RH = sqrt((x_RH0 - L_a2*sin(th1))*(x_RH0 - L_a2*sin(th1)) + (z_RH0 + L_a2*cos(th1))*(z_RH0 + L_a2*cos(th1)));

    // Shoulder Joint angle
    if (z_RH > 0){
    th3 = PI/2 + (atan(y_RH0/R2_RH) + acos((L_a3*L_a3 + R1_RH*R1_RH - L_a4*L_a4)/(2*L_a3*R1_RH)));
    }else{
    th3 = -PI/2 + (-atan(y_RH0/R2_RH) + acos((L_a3*L_a3 + R1_RH*R1_RH - L_a4*L_a4)/(2*L_a3*R1_RH)));
    }


    DXL_POS_1 = (512 + (int)(th1* 195.3786));
    DXL_POS_3 = (512 + (int)(th3* 195.3786));
    DXL_POS_5 = (512 + (int)(th5*195.3786));

    positions[0] = (uint16_t)DXL_POS_1; 
    positions[2] = (uint16_t)DXL_POS_3;
    positions[4] = (uint16_t)DXL_POS_5; 
}

void IK_LH(float x, float y, float z, std::vector<uint16_t> &positions){
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

    th2 = atan(x_LH0/(z_LH0 + 0.00001));

    // Range of XL320 motor -150 to 150 deg

    float R1_LH = sqrt((x_LH0 - L_a2*sin(th2))*(x_LH0 - L_a2*sin(th2)) + y_LH0*y_LH0 + (z_LH0+L_a2*cos(th2))*(z_LH0+L_a2*cos(th2)));

    if (R1_LH > 117){
        R1_LH = 117;
    }else{
        R1_LH = R1_LH;
    }

    float alpha_LH = acos((L_a3*L_a3 + L_a4*L_a4 - R1_LH*R1_LH)/(2*L_a3*L_a4));

    // Elbow Joint angle 
    th6 = PI - alpha_LH;

    float R2_LH = sqrt((x_LH0 - L_a2*sin(th2))*(x_LH0 - L_a2*sin(th2)) + (z_LH0 + L_a2*cos(th2))*(z_LH0 + L_a2*cos(th2)));

    // Shoulder Joint angle
    if (z_LH0 > 0){
        th4 = -((atan(R2_LH/y_LH0) + acos((L_a3*L_a3 + R1_LH*R1_LH - L_a4*L_a4)/(2*L_a3*R1_LH))));
    }else{
        th4 = -(-PI/2 + (atan(y_LH0/R2_LH) + acos((L_a3*L_a3 + R1_LH*R1_LH - L_a4*L_a4)/(2*L_a3*R1_LH))));
    }

    DXL_POS_2 = (512 + (int)(th2* 195.3786));
    DXL_POS_4 = (512 + (int)(th4* 195.3786));
    DXL_POS_6 = (512 + (int)(th6*195.3786));
    

    positions[1] = (uint16_t)DXL_POS_2;
    positions[3] = (uint16_t)DXL_POS_4;
    positions[5] = (uint16_t)DXL_POS_6;  
}
