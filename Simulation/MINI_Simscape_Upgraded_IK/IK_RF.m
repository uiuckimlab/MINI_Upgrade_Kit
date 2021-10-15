% ECE 598 JK Fall 2020 University of Illinois at Urbana-Champaign
% ROBOTIS MINI Right Foot Inverse Kinematics
% 09/07/2020 by Kevin Gim (kggim2@illinois.edu)

function [th7,th9, th11, th13, th15] = IK_RF(x_RF_in, y_RF_in, z_RF_in, roll_RF, pitch_RF)

% Right foot
% Controllable position x, y, z, roll, pitch

L_by = 24; % Origin to pelvis vertical length
L_bz = 72; % Pelvis horizontal length
L_bx = 15; % Shoulder joint axis to Leg center (On Sagittal Plane)
L_l1 = 6;  % Pelvis Roll axis to pitch axis
L_l2 = 45; % Thigh Length
L_l3 = 42; % Shank Length
L_l4 = 31;  % Ankle Length
L_f = 9; % Foot horizontal length

x_RF = x_RF_in;
y_RF = y_RF_in;
z_RF = z_RF_in;



R0RF = RX(roll_RF) * RY(pitch_RF);
T0RF = R0RF + TXYZ(x_RF, y_RF, z_RF) - eye(4);
T015 = T0RF * RY(-(pi)/2) * RZ((pi)) * TXYZ(-L_l4,-L_f,0);
TRF0_15 = (TXYZ(L_bx,-L_by,-L_bz)*RY((pi)/2))\T015;

pos_RF = TRF0_15(1:3,4);
th7 = atan(pos_RF(2)/pos_RF(1));
R1_RF = sqrt((pos_RF(1)-L_l1*cos(th7))^2 + (pos_RF(2)-L_l1*sin(th7))^2 + pos_RF(3)^2);
alpha_RF = acos((L_l2^2 + L_l3^2 -R1_RF^2)/(2*L_l2*L_l3));
th11 = pi-alpha_RF;
R2_RF = sqrt((pos_RF(1)-L_l1*cos(th7))^2 + (pos_RF(2)-L_l1*sin(th7))^2);

th9 = -((atan(pos_RF(3)/R2_RF) + acos((L_l2^2 + R1_RF^2 - L_l3^2)/(2*L_l2*R1_RF))));

T_RF0_11 = [ cos(th9 + th11)*cos(th7), -sin(th9 + th11)*cos(th7), -sin(th7), cos(th7)*(L_l1 + L_l2*cos(th9));
    cos(th9 + th11)*sin(th7), -sin(th9 + th11)*sin(th7),  cos(th7), sin(th7)*(L_l1 + L_l2*cos(th9));
    -sin(th9 + th11),          -cos(th9 + th11),         0,                  -L_l2*sin(th9);
    0,                         0,         0,                               1];

T_11_15 = (T_RF0_11)\TRF0_15;
th13 = asin(T_11_15(1,3));
th15 = -asin(-T_11_15(3,1));
