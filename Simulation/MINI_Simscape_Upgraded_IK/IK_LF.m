% ECE 598 JK Fall 2020 University of Illinois at Urbana-Champaign
% ROBOTIS MINI Left Foot Inverse Kinematics
% 09/07/2020 by Kevin Gim (kggim2@illinois.edu)

function [th8, th10, th12, th14, th16] = IK_LF(x_LF_in, y_LF_in, z_LF_in, roll_LF, pitch_LF)

%Left foot
% Controllable position x, y, z, roll, pitch

L_by = 24; % Origin to pelvis vertical length
L_bz = 72; % Pelvis horizontal length
L_bx = 15; % Shoulder joint axis to Leg center (On Sagittal Plane)
L_l1 = 6;  % Pelvis Roll axis to pitch axis
L_l2 = 45; % Thigh Length
L_l3 = 42; % Shank Length
L_l4 = 31;  % Ankle Length
L_f = 9; % Foot horizontal length

x_LF = x_LF_in;
y_LF = y_LF_in;
z_LF = z_LF_in;


R0LF = RX(roll_LF) * RY(pitch_LF);
T0LF = R0LF + TXYZ(x_LF, y_LF, z_LF) - eye(4);
T016 = T0LF * RY(-(pi)/2) *RZ((pi)) * TXYZ(-L_l4,L_f,0);
TLF0_16 = (TXYZ(L_bx,L_by,-L_bz)*RY((pi)/2))\T016;
pos_LF = TLF0_16(1:3,4);

th8 = atan(pos_LF(2)/pos_LF(1));
R1_LF = sqrt((pos_LF(1)-L_l1*cos(th8))^2 + (pos_LF(2)-L_l1*sin(th8))^2 + pos_LF(3)^2);
alpha_LF = acos((L_l2^2 + L_l3^2 -R1_LF^2)/(2*L_l2*L_l3));

th12 = -pi+alpha_LF;
R2_LF = sqrt((pos_LF(1)-L_l1*cos(th8))^2 + (pos_LF(2)-L_l1*sin(th8))^2);

th10 = (atan(pos_LF(3)/R2_LF) + acos((L_l2^2 + R1_LF^2 - L_l3^2)/(2*L_l2*R1_LF)));


T_LF0_12 = [ cos(th10 + th12)*cos(th8), -sin(th10 + th12)*cos(th8),  sin(th8), cos(th8)*(L_l1 + L_l2*cos(th10));
    cos(th10 + th12)*sin(th8), -sin(th10 + th12)*sin(th8), -cos(th8), sin(th8)*(L_l1 + L_l2*cos(th10));
    sin(th10 + th12),           cos(th10 + th12),         0,                   L_l2*sin(th10);
    0,                          0,         0,                                1];

T_12_16 = (T_LF0_12)\TLF0_16;
th14 = -asin(T_12_16(1,3));
th16 = asin(-T_12_16(3,1));



