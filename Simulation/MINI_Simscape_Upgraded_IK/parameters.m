% Contact parameters

Contact_Stiff = 1e5;
Contact_Damping = 2e4;
Contact_Trans = 1e-2;

Coff_static = 0.9;
Coff_dyn = 0.9;
V_critical = 1e-5;

% Ball size
Contact_r = 0.5;
Contact_color = [1.0 0 0];
CoM_r = 5;
CoM_color = [1.0 0 0];
ZMP_r = 5;
ZMP_color = [0 0 1.0];
ZMP2_r = 5;
ZMP2_color = [0.5 0 0.5];
CoP_r = 5;
CoP_color = [1.0 1.0 0];

% Butterworth Low-pass Filter Parameters for Foot force sensors 
Butter_order = 4;
Butter_freq = 2*pi*50;

% Butterworth Low-pass Filter Parameters for an IMU
Butter_order_IMU = 2;
Butter_freq_IMU = 2*pi*200;
foot_opacity = 0.1;

joint_filter = 0.001;

% Robot height
pz = -(186-66.82);

x_RF0 = 15;
y_RF0 = 0;
z_RF0 = -186;

x_LF0 = 15;
y_LF0 = 0;
z_LF0 = -186;

% [th7_ofst, th9_ofst, th11_ofst, th13_ofst, th15_ofst] = IK_RF(x_RF0, y_RF0, z_RF0, 0, 0);
% [th8_ofst, th10_ofst, th12_ofst, th14_ofst, th16_ofst] = IK_LF(x_LF0, y_LF0, z_LF0, 0, 0);
th1_ofst = pi/2;
th3_ofst = pi/2;
th5_ofst = 0;

th2_ofst = -pi/2;
th4_ofst = pi/2;
th6_ofst = 0;

th7_ofst = pi/2;
th9_ofst = 0;
th11_ofst = 0;
th13_ofst = 0;
th15_ofst = 0;

th8_ofst = pi/2;
th10_ofst = -pi;
th12_ofst = pi;
th14_ofst = 0;
th16_ofst = 0;
