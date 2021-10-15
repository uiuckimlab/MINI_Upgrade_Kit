%SetupMINI

m_body = 250.48;
m_arm1 = 4.26;
m_arm2 = 23;
m_arm3 = 26.16;
m_leg1 = 19.9;
m_leg2 = 23;
m_leg3 = 8.99;
m_leg4 = 19.57;
m_leg5 = 36.53;

CoM_BODY = [-1.77; 0; -21.33];
CoM_RH_1 = [4.57, 0, -8.85];
CoM_RH_2 = [12.05, 0.97, -0.05];
CoM_RH_3 = [20, 1.29, 0.18];
CoM_LH_1 = [4.57, 0, -8.85];
CoM_LH_2 = [12.05, -0.97, -0.05];
CoM_LH_3 = [20, -1.29, 0.18];
CoM_RF_1 = [-1.22, -0.06, -1.47];
CoM_RF_2 = [32.95, -0.97, -0.05];
CoM_RF_3 = [17.90, -6.88, 0];
CoM_RF_4 = [-6.23, -1.76, 0.06];
CoM_RF_5 = [12.25, 8.01, 21.06];
CoM_LF_1 = [-1.22, 0.06, -1.47];
CoM_LF_2 = [32.95, 0.97, -0.05];
CoM_LF_3 = [17.90, 6.88, 0];
CoM_LF_4 = [-6.23, 1.76, 0.06];
CoM_LF_5 = [12.25, -8.01, 21.06];

L_sh = 39; % Origin to arm roll joint
L_a1 = 18; % Shoulder bracket horizontal distance
L_a2 = 12; % Shoulder bracket vertical distance
L_a3 = 45; % Upper arm length
L_a4 = 72; % Lower arm length

L_by = 24; % Origin to pelvis vertical length
L_bz = 72; % Pelvis horizontal length
L_bx = 15; % Shoulder joint axis to Leg center (On Sagittal Plane)
L_l1 = 6;  % Pelvis Roll axis to pitch axis
L_l2 = 45; % Thigh Length
L_l3 = 42; % Shank Length
L_l4 = 31;  % Ankle Length


MINI     = struct('name','BODY'    , 'm', m_body, 'P_CoM',CoM_BODY);
MINI(2)  = struct('name','RH_1'    , 'm', m_arm1, 'P_CoM',CoM_RH_1);
MINI(3)  = struct('name','RH_2'    , 'm', m_arm2, 'P_CoM',CoM_RH_2);
MINI(4)  = struct('name','RH_3'    , 'm', m_arm3, 'P_CoM',CoM_RH_3);

MINI(5)  = struct('name','LH_1'    , 'm', m_arm2, 'P_CoM',CoM_LH_1);
MINI(6)  = struct('name','LH_2'    , 'm', m_arm2, 'P_CoM',CoM_LH_2);
MINI(7)  = struct('name','LH_3'    , 'm', m_arm3, 'P_CoM',CoM_LH_3);

MINI(8)   = struct('name','RF_1'    , 'm', m_leg1, 'P_CoM',CoM_RF_1);
MINI(9)   = struct('name','RF_2'    , 'm', m_leg2, 'P_CoM',CoM_RF_2);
MINI(10)  = struct('name','RF_3'    , 'm', m_leg3, 'P_CoM',CoM_RF_3);
MINI(11)  = struct('name','RF_4'    , 'm', m_leg4, 'P_CoM',CoM_RF_4);
MINI(12)  = struct('name','RF_5'    , 'm', m_leg5, 'P_CoM',CoM_RF_5);

MINI(13)  = struct('name','LF_1'    , 'm', m_leg1, 'P_CoM',CoM_LF_1);
MINI(14)  = struct('name','LF_2'    , 'm', m_leg2, 'P_CoM',CoM_LF_2);
MINI(15)  = struct('name','LF_3'    , 'm', m_leg3, 'P_CoM',CoM_LF_3);
MINI(16)  = struct('name','LF_4'    , 'm', m_leg4, 'P_CoM',CoM_LF_4);
MINI(17)  = struct('name','LF_5'    , 'm', m_leg5, 'P_CoM',CoM_LF_5);
