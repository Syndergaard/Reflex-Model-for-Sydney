% BICEP AND ELBOW FLEXION-EXTENSION MODEL

% Muscle Excitation-Contraction Dynamics
T_1 = 0.03;
T_2 = 0.04;
C = 525.1;

ss_A = [    0,              1;
            -1/(T_1*T_2),   -(T_1+T_2)/(T_1*T_2)];
ss_B = [0;
        C/(T_1*T_2)];
ss_C = [1, 0];
ss_D = 0;

sys_OL_u2f = ss(ss_A,ss_B,ss_C,ss_D);
    sys_OL_u2f.inputDelay = 0.015;
sys_GTO = ss(1.27/C);
    sys_GTO.outputDelay = 0.015;
sys_CL_u2f = feedback(sys_OL_u2f,sys_GTO);

step(sys_CL_u2f,0:0.001:1)

% Musculoskeletal Geometry
M = 0.045754;
sys_f2tau = ss(M);

% Mechanical Impedance
I = 0.075809;
I_inv = inv(I);
D = 0.6069;
K = 8.67;

ss_A = [0,          1;
         -I_inv*K,   -I_inv*D];
ss_B = [0;
        I_inv];
ss_C = [1, 0];
ss_D = 0;

sys_tau2q = ss(ss_A,ss_B,ss_C,ss_D);

% Open-Loop model (with GTOs)
sys_OL_u2q = sys_tau2q * sys_f2tau * sys_CL_u2f;

step(sys_OL_u2q,0:0.001:2)