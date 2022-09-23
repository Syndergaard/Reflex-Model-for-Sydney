function sys_OpenLoop = Open_Loop_Model(posture, delay_multiplier)%,Proximal_Delay,Distal_Delay)
    addpath('Parameter Matrices')
    
    % Load Model Parameters
    M = readmatrix(['M', num2str(posture), '.csv']);
    I = readmatrix(['I', num2str(posture), '.csv']);
    D = readmatrix('D.csv');
    K = readmatrix('K.csv');
    C = readmatrix('Max Muscle Force.csv');
    EffDelays = readmatrix('DelaysEfferent.csv') * delay_multiplier;
    muscle_names = {'ECR';'ECU';'FCR';'FCU'};
    joint_DOF_names = {'WFE';'WRUD'};
    
    % u2f
    t_1 = 0.03; % 30ms
    t_2 = 0.04; % 40ms
    A_u2f = [zeros(4) , eye(4) ; -eye(4)/(t_1*t_2) , -eye(4)*(t_1+t_2)/(t_1*t_2)];
    B_u2f = [zeros(4) ; C/(t_1*t_2)];
    C_u2f = [eye(4) , zeros(4)];
    D_u2f = zeros(4);
    sys_u2f = ss(A_u2f,B_u2f,C_u2f,D_u2f);
        sys_u2f.InputName = muscle_names;
        sys_u2f.OutputName = muscle_names;

    % f2tau
    sys_f2tau = ss([],[],[],M);
        sys_f2tau.InputName = muscle_names;
        sys_f2tau.OutputName = joint_DOF_names;
    
    % tau2q
    % State Space Model
    E = eye(2);
    F = zeros(2);
    A_s = [ F,          E;
            -inv(I)*K,  -inv(I)*D];
    B_s = [F, inv(I)]';
    C_s = [E, F];
    D_s = F;
    sys_tau2q = ss(A_s, B_s, C_s, D_s);
        sys_tau2q.InputName = joint_DOF_names;
        sys_tau2q.OutputName = joint_DOF_names;
    
    sys_OpenLoop = sys_tau2q * sys_f2tau * sys_u2f;