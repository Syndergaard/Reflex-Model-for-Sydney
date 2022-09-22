function sys_u_des_2_u_ma = Descending_Signal_to_Muscle_Activity(posture,fb_mult_P,fb_mult_D,GTO_gain,delay_multiplier)

    %% Load parameters
    addpath('Parameter Matrices')
    % Load Moment Arm, Inertia, Damping, Stiffness, Max Muscle Force Matrices, Spindle Gains, and Delays
    M = readmatrix(['M', num2str(posture), '.csv']);
    I = readmatrix(['I', num2str(posture), '.csv']);
    D = readmatrix('D.csv');
    K = readmatrix('K.csv');
    C = readmatrix('Max Muscle Force.csv');
    SpindleGains = readmatrix('Spindle_Gains.csv');
%     SpindleGains = csvread('Spindle_Gains (not symmetric).csv');
%     SpindleGains = csvread('Spindle_Gains_Calculated.csv');
%     SpindleGains = csvread('Spindle_Gains_Randomized.csv');
    AffDelays = readmatrix('DelaysAfferent.csv') * delay_multiplier;
    EffDelays = readmatrix('DelaysEfferent.csv');
    CentDelays = NaN(4);
    for i = 1:4
        for j = 1:4
            if SpindleGains(i,j) >= 0
                CentDelays(i,j) = 0.001;
            else
                CentDelays(i,j) = 0.0025;
            end
        end
    end
    Aff_Cent_Delays = (repmat(AffDelays,1,4) + CentDelays) * delay_multiplier; % Assumes the central delay is small compared to the efferent delay, but allow delays to be zero if delay_multiplier==0.
    muscle_names = {'ECR';'ECU';'FCR';'FCU'};
    joint_DOF_names = {'WFE';'WRUD'};
    
    %% Forward Path
    sys_fwd_path = ss(eye(4));
        sys_fwd_path.InputDelay = EffDelays;
        sys_fwd_path.InputName = muscle_names;
        sys_fwd_path.OutputName = muscle_names;

    %% Inner Loop Feedback Path (Excitation/Contraction Dynamics & GTO's)
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

    % GTO's
    sys_GTO = ss(diag(GTO_gain*1.27 ./ diag(C)));
        sys_GTO.InputName = muscle_names;
        sys_GTO.OutputName = muscle_names;
        sys_GTO.InternalDelay = Aff_Cent_Delays;

    %% Close Inner Loop
    sys_closed_inner_loop = feedback(sys_fwd_path,sys_GTO*sys_u2f);

    %% Outer Loop Feedback Path

    % We already have u2f

    % f2tau
    sys_f2tau = ss(M);
        sys_f2tau.InputName = muscle_names;
        sys_f2tau.OutputName = joint_DOF_names;

    % tau2q - State Space Model
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
    
    % q2DeltaLambda
    sys_q2DeltaLambda = ss([],[],[],-M');
        sys_q2DeltaLambda.InputName = joint_DOF_names;
        sys_q2DeltaLambda.OutputName = muscle_names;

    % DeltaLambda2u_fb
        G = cat(3,fb_mult_D,fb_mult_P).*SpindleGains;
        G = num2cell(G,3);
        G = cellfun(@(x) squeeze(x)', G, 'UniformOutput', false);
    sys_DeltaLambda2u_fb = tf(G, 1);
        sys_DeltaLambda2u_fb.InputName = muscle_names;
        sys_DeltaLambda2u_fb.OutputName = muscle_names;
        sys_DeltaLambda2u_fb.InternalDelay = Aff_Cent_Delays;

    %% Close Outer Loop
    sys_u_des_2_u_ma = feedback(sys_closed_inner_loop,sys_DeltaLambda2u_fb*sys_q2DeltaLambda*sys_tau2q*sys_f2tau*sys_f2tau);

end