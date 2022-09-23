sys_u_des_2_q = Reflex_Model(1, 1.3942, 0.1394, 1, 1);
sys_u_des_2_ma = Descending_Signal_to_Muscle_Activity(1, 1.3942, 0.1394, 1, 1);
sys_OpenLoop = Open_Loop_Model(1,1);

%% Sanity check.
% If I put the output from u_des_2_u_ma through the open-loop model, I
% should get the same thing as I would get straight from u_des_2_q.
% I will check this by calculating the Bode plots of all three models.
% Multiplying the magnitude ratios of sys_u_des_2_ma and sys_OpenLoop
% should yield the same magnitude ratio as sys_u_es_2_q.
% Adding the phase
% shifts of sys_u_des_2_ma and sys_OpenLoop
% should yield the same phase shift as sys_u_es_2_q.

w_Hz = [0:0.1:20];
w_rad_per_s = w_Hz*2*pi;

[M_OL,P_OL] = bode(sys_OpenLoop,w_rad_per_s);
[M_u2ma,P_u2ma] = bode(sys_u_des_2_ma,w_rad_per_s);
[M_u2q,P_u2q] = bode(sys_u_des_2_q,w_rad_per_s);

[M_composite,P_composite] = bode(sys_OpenLoop*sys_u_des_2_ma,w_rad_per_s);

% plot Magnitude Ratios
figure(1); clf
tiledlayout(2,4)
for output = 1:2
    for inpt = 1:4
        nexttile
        plot(w_Hz,squeeze(M_u2q(output,inpt,:)))
        hold on
        plot(w_Hz,squeeze(M_composite(output,inpt,:)))
    end
end
legend("Full System", "Composite System")

% plot Phase Shifts
figure(2); clf
tiledlayout(2,4)
for output = 1:2
    for inpt = 1:4
        nexttile
        plot(w_Hz,squeeze(P_u2q(output,inpt,:)))
        hold on
        plot(w_Hz,squeeze(P_composite(output,inpt,:)))
    end
end