posture = 1;

Kgto_range = [0,2.7]/1.27; % Rozendaal 1997 pg 81 (last sentence on page)
Kp_range = [0, 1.3942]; % Default Gains Matrix & Posture 1

Kd_Kp_RATIO_range = [0.06,0.14]; % Winters & Crago Ch 11 pg 172
delay_multiplier_range = [0.75,1.5];

n = 6;

delta = diff(Kgto_range) / (n-1);                   Kgto = [Kgto_range(1):delta:Kgto_range(2)];
delta = diff(Kp_range) / (n-1);                     Kp = [Kp_range(1):delta:Kp_range(2)];
delta = diff(Kd_Kp_RATIO_range) / (n-1);            Kd_Kp_RATIO = [Kd_Kp_RATIO_range(1):delta:Kd_Kp_RATIO_range(2)];
delta = diff(delay_multiplier_range) / (n-1);       delay_multiplier = [delay_multiplier_range(1):delta:delay_multiplier_range(2)];

clearvars -except Kgto Kp Kd_Kp_RATIO delay_multiplier n posture average_mags last_indices peak_frequencies peak_mags

% EXCLUDE Kp = 0
w_low = 4*2*pi; w_high = 8*2*pi;
for i = 1:n
    GTO_gain = Kgto(i);
    for j = 2:n % Starts at 2 to avoid repeats of no muscle spindles (if starting at 1, we'd have repeat models for all values of Kd_Kp_RATIO)
        fb_gain_P = Kp(j);
        Kd = fb_gain_P * Kd_Kp_RATIO;
        for k = 1:n
            fb_gain_D = Kd(k);
            for l = 1:n
                delay_mult = delay_multiplier(l);

                % Create sys and get magnitude ratios
                sys = Full_Model(posture,fb_gain_P,fb_gain_D,GTO_gain,delay_mult);
                [mag,phase,wout] = bode(sys,{w_low,w_high});
                wout = wout/(2*pi);

                for input = 1:4
                    mag_input = squeeze(mag(:,input,:));
                    for output = 1:2
                        mag_input_output = mag_input(output,:);

                        ave = trapz(wout,mag_input_output)/(w_high-w_low);
                        average_mags(i,j-1,k,l,output,input) = ave;

                        [max_val,max_ind] = max(mag_input_output);
                        peak_mags(i,j-1,k,l,output,input) = max_val;
                        peak_frequencies(i,j-1,k,l,output,input) = wout(max_ind);

                        disp(['Average Magnitude: ', num2str(ave)])
                        disp(['Maxiumum Peak Magnitude: ', num2str(max_val)])
                        disp(['Frequency of Peak: ', num2str(wout(max_ind))])
                        disp([i,j,k,l,output,input])

                    end
                end

            end
        end
        last_indices = [i,j,k,l,output,input];
        save('Param_Space_Results_exclude_Kp_equals_0.mat','average_mags','peak_mags','peak_frequencies','last_indices')
    end
end

% GTO only (and no open-loop)
clearvars -except w_low w_high Kgto Kp Kd_Kp_RATIO delay_multiplier n posture

average_mags = NaN([n-1,1,1,n,2,4]);
peak_mags = NaN([n-1,1,1,n,2,4]);
peak_frequencies = NaN([n-1,1,1,n,2,4]);
for i = 2:n
    GTO_gain = Kgto(i);
    for j = 1
        fb_gain_P = Kp(j);
        Kd = fb_gain_P * Kd_Kp_RATIO;
        for k = 1
            fb_gain_D = Kd(k);
            for l = 1:n
                delay_mult = delay_multiplier(l);
                
                % Create sys and get magnitude ratios
                sys = Full_Model(posture,fb_gain_P,fb_gain_D,GTO_gain,delay_mult);
                [mag,phase,wout] = bode(sys,{w_low,w_high});
                wout = wout/(2*pi);

                for input = 1:4
                    mag_input = squeeze(mag(:,input,:));
                    for output = 1:2
                        mag_input_output = mag_input(output,:);

                        ave = trapz(wout,mag_input_output)/(w_high-w_low);
                        average_mags(i-1,j,k,l,output,input) = ave;

                        [max_val,max_ind] = max(mag_input_output);
                        peak_mags(i-1,j,k,l,output,input) = max_val;
                        peak_frequencies(i-1,j,k,l,output,input) = wout(max_ind);

                        disp(['Average Magnitude: ', num2str(ave)])
                        disp(['Maxiumum Peak Magnitude: ', num2str(max_val)])
                        disp(['Frequency of Peak: ', num2str(wout(max_ind))])
                        disp([i,j,k,l,output,input])

                    end
                end
            end
        end
        last_indices = [i,j,k,l,output,input];
        save('Param_Space_Results_GTO_only.mat','average_mags','peak_mags','peak_frequencies','last_indices')
    end
end

% Open Loop
clearvars -except w_low w_high Kgto Kp Kd_Kp_RATIO delay_multiplier n posture

average_mags = NaN([1,1,1,1,2,4]);
peak_mags = NaN([1,1,1,1,2,4]);
peak_frequencies = NaN([1,1,1,1,2,4]);

    % Create sys and get magnitude ratios
sys = Full_Model(posture,0,0,0,0);
[mag,phase,wout] = bode(sys,{w_low,w_high});
wout = wout/(2*pi);

for input = 1:4
    mag_input = squeeze(mag(:,input,:));
    for output = 1:2
        mag_input_output = mag_input(output,:);

        ave = trapz(wout,mag_input_output)/(w_high-w_low);
        average_mags(1,1,1,1,output,input) = ave;

        [max_val,max_ind] = max(mag_input_output);
        peak_mags(1,1,1,1,output,input) = max_val;
        peak_frequencies(1,1,1,1,output,input) = wout(max_ind);

        disp(['Average Magnitude: ', num2str(ave)])
        disp(['Maxiumum Peak Magnitude: ', num2str(max_val)])
        disp(['Frequency of Peak: ', num2str(wout(max_ind))])
        disp([1,1,1,1,output,input])

    end
end

save('Param_Space_Results_Open_Loop.mat','average_mags','peak_mags','peak_frequencies')