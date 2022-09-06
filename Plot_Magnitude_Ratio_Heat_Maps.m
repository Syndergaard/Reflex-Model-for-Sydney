%%% REMEMBER TO PERFORM STATISTICS ON LOG TRANSFORMED DATA %%%
muscle_names = {'ECR';'ECU';'FCR';'FCU'};
joint_DOF_names = {'WFE';'WRUD'};

% Load and reshape Open Loop data
load('Param_Space_Results_Open_Loop.mat');
ave_mags_Open_Loop = squeeze(average_mags)';

% Load and reshape data from which Kp = 0
load('Param_Space_Results_exclude_Kp_equals_0.mat');
n = size(average_mags,1);
ave_mags_exclude_Kp_0 = nan(4,2,n*(n-1)*n*n);
for i = 1:4
    for j = 1:2
        ave_mags_exclude_Kp_0(i,j,:) = reshape(average_mags(:,:,:,:,j,i),[1,1,n*(n-1)*n*n]);
    end
end

% Reshape PD data (no need to reload, b/c it's contained within the Kp=0 data)
average_mags_PD = average_mags(1,:,:,:,:,:);
ave_mags_PD = nan(4,2,1*(n-1)*n*n);
for i = 1:4
    for j = 1:2
        ave_mags_PD(i,j,:) = reshape(average_mags_PD(:,:,:,:,j,i),[1,1,1*(n-1)*n*n]);
    end
end
means_PD_summary = exp(mean(log(cat(3,ave_mags_PD,ave_mags_Open_Loop)),3));
std_PD_summary = exp(std(log(cat(3,ave_mags_PD,ave_mags_Open_Loop)),0,3));

% Load and reshape GTO data
load('Param_Space_Results_GTO_only.mat');
ave_mags_FF = nan(4,2,(n-1)*1*1*n);
for i = 1:4
    for j = 1:2
        ave_mags_FF(i,j,:) = reshape(average_mags(:,:,:,:,j,i),[1,1,(n-1)*1*1*n]);
    end
end
    % Note that I use the mean() and std() on the log transformed data
means_FF_summary = exp(mean(log(cat(3,ave_mags_FF,ave_mags_Open_Loop)),3));
std_FF_summary = exp(std(log(cat(3,ave_mags_FF,ave_mags_Open_Loop)),0,3));

% Concatenate all data
average_mags = cat(3,ave_mags_Open_Loop,ave_mags_FF,ave_mags_exclude_Kp_0);
means_FullModel_summary = exp(mean(log(average_mags),3));
std_FullModel_summary = exp(std(log(average_mags),0,3));

% Plot averages
figure(1); TL1 = tiledlayout(2,2,'TileSpacing','compact'); ax1{1} = nexttile;
imagesc(ave_mags_Open_Loop)
xticks(1:2); yticks(1:4)
xticklabels([]); yticklabels(muscle_names)
title("Open-Loop")
cl1 = caxis;
set(gca,'fontsize',9)

ax1{2} = nexttile;
imagesc(means_FF_summary)
xticks(1:2); yticks(1:4)
xticklabels([]); yticklabels([])
title("Force Feedback Only")
cl1(end+1,:) = caxis;
set(gca,'fontsize',9)

ax1{3} = nexttile;
imagesc(means_PD_summary)
xticks(1:2); yticks(1:4)
xticklabels(joint_DOF_names); yticklabels(muscle_names)
title("PD Feedback Only")
cl1(end+1,:) = caxis;
set(gca,'fontsize',9)

ax1{4} = nexttile;
imagesc(means_FullModel_summary)
xticks(1:2); yticks(1:4)
xticklabels(joint_DOF_names); yticklabels([])
title("All Data")
cl1(end+1,:) = caxis;
set(gca,'fontsize',9)

% Force all plots to have the same color axis
maxcolor = max(max(cl1));
for i = 1:4
    caxis(ax1{i}, [0,maxcolor])
end
cb = colorbar;
cb.Layout.Tile = 'east';
caxis([0,maxcolor])

% Calculate 2D correlation coefficients as compared to open loop
FF_correlation = corr2(ave_mags_Open_Loop,means_FF_summary)
PD_correlation = corr2(ave_mags_Open_Loop,means_PD_summary)
Full_correlation = corr2(ave_mags_Open_Loop,means_FullModel_summary)