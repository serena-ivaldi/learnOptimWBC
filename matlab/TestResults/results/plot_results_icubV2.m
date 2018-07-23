clear all;
filepath = '/home/mcharbonneau/src/learnOptimWBC/matlab/TestResults/results/';
%Retrieve results from simulations
robust_experiments = [4; 8; 11; 14; 17; 21; 22; 28; 29; 30]; %23
performance_experiments = [5; 9; 12; 15; 18; 31; 32; 33; 34; 35];
performanceRobust_experiments = [6; 10; 13; 16; 19; 20; 24; 25; 26; 27];

%% PLOTS

k = 4;
results_file = strcat(num2str(k), '_iCub_standing_sim_1.0/icubGazeboSim_40s.mat');
load( [filepath results_file])
pose_CoM_desired = permute(pose_CoM_des.Data, [3,1,2]);
pose_lFoot_desired = permute(pose_lFoot_des.Data, [3,1,2]);
pose_rFoot_desired = permute(pose_rFoot_des.Data, [3,1,2]);


j_robust = 1;
j_performance = 1;
j_performanceRobust = 1;
nsuccess_performance = 0;
nsuccess_robust = 0;
nsuccess_performanceRobust = 0;

for k = 4:35
    
    if k ~= 7 && k ~= 23
        results_file = strcat(num2str(k), '_iCub_standing_sim_1.0/icubGazevoV2_5_noise25_wrenches.mat');
        load( [filepath results_file])
    end
    
    if k == 5 %this is the baseline - initial values =)
        pose_CoM_baseline   = pose_CoM;
        pose_lFoot_baseline = pose_lFoot;
        pose_rFoot_baseline = pose_rFoot;
        pos_ZMP_baseline    = ZMP;
        pos_SP_baseline     = permute(support_polygon, [3,1,2]);
        err_ZMP_baseline    = zmpErr;
        torques_baseline    = torques;
    end
    
    if find( k == robust_experiments)
        pose_CoM_robust{j_robust}   = pose_CoM;
        pose_lFoot_robust{j_robust} = pose_lFoot;
        pose_rFoot_robust{j_robust} = pose_rFoot;
        pos_ZMP_robust{j_robust}    = ZMP;
        pos_SP_robust{j_robust}     = permute(support_polygon, [3,1,2]);
        err_ZMP_robust{j_robust}    = zmpErr;
        torques_robust{j_robust}    = torques;
        j_robust = j_robust + 1;
        if size(pose_CoM,1) > 4000
            nsuccess_robust = nsuccess_robust + 1;
        end
        
    elseif find( k == performance_experiments)
        pose_CoM_performance{j_performance}   = pose_CoM;
        pose_lFoot_performance{j_performance} = pose_lFoot;
        pose_rFoot_performance{j_performance} = pose_rFoot;
        pos_ZMP_performance{j_performance}    = ZMP;
        pos_SP_performance{j_performance}     = permute(support_polygon, [3,1,2]);
        err_ZMP_performance{j_performance}    = zmpErr;
        torques_performance{j_performance}    = torques;
        j_performance = j_performance + 1;
        if size(pose_CoM,1) > 4000
            nsuccess_performance = nsuccess_performance + 1;
        end
        
    elseif find( k == performanceRobust_experiments)
        pose_CoM_performanceRobust{j_performanceRobust}   = pose_CoM;
        pose_lFoot_performanceRobust{j_performanceRobust} = pose_lFoot;
        pose_rFoot_performanceRobust{j_performanceRobust} = pose_rFoot;
        pos_ZMP_performanceRobust{j_performanceRobust}    = ZMP;
        pos_SP_performanceRobust{j_performanceRobust}     = permute(support_polygon, [3,1,2]);
        err_ZMP_performanceRobust{j_performanceRobust}    = zmpErr;
        torques_performanceRobust{j_performanceRobust}    = torques;
        j_performanceRobust = j_performanceRobust + 1;
        if size(pose_CoM,1) > 4000
            nsuccess_performanceRobust = nsuccess_performanceRobust + 1;
        end
        
 
    end
end

%% Success rates
nsuccess = 0;
for k = 1:10
    results_file = strcat('102_iCub_standing_sim_1.0/', num2str(k), '_of_102_iCub_standing_sim_1.0/icubGazevoV2_5_noise25_wrenches.mat');
    load( [filepath results_file])
    pose_CoM_noDR{k} = pose_CoM;
    pose_lFoot_noDR{k} = pose_lFoot;
    pose_rFoot_noDR{k} = pose_rFoot;
    if size(pose_CoM,1) > 4000
        nsuccess = nsuccess + 1;
    end
end


fprintf('Success rate of performance experiments: %.1f %% \n', nsuccess_performance/(j_performance-1)*100);
fprintf('Success rate of robust experiments: %.1f %% \n', nsuccess_robust/(j_robust-1)*100);
%assume 10 experiments, and 103_9 was successful -- NOT
fprintf('Success rate of performanceRobust experiments: %.1f %% \n', (nsuccess_performanceRobust)/(j_performanceRobust-1)*100); 
fprintf('Success rate of experiments without DR: %d %% \n', nsuccess/10*100)

%% Plot formats
time = 0:0.01:40;


width = 3;     % Width in inches
height = 3;    % Height in inches
alw = 0.75;    % AxesLineWidth
fsz = 20; %15      % Fontsize
lw = 1.5;      % LineWidth
msz = 8;       % MarkerSize

% The properties we've been using in the figures
set(0,'defaultLineLineWidth',lw);   % set the default line width to lw
set(0,'defaultLineMarkerSize',msz); % set the default line marker size to msz
set(0,'defaultLineLineWidth',lw);   % set the default line width to lw
set(0,'defaultLineMarkerSize',msz); % set the default line marker size to msz
set(0,'defaultAxesFontSize',fsz); %set the fontsize t0 fsz

% Set the default Size for display
defpos = get(0,'defaultFigurePosition');
set(0,'defaultFigurePosition', [defpos(1) defpos(2) width*100, height*100]);

% Set the defaults for saving/printing to a file
set(0,'defaultFigureInvertHardcopy','on'); % This is the default anyway
set(0,'defaultFigurePaperUnits','inches'); % This is the default anyway
defsize = get(gcf, 'PaperSize');
left = (defsize(1)- width)/2;
bottom = (defsize(2)- height)/2;
defsize = [left, bottom, width, height];
set(0, 'defaultFigurePaperPosition', defsize);

close all;

%% CoM phase plots x v.s. y
CoM_axis = [-15 30 -20 150];

figure();
plot(pose_CoM_desired(:,1)*1000, pose_CoM_desired(:,2)*1000)
xlabel('CoM_x (mm)');
ylabel('CoM_y (mm)');
axis(CoM_axis);
title('Reference');
set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);
set(gca,'box','off')

figure(); %- it's just falling
plot(pose_CoM_baseline(:,1)*1000, pose_CoM_baseline(:,2)*1000)
% title('Phase plot of CoM position, obtained with initial weights');
xlabel('CoM_x (mm)');
ylabel('CoM_y (mm)');
axis(CoM_axis);
% set(gca,'yticklabel',[])
set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);
set(gca,'box','off')

figure();
for k = 1:j_robust-1 %k = 1, 2, 4 are falling
    hold on;
    if size(pose_CoM_robust{k},1) == length(time) %1,2, 4 fail
        plot(pose_CoM_robust{k}(:,1)*1000, pose_CoM_robust{k}(:,2)*1000)
    end
end
title('w{\phi_r}');
% legend('Training 1', 'Training 2');
ylabel('CoM_y (mm)');
xlabel('CoM_x (mm)');
axis(CoM_axis);
% set(gca,'yticklabel',[])
set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);
set(gca,'box','off')

figure()
for k = 1:j_performance-1
    hold on;
    if size(pose_CoM_performance{k},1) == length(time)
        plot(pose_CoM_performance{k}(:,1)*1000, pose_CoM_performance{k}(:,2)*1000)
    end
end
% title('Phase plot of CoM, obtained with weights optimized using \phi_p');
% legend('Training 1', 'Training 2');
% ylabel('CoM_y (mm)');
xlabel('CoM_x (mm)');
axis(CoM_axis);
set(gca,'yticklabel',[])
set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);
set(gca,'box','off')

figure()
for k = 1:j_performanceRobust-1
    hold on;
    if k > 3 %just plot the first 3
    elseif size(pose_CoM_performanceRobust{k},1) == length(time) 
        plot(pose_CoM_performanceRobust{k}(:,1)*1000, pose_CoM_performanceRobust{k}(:,2)*1000)
    end
end
%title('Robot');
title('w{\phi_{pr}}');
% legend('Training 1', 'Training 2', 'Training 3');
%ylabel('CoM_y (mm)');
xlabel('CoM_x (mm)');
axis(CoM_axis);
% set(gca,'yticklabel',[])
set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);
set(gca,'box','off')


figure()
for k = 1:10
    hold on;
    if size(pose_CoM_noDR{k},1) == length(time) 
        plot(pose_CoM_noDR{k}(:,1)*1000, pose_CoM_noDR{k}(:,2)*1000)
    end
end
%title('Robot');
title('w{\phi_{pr}} no DR');
% legend('Training 1', 'Training 2', 'Training 3');
%ylabel('CoM_y (mm)');
xlabel('CoM_x (mm)');
axis(CoM_axis);
set(gca,'yticklabel',[])
set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);
set(gca,'box','off')


%FOR LEGEND -- use the one from CoM_legend =)
    fig = figure(); %this one is just for the axis!
    for k = 1:j_performanceRobust-1
        hold on;
        plot(pose_CoM_performanceRobust{k}(:,1)*1000, pose_CoM_performanceRobust{k}(:,2)*1000)
    end
    xlabel('CoM_x (mm)');
    ylabel('y (mm)');
    set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);
    legendHandle = legend('T1', 'T2', 'T3', 'T4');
    legend('Orientation','horizontal');
    legend('Location','bestoutside')
%    saveLegendToImage(fig, legendHandle, 'CoM_legend', 'jpg');

%% Foot trajectories
%foot_axis = [0 40 0 50];
    foot_axis = [0 40 0 100];

cc=lines(1);    
figure();
plot(time,pose_lFoot_desired(:,3)*100000 - pose_lFoot_desired(1,3)*100000, 'Color', cc(1,:) );
hold on;
plot(time,pose_rFoot_desired(:,3)*100000, 'Color', cc(1,:) );
title('Reference');
xlabel('time (s)');
ylabel('foot_z (mm)');
axis(foot_axis);
set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);
    

% figure(); %it just fell
% plot(time,pose_lFoot_baseline(:,3)*10000);
% hold on;
% plot(time,pose_rFoot_baseline(:,3)*10000);
% xlabel('time (s)');
% ylabel('z (mm)');
% title('Feet trajectories, obtained with initial weights');
% legend('Left foot', 'Right foot');

figure() %lFoot z position
for k = 1:j_robust-1
    hold on;
    if size(pose_lFoot_robust{k},1) == length(time) %2 failed %1,2,4 one failed
    plot(time,pose_lFoot_robust{k}(:,3)*10000);
    plot(time,pose_rFoot_robust{k}(:,3)*10000);
    end
end
% legend('left foot, training 1', 'right foot, training 1', 'left foot, training 2', 'right foot, training 2');
title('w{\phi_r}');
xlabel('time (s)');
ylabel('foot_z (mm)');
axis(foot_axis);
set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);

figure() %lFoot z position %they all failed
for k = 1:j_performance-1
    hold on;
    if size(pose_lFoot_performance{k},1) == length(time)
        plot(time,pose_lFoot_performance{k}(:,3)*10000);
        plot(time,pose_rFoot_performance{k}(:,3)*10000);
    end
end
xlabel('time (s)');
% ylabel('z (mm)');
% legend('left foot, training 1', 'right foot, training 1', 'left foot, training 2', 'right foot, training 2');
% title('Feet trajectories, obtained with weights optimized using \phi_p');
axis(foot_axis);
set(gca,'yticklabel',[])
set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);

figure() %lFoot z position
for k = 1:j_performanceRobust-1
    hold on;
    if size(pose_lFoot_performanceRobust{k},1) == length(time) %2 failed
        plot(time,pose_lFoot_performanceRobust{k}(:,3)*10000);
        plot(time,pose_rFoot_performanceRobust{k}(:,3)*10000);
    end
end
% ylabel('z (mm)');
% legend('left foot, training 1', 'right foot, training 1', 'left foot, training 2', 'right foot, training 2');
title('w{\phi_{pr}}');
xlabel('time (s)');
axis(foot_axis);
set(gca,'yticklabel',[])
set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);

%ONE TRAINING ONE COLOUR
figure() %lFoot z position
cc=lines(j_performanceRobust-1);
for k = 1:j_performanceRobust-1
    hold on;
    if k > 3
    elseif size(pose_lFoot_performanceRobust{k},1) == length(time) %2 failed
        plot(time,pose_lFoot_performanceRobust{k}(:,3)*10000, 'Color', cc(k,:));
        plot(time,pose_rFoot_performanceRobust{k}(:,3)*10000, 'Color', cc(k,:));
    end
end
% ylabel('z (mm)');
% legend('left foot, training 1', 'right foot, training 1', 'left foot, training 2', 'right foot, training 2');
title('w{\phi_{pr}}');
% title('Robot');
xlabel('time (s)');
ylabel('foot_z (mm)');
axis(foot_axis);
% set(gca,'yticklabel',[])
set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);


figure() %lFoot z position
cc=lines(3);
i = 0;
for k = 1:10
    hold on;
    if size(pose_lFoot_noDR{k},1) == length(time) %2 failed
        i = i + 1;
        plot(time,pose_lFoot_noDR{k}(:,3)*1000, 'Color', cc(i,:));
        plot(time,pose_rFoot_noDR{k}(:,3)*1000, 'Color', cc(i,:));
    end
end
% ylabel('z (mm)');
% legend('left foot, training 1', 'right foot, training 1', 'left foot, training 2', 'right foot, training 2');
title('w{\phi_{pr}} no DR');
% title('Robot');
xlabel('time (s)');
%ylabel('foot_z (mm)');
axis(foot_axis);
set(gca,'yticklabel',[])
set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);

%For legend
%     fig = figure(); %lFoot z position
%     % title('Feet trajectories, obtained with weights optimized using \phi_{pr}');
%     for k = 1:10
%         hold on;
%         plot(time,pose_lFoot_noDR{k}(:,3)*10000);
%         plot(time,pose_rFoot_noDR{k}(:,3)*10000);
%     end
%     xlabel('time (s)');
% %     ylabel('z (mm)');
%     axis(foot_axis);
%     set(gca,'yticklabel',[])
%     set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);
%     legend('Orientation','horizontal');
%     legendHandle = legend('L_{T1}', 'R_{T1}', 'L_{T2}', 'R_{T2}', 'L_{T3}', 'R_{T3}', 'L_{T4}', 'R_{T4}');
%     legend('Location','bestoutside')
% %    saveLegendToImage(fig, legendHandle, 'feet_legend', 'jpg');
% 
%% Torques, ZMP too noisy (due to artificial noise)
% %% torques
% figure() %torques
% Fs = 1000;
% d = designfilt('bandstopiir','FilterOrder',2, ...
%                'HalfPowerFrequency1',40,'HalfPowerFrequency2',500, ...
%                'DesignMethod','butter','SampleRate', Fs);
%            x = torques_performanceRobust{k}(:,3);
%                buttLoop = filtfilt(d,x);
%     plot(time, buttLoop);
% 
% for k = 1:j_performanceRobust-1
%     hold on;
%     if k == 2 %this one fails
%     else
%         plot(time,torques_performanceRobust{k}(:,3));
%     end
% end
% xlabel('time (s)');
% ylabel('\tau (Nm)');
% legend('left foot, training 1', 'right foot, training 1', 'left foot, training 2', 'right foot, training 2');
% title('Joint torques, obtained with weights optimized using \phi_{pr}');


% %Plot CoM position %y-direction CoM
% plot(time, pose_CoM_desired(:,2));
% hold on;
% plot(time, pose_CoM_baseline(:,2));
% for k = 1:j_robust-1
%     if length(pose_CoM_robust{k}) < length(time) %robot fell
%         plot(time, [pose_CoM_robust{k}(:,2); zeros(length(time)-length(pose_CoM_robust{k}),1)]);
%     else
%         plot(time,pose_CoM_robust{k}(:,2));
%     end
% end
% for k = 1:j_performance-1
%     plot(time,pose_CoM_performance{k}(:,2));
% end
% xlabel('time');
% ylabel('CoM_y');


%% ZMP x plots -- TOO NOISY
% 
% % figure(); %ZMP v.s. SP
% % hold on;
% % plot(time, 0.5 * (pos_SP_baseline(1,1,1)*1000 * ones(size(time)) + pos_SP_baseline(1,1,2)*1000 * ones(size(time))));
% % plot(time, pos_SP_baseline(1,1,1)*1000 * ones(size(time)), '-r');
% % plot(time, pos_SP_baseline(1,1,2)*1000 * ones(size(time)), '-r');
% % xlabel('time (s)');
% % ylabel('x_{ZMP} (mm)');
% % title('ZMP trajectory, ideal');
% % legend('x_{ZMP}', 'SP bounds');
% 
% 
% % figure(); %ZMP v.s. SP
% % plot(time,pos_ZMP_baseline(:,1)*1000);
% % hold on;
% % plot(time, pos_SP_baseline(:,1,1)*1000, '-r');
% % plot(time, pos_SP_baseline(:,1,2)*1000, '-r');
% % xlabel('time (s)');
% % ylabel('x_{ZMP} (mm)');
% % title('ZMP trajectory, obtained with initial weights');
% % legend('x_{ZMP}', 'SP bounds');
% 
% figure();
% for k = 1:j_robust-1
%     hold on;
%     if k == 2 || k == 1 || k == 4 %this one fails
%     else
%     plot(time, pos_ZMP_robust{k}(:,1)*1000);
%     end
% end
% plot(time, pos_SP_robust{3}(:,1,1)*1000, '-r');
% plot(time, pos_SP_robust{3}(:,1,2)*1000, '-r');
% xlabel('time (s)');
% ylabel('x_{ZMP} (mm)');
% title('ZMP trajectory, obtained with with weights optimized using \phi_r');
% legend('x_{ZMP}, training 1', 'x_{ZMP}, training 2', 'SP bounds');
% 
% 
% % figure();
% % for k = 1:j_performance-1
% %     hold on;
% %     if k == 3 %this one fails
% %     else
% %     plot(time, pos_ZMP_performance{k}(:,1)*1000);
% %     end
% % end
% % plot(time, pos_SP_performance{1}(:,1,1)*1000, '-r');
% % plot(time, pos_SP_performance{1}(:,1,2)*1000, '-r');
% % xlabel('time (s)');
% % ylabel('x_{ZMP} (mm)');
% % title('ZMP trajectory, obtained with with weights optimized using \phi_p');
% % legend('x_{ZMP}, training 1', 'x_{ZMP}, training 2', 'SP bounds');
% 
% figure();
% for k = 1:j_performanceRobust-1
%     hold on;
%     if k == 2 %this one fails
%     else
%     plot(time, pos_ZMP_performanceRobust{k}(:,1)*1000);
%     end
% end
% plot(time, pos_SP_performanceRobust{1}(:,1,1)*1000, '-r');
% plot(time, pos_SP_performanceRobust{1}(:,1,2)*1000, '-r');
% xlabel('time (s)');
% ylabel('x_{ZMP} (mm)');
% title('ZMP trajectory, obtained with with weights optimized using \phi_{pr}');
% legend('x_{ZMP}, training 1', 'x_{ZMP}, training 2', 'SP bounds');


%% ZMP y plots -- TOO NOISY

% figure(); %ZMP v.s. SP
% hold on;
% plot(time, 0.5 * (pos_SP_baseline(2,1,1)*1000 * ones(size(time)) + pos_SP_baseline(2,1,2)*1000 * ones(size(time))));
% plot(time, pos_SP_baseline(2,1,1)*1000 * ones(size(time)), '-r');
% plot(time, pos_SP_baseline(2,1,2)*1000 * ones(size(time)), '-r');
% xlabel('time (s)');
% ylabel('y_{ZMP} (mm)');
% title('ZMP trajectory, ideal');
% legend('y_{ZMP}', 'SP bounds');
% 
% 
% figure(); %ZMP v.s. SP
% plot(time,pos_ZMP_baseline(:,2)*1000);
% hold on;
% plot(time, pos_SP_baseline(:,2,1)*1000, '-r');
% plot(time, pos_SP_baseline(:,2,2)*1000, '-r');
% xlabel('time (s)');
% ylabel('y_{ZMP} (mm)');
% title('ZMP trajectory, obtained with initial weights');
% legend('y_{ZMP}', 'SP bounds');
% 
% figure();
% for k = 1:j_robust-1
%     hold on;
%     if k == 2 %this one fails
%     else
%     plot(time, pos_ZMP_robust{k}(:,2)*1000);
%     end
% end
% plot(time, pos_SP_robust{1}(:,2,1)*1000, '-r');
% plot(time, pos_SP_robust{1}(:,2,2)*1000, '-r');
% xlabel('time (s)');
% ylabel('y_{ZMP} (mm)');
% title('ZMP trajectory, obtained with with weights optimized using \phi_r');
% legend('y_{ZMP}, training 1', 'y_{ZMP}, training 2', 'SP bounds');
% 
% 
% figure();
% for k = 1:j_performance-1
%     hold on;
%     if k == 3 %this one fails
%     else
%     plot(time, pos_ZMP_performance{k}(:,2)*1000);
%     end
% end
% plot(time, pos_SP_performance{1}(:,2,1)*1000, '-r');
% plot(time, pos_SP_performance{1}(:,2,2)*1000, '-r');
% xlabel('time (s)');
% ylabel('y_{ZMP} (mm)');
% title('ZMP trajectory, obtained with with weights optimized using \phi_p');
% legend('y_{ZMP}, training 1', 'y_{ZMP}, training 2', 'SP bounds');
% 
% figure();
% for k = 1:j_performanceRobust-1
%     hold on;
%     plot(time, pos_ZMP_performanceRobust{k}(:,2)*1000);
% end
% plot(time, pos_SP_performanceRobust{2}(:,2,1)*1000, '-r');
% plot(time, pos_SP_performanceRobust{2}(:,2,2)*1000, '-r');
% xlabel('time (s)');
% ylabel('y_{ZMP} (mm)');
% title('ZMP trajectory, obtained with with weights optimized using \phi_{pr}');
% legend('y_{ZMP}, training 1', 'y_{ZMP}, training 2', 'y_{ZMP}, training 3', 'SP bounds');
