clear all;
filepath = '/home/mcharbonneau/src/learnOptimWBC/matlab/TestResults/results/';
%Retrieve results from simulations
robust_experiments = [4; 8; 11; 14; 17; 21; 22; 28; 29; 30]; %23
performance_experiments = [5; 9; 12; 15; 18; 31; 32; 33; 34; 35];
performanceRobust_experiments = [6; 10; 13; 16; 19; 20; 24; 25; 26; 27];

%YOU ARE FORGETTING TORQUES

time = 0:0.01:40;
j_robust = 1;
j_performance = 1;
j_performanceRobust = 1;
nsuccess_performance = 0;
nsuccess_robust = 0;
nsuccess_performanceRobust = 0;

for k = 4:35
    if k ~= 7 && k ~= 23
        results_file = strcat(num2str(k), '_iCub_standing_sim_1.0/icubGazeboSim_40s.mat');
        load( [filepath results_file])
    end
    
    if j_robust == 1 %get the desired values from the first run
       pose_CoM_desired = permute(pose_CoM_des.Data, [3,1,2]);
       pose_lFoot_desired = permute(pose_lFoot_des.Data, [3,1,2]);
       pose_rFoot_desired = permute(pose_rFoot_des.Data, [3,1,2]);
    end
    
    if k == 5 %this is the baseline - initial values =)
        pose_CoM_baseline   = permute(pose_CoM.Data, [3,1,2]);
        pose_lFoot_baseline = permute(pose_lFoot.Data, [3,1,2]);
        pose_rFoot_baseline = permute(pose_rFoot.Data, [3,1,2]);
        pos_ZMP_baseline    = ZMP.Data;
        pos_SP_baseline     = permute(support_polygon.Data, [3,1,2]);
        err_ZMP_baseline    = zmpErr.Data;
        torques_baseline    = torques.Data;
    end
    
    if find( k == robust_experiments)
        pose_CoM_robust{j_robust}   = permute(pose_CoM.Data, [3,1,2]);
        pose_lFoot_robust{j_robust} = permute(pose_lFoot.Data, [3,1,2]);
        pose_rFoot_robust{j_robust} = permute(pose_rFoot.Data, [3,1,2]);
        pos_ZMP_robust{j_robust}    = ZMP.Data;
        pos_SP_robust{j_robust}     = permute(support_polygon.Data, [3,1,2]);
        err_ZMP_robust{j_robust}    = zmpErr.Data;
        torques_robust{j_robust}    = torques.Data;
        j_robust = j_robust + 1;
        if size(pose_CoM.Data,3) > 4000
            nsuccess_robust = nsuccess_robust + 1;
        end
        
    elseif find( k == performance_experiments)
        pose_CoM_performance{j_performance} = permute(pose_CoM.Data, [3,1,2]);
        pose_lFoot_performance{j_performance} = permute(pose_lFoot.Data, [3,1,2]);
        pose_rFoot_performance{j_performance} = permute(pose_rFoot.Data, [3,1,2]);
        pos_ZMP_performance{j_performance}    = ZMP.Data;
        pos_SP_performance{j_performance}     = permute(support_polygon.Data, [3,1,2]);
        err_ZMP_performance{j_performance}    = zmpErr.Data;
        torques_performance{j_performance}    = torques.Data;
        j_performance = j_performance + 1;
        if size(pose_CoM.Data,3) > 4000
            nsuccess_performance = nsuccess_performance + 1;
        end
        
    elseif find( k == performanceRobust_experiments)
        pose_CoM_performanceRobust{j_performanceRobust} = permute(pose_CoM.Data, [3,1,2]);
        pose_lFoot_performanceRobust{j_performanceRobust} = permute(pose_lFoot.Data, [3,1,2]);
        pose_rFoot_performanceRobust{j_performanceRobust} = permute(pose_rFoot.Data, [3,1,2]);
        pos_ZMP_performanceRobust{j_performanceRobust}    = ZMP.Data;
        pos_SP_performanceRobust{j_performanceRobust}     = permute(support_polygon.Data, [3,1,2]);
        err_ZMP_performanceRobust{j_performanceRobust}    = zmpErr.Data;
        torques_performanceRobust{j_performanceRobust}    = torques.Data;
        j_performanceRobust = j_performanceRobust + 1;
        if size(pose_CoM.Data,3) > 4000
            nsuccess_performanceRobust = nsuccess_performanceRobust + 1;
        end
 
    end
end

%% Success rates
fprintf('Success rate of performance experiments: %.1f %% \n', nsuccess_performance/(j_performance-1)*100);
fprintf('Success rate of robust experiments: %.1f %% \n', nsuccess_robust/(j_robust-1)*100);
%assume 10 experiments, and 103_9 was successful -- NOT
fprintf('Success rate of performanceRobust experiments: %.1f %% \n', (nsuccess_performanceRobust)/(j_performanceRobust-1)*100); 

close all;
%% Plot formats

width = 3;     % Width in inches
height = 3;    % Height in inches
alw = 0.75;    % AxesLineWidth
fsz = 15;      % Fontsize
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

a = 3;
%a = 1: show CoM plots
%a = 2: show feet plots
%a = 3: show ZMPx plots
%a = 4: show ZMPy plots

if a == 1
    %% CoM phase plots x v.s. y
    CoM_axis = [-15 30 -20 150]; %CoM_axis = [0 30 0 140];
    
%     figure();
%     plot(pose_CoM_desired(:,1)*1000, pose_CoM_desired(:,2)*1000)
%     xlabel('x (mm)');
%     ylabel('y (mm)');
%     axis(CoM_axis);
%     % title('Phase plot of desired CoM position');
    
    figure();
    plot(pose_CoM_baseline(:,1)*1000, pose_CoM_baseline(:,2)*1000)
    xlabel('CoM_x (mm)');
    ylabel('CoM_y (mm)');
    axis(CoM_axis);
    % set(gca,'yticklabel',[])
    set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);
    set(gca,'box','off')
    title('w_0');
    
    figure();
    for k = 1:j_robust-1 %k = 1, 3 are not falling
        hold on;
        if size(pose_CoM_robust{k},1) < length(time) %this one fails
        elseif k > 4
        else
            plot(pose_CoM_robust{k}(:,1)*1000, pose_CoM_robust{k}(:,2)*1000)
        end
    end
    xlabel('CoM_x (mm)');
    % ylabel('y (mm)');
    axis(CoM_axis);
    set(gca,'yticklabel',[])
    set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);
    title('w{\phi_r}');
    % legend('T1', 'T2', 'T3');
    % legend('Orientation','horizontal');
    % legend('Location','northoutside')
    
    figure()
    i = 0;
    for k = 1:j_performance-1
        hold on;
        if size(pose_CoM_performance{k},1) < length(time) %this one fails
        else
            if i < 3
            i = i + 1;
            plot(pose_CoM_performance{k}(:,1)*1000, pose_CoM_performance{k}(:,2)*1000)
            end
        end
    end
    xlabel('CoM_x (mm)');
    % ylabel('y (mm)');
    axis(CoM_axis);
    set(gca,'yticklabel',[])
    set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);
    title('w{\phi_p}');
    % legend('T1', 'T2', 'T3');
    %legend('Orientation','horizontal');
    %legend('Location','northoutside');
    
    figure()
    for k = 1:j_performanceRobust-1
        hold on;
        if k > 3
        else
        plot(pose_CoM_performanceRobust{k}(:,1)*1000, pose_CoM_performanceRobust{k}(:,2)*1000)
        end
    end
    xlabel('CoM_x (mm)');
    % ylabel('y (mm)');
    axis(CoM_axis);
    set(gca,'yticklabel',[])
    set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);
    title('w{\phi_{pr}}');
    
    
%     fig = figure(); %this one is just for the legend!
%     for k = 1:3 %j_performanceRobust-1
%         hold on;
%         plot(pose_CoM_performanceRobust{k}(:,1)*1000, pose_CoM_performanceRobust{k}(:,2)*1000)
%     end
%     xlabel('CoM_x (mm)');
%     ylabel('y (mm)');
%     set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);
%     legendHandle = legend('T1', 'T2', 'T3');
%     legend('Orientation','horizontal');
%     legend('Location','bestoutside')
%     saveLegendToImage(fig, legendHandle, 'CoM_legend', 'jpg');
    
elseif a == 2
    %% Foot trajectories
    foot_axis = [0 40 0 100];
    
    % figure();
    % % plot(pose_lFoot_desired(:,1)*100000, pose_lFoot_desired(:,3)*100000)
    % % xlabel('x (mm)');
    % % ylabel('z (mm)');
    % % title('Phase plot of desired left foot position');
    % plot(time,pose_lFoot_desired(:,3)*100000 - pose_lFoot_desired(1,3)*100000 );
    % hold on;
    % plot(time,pose_rFoot_desired(:,3)*100000);
    % xlabel('time (s)');
    % ylabel('z (mm)');
    % axis(foot_axis);
    % % title('Feet desired trajectories');
    % legend('Orientation','horizontal');
    % legend('Left foot', 'Right foot');
    
   
    figure();
    cc=lines(1);
    plot(time,pose_lFoot_baseline(:,3)*10000, 'Color', cc(1,:));
    hold on;
    plot(time,pose_rFoot_baseline(:,3)*10000, 'Color', cc(1,:));
    xlabel('time (s)');
    ylabel('foot_z (mm)');
    axis(foot_axis);
    set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);
    title('w_0');
    % legend('Left foot', 'Right foot');
    set(gca,'box','off')
    
    figure() %lFoot z position
    cc=lines(3);
    i = 0;
    for k = 1:j_robust-1
        hold on;
        if size(pose_CoM_robust{k},1) < length(time) %this one fails
        elseif i < 3
            i = i + 1;
            plot(time,pose_lFoot_robust{k}(:,3)*10000, 'Color', cc(i,:));
            plot(time,pose_rFoot_robust{k}(:,3)*10000, 'Color', cc(i,:));
        end
    end
    xlabel('time (s)');
%     ylabel('z (mm)');
    axis(foot_axis);
    set(gca,'yticklabel',[])
    set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);
    % legend('Orientation','horizontal');
    % legend('left foot, T1', 'right foot, T1', 'left foot, T2', 'right foot, T2');
    title('w{\phi_r}');
    
    figure() %lFoot z position
    cc=lines(10);
    i = 0;
    for k = 1:j_performance-1
        hold on;
        if size(pose_CoM_performance{k},1) < length(time) %this one fails
        elseif i < 3
            i = i + 1;
            plot(time,pose_lFoot_performance{k}(:,3)*10000, 'Color', cc(i,:));
            plot(time,pose_rFoot_performance{k}(:,3)*10000, 'Color', cc(i,:));
        end
    end
    xlabel('time (s)');
%     ylabel('z (mm)');
    axis(foot_axis);
    set(gca,'yticklabel',[])
    set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);
    % legend('left foot, training 1', 'right foot, training 1', 'left foot, training 2', 'right foot, training 2');
    title('w{\phi_p}');
    
    
    figure() %lFoot z position
    cc=lines(3);
    i = 0;
    for k = 1:j_performanceRobust-1
        hold on;
        if k == 3 || k > 4
        else
        i = i + 1;
        plot(time,pose_lFoot_performanceRobust{k}(:,3)*10000, 'Color', cc(i,:));
        plot(time,pose_rFoot_performanceRobust{k}(:,3)*10000, 'Color', cc(i,:));
        end
    end
    xlabel('time (s)');
%     ylabel('z (mm)');
    axis(foot_axis);
    set(gca,'yticklabel',[])
    set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);
    % legend('Orientation','horizontal');
    % legend('left, T1', 'right, T1', 'left, T2', 'right, T2', 'left, T3', 'right, T3');
    title('w{\phi_{pr}}');
    
%     fig = figure(); %lFoot z position
%     % title('Feet trajectories, obtained with weights optimized using \phi_{pr}');
%     for k = 1:j_performanceRobust-1
%         hold on;
%         plot(time,pose_lFoot_performanceRobust{k}(:,3)*10000);
%         plot(time,pose_rFoot_performanceRobust{k}(:,3)*10000);
%     end
%     xlabel('time (s)');
% %     ylabel('z (mm)');
%     axis(foot_axis);
%     set(gca,'yticklabel',[])
%     set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);
%     legend('Orientation','horizontal');
%     legendHandle = legend('T1 left', 'T1 right', 'T2 left', 'T2 right', 'T3 left', 'T3 right');
%     legend('Location','bestoutside')
%    saveLegendToImage(fig, legendHandle, 'feet_legend', 'jpg');
    
elseif a == 3
    
    %% ZMP x plots
    % figure() %ZMP error
    % hold on;
    % for k = 1:j_robust-1
    %     plot(time, err_ZMP_robust{k});
    % end
    % for k = 1:j_performance-1
    %     plot(time, err_ZMP_performance{k});
    % end
    
    % figure(); %ZMP v.s. SP
    % hold on;
    % plot(time, 0.5 * (pos_SP_baseline(1,1,1)*1000 * ones(size(time)) + pos_SP_baseline(1,1,2)*1000 * ones(size(time))));
    % plot(time, pos_SP_baseline(1,1,1)*1000 * ones(size(time)), '-r');
    % plot(time, pos_SP_baseline(1,1,2)*1000 * ones(size(time)), '-r');
    % xlabel('time (s)');
    % ylabel('x_{ZMP} (mm)');
    % title('ZMP trajectory, ideal');
    % legend('x_{ZMP}', 'SP bounds');
    
    zmpx_axis = [0 40 -55 105];
    
    figure(); %ZMP v.s. SP
    plot(time,pos_ZMP_baseline(:,1)*1000);
    hold on;
    plot(time, pos_SP_baseline(:,1,1)*1000, 'Color', [157, 88, 176]/255);
    plot(time, pos_SP_baseline(:,1,2)*1000, 'Color', [157, 88, 176]/255);
    xlabel('time (s)');
    ylabel('ZMP_x (mm)');
    title('w_0');
    axis(zmpx_axis);
    set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);
    set(gca,'box','off')
    % title('ZMP trajectory, obtained with initial weights');
    % legend('x_{ZMP}', 'SP bounds');
    
    figure();
    i = 0;
    for k = 1:j_robust-1
        hold on;
        if size(pose_CoM_robust{k},1) < length(time) %this one fails
        elseif i < 3
            i = i + 1;
            plot(time, pos_ZMP_robust{k}(:,1)*1000);
        end
    end
    plot(time, pos_SP_robust{1}(:,1,1)*1000, 'Color', [157, 88, 176]/255);
    plot(time, pos_SP_robust{1}(:,1,2)*1000, 'Color', [157, 88, 176]/255);
    xlabel('time (s)');
    % ylabel('x_{ZMP} (mm)');
    title('w{\phi_r}');
    % title('ZMP trajectory, obtained with with weights optimized using \phi_r');
    % legend('x_{ZMP}, training 1', 'x_{ZMP}, training 2', 'SP bounds');
    axis(zmpx_axis);
    set(gca,'yticklabel',[])
    set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);
    
    
    figure();
    i = 0;
    for k = 1:j_performance-1
        hold on;
        if size(pose_CoM_performance{k},1) < length(time) %this one fails
        elseif i < 2
            i = i + 1;
            plot(time, pos_ZMP_performance{k}(:,1)*1000);
        end
    end
    plot(time, pos_SP_performance{1}(:,1,1)*1000, 'Color', [157, 88, 176]/255);
    plot(time, pos_SP_performance{1}(:,1,2)*1000, 'Color', [157, 88, 176]/255);
    xlabel('time (s)');
    % ylabel('x_{ZMP} (mm)');
    title('w{\phi_p}');
    axis(zmpx_axis);
    set(gca,'yticklabel',[])
    set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);
    
    figure();
    i = 0;
    for k = 1:j_performanceRobust-1
        hold on;
        if i < 2
            i = i + 1;
            plot(time, pos_ZMP_performanceRobust{k}(:,1)*1000);
        end
    end
    plot(time, pos_SP_performanceRobust{1}(:,1,1)*1000, 'Color', [157, 88, 176]/255);
    plot(time, pos_SP_performanceRobust{1}(:,1,2)*1000, 'Color', [157, 88, 176]/255);
    xlabel('time (s)');
    % ylabel('x_{ZMP} (mm)');
    title('w{\phi_{pr}}');
    axis(zmpx_axis);
    set(gca,'yticklabel',[])
    set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);
    
%     %FOR LEGEND
%     fig = figure();
%     for k = 1:j_performanceRobust-1
%         hold on;
%         plot(time, pos_ZMP_performanceRobust{k}(:,1)*1000);
%     end
%     plot(time, pos_SP_performanceRobust{1}(:,1,1)*1000, 'Color', [157, 88, 176]/255);
%     plot(time, pos_SP_performanceRobust{1}(:,1,2)*1000, 'Color', [157, 88, 176]/255);
%     xlabel('time (s)');
%     % ylabel('x_{ZMP} (mm)');
%     % title('ZMP trajectory, obtained with with weights optimized using \phi_{pr}');
%     axis(zmpx_axis);
%     set(gca,'yticklabel',[])
%     set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);
%     legendHandle = legend('T1', 'T2', 'T3', 'support polygon upper bound', 'support polygon lower bound');
%     legend('Orientation','horizontal');
%     legend('Location','bestoutside')
% %     saveLegendToImage(fig, legendHandle, 'zmpx_legend', 'jpg');

    
elseif a == 4
%% ZMP y plots

% figure(); %ZMP v.s. SP
% hold on;
% plot(time, 0.5 * (pos_SP_baseline(2,1,1)*1000 * ones(size(time)) + pos_SP_baseline(2,1,2)*1000 * ones(size(time))));
% plot(time, pos_SP_baseline(2,1,1)*1000 * ones(size(time)), '-r');
% plot(time, pos_SP_baseline(2,1,2)*1000 * ones(size(time)), '-r');
% xlabel('time (s)');
% ylabel('y_{ZMP} (mm)');
% title('ZMP trajectory, ideal');
% legend('y_{ZMP}', 'SP bounds');

zmpy_axis = [0 40 -45 180];

figure(); %ZMP v.s. SP
plot(time,pos_ZMP_baseline(:,2)*1000);
hold on;
plot(time, pos_SP_baseline(:,2,1)*1000, 'Color', [157, 88, 176]/255);
plot(time, pos_SP_baseline(:,2,2)*1000, 'Color', [157, 88, 176]/255);
xlabel('time (s)');
ylabel('ZMP_y (mm)');
% title('ZMP trajectory, obtained with initial weights');
% legend('y_{ZMP}', 'SP bounds');
axis(zmpy_axis);
set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);
set(gca,'box','off')


figure();
i = 0;
for k = 1:j_robust-1
    hold on;
    if size(pose_CoM_robust{k},1) < length(time) %this one fails
    elseif i < 3
    	i = i + 1;
    plot(time, pos_ZMP_robust{k}(:,2)*1000);
    end
end
plot(time, pos_SP_robust{1}(:,2,1)*1000, 'Color', [157, 88, 176]/255);
plot(time, pos_SP_robust{1}(:,2,2)*1000, 'Color', [157, 88, 176]/255);
xlabel('time (s)');
% ylabel('y_{ZMP} (mm)');
axis(zmpy_axis);
set(gca,'yticklabel',[])
set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);

figure();
i = 0;
for k = 1:j_performance-1
    hold on;
    if size(pose_CoM_performance{k},1) < length(time) %this one fails
    elseif i < 2
    	i = i + 1;
    plot(time, pos_ZMP_performance{k}(:,2)*1000);
    end
end
plot(time, pos_SP_performance{1}(:,2,1)*1000, 'Color', [157, 88, 176]/255);
plot(time, pos_SP_performance{1}(:,2,2)*1000, 'Color', [157, 88, 176]/255);
xlabel('time (s)');
% ylabel('y_{ZMP} (mm)');
axis(zmpy_axis);
set(gca,'yticklabel',[])
set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);

figure();
i = 0;
for k = 1:j_performanceRobust-1
    hold on;
    if i < 2
        i = i + 1;
    plot(time, pos_ZMP_performanceRobust{k}(:,2)*1000);
    end
end
plot(time, pos_SP_performanceRobust{2}(:,2,1)*1000, 'Color', [157, 88, 176]/255);
plot(time, pos_SP_performanceRobust{2}(:,2,2)*1000, 'Color', [157, 88, 176]/255);
xlabel('time (s)');
% ylabel('y_{ZMP} (mm)');
axis(zmpy_axis);
set(gca,'yticklabel',[])
set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);

% 
% fig = figure(); %FOR LEGEND
% for k = 1:j_performanceRobust-1
%     hold on;
%     plot(time, pos_ZMP_performanceRobust{k}(:,2)*1000);
% end
% plot(time, pos_SP_performanceRobust{2}(:,2,1)*1000, 'Color', [157, 88, 176]/255);
% plot(time, pos_SP_performanceRobust{2}(:,2,2)*1000, 'Color', [157, 88, 176]/255);
% xlabel('time (s)');
% % ylabel('y_{ZMP} (mm)');
% % title('ZMP trajectory, obtained with with weights optimized using \phi_{pr}');
% % legend('y_{ZMP}, training 1', 'y_{ZMP}, training 2', 'y_{ZMP}, training 3', 'SP bounds');
%     axis(zmpy_axis);
%     set(gca,'yticklabel',[])
%     set(gca,'LineWidth',1,'TickLength',[0.05 0.05]);
%     legendHandle = legend('T1', 'T2', 'T3', 'SP upper bound', 'SP lower bound');
%     legend('Orientation','horizontal');
%     legend('Location','bestoutside')
%     saveLegendToImage(fig, legendHandle, 'zmpy_legend', 'jpg');

end

