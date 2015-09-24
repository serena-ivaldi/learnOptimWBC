% with this file is possible to draw graph about method robustness 
% to compare methods and to establish if the activation policies have to many
% basis functions(overfitting).
% TODO add time plot
function PlotGraphMultipleExperiment
   
   close all 
   clear variable 
   clc
   
   
   % if i give more than one result folder i will merge the result all
   % togheter
   list_of_folder = {'6_Jaco1.3_scene1.1'};
   list_of_subfolder1 = [5 20 21 38 47 50 74 90 97];  % here i specify the folder that i want to take into account for the plot (in this way i remove all the failures)
   list_of_subfolder2 = [];
   list_of_subfolder = {list_of_subfolder1,list_of_subfolder2};
   % name of the method that will be displayed in the legenda of graph
   name_of_methods = {'RUF + CMA-ES','GHC + CMA-ES'};
   color_list={'b','r','m','g','c','k'};
   % interpolation step for the tau if we use a small step wi will badly
   % capture the value of the applied torque
   interpolation_step = 0.01;
   % SET OF FLAGS TO SWITCH ON AND OFF DIFFERENT FEATURE OF THE METHOD
   variance_flag = true; % to select if i want the variance or not on the fitness graph
   transparent_flag = 1; % make transparent variance in fitness (0 or 1)
   alpha_flag =true;     % plot mena and variance for the alpha 
   position_joint_torque_flag = true;
   tresh_for_count_success_experiment = [-50,-50]; % one for each batch
   
   
   % with this list i decide wich data mat im going to plot for experiment1
%    list_of_data_to_plot = [1,2];
%    style = {'-k','-.r','--b'};
%    experiment1 = false;
   
   % i plot x and y position of end effector of specific experiments
   % and i merge all the plot 
%    list_of_folder_xy = {'_of_115_LBR4p11.0_scene9_UF_mulitple_task_stability_Null_space_projectors','_of_116_LBR4p8.0_scene9_GHC_test_wall_and_two_attractive_point'};
%    experiment_index = [18,1];
%    name_of_methods_xy = {'RUF + CMA-ES','GHC'};
%    style_xy = {'-k','-r','--b'};
%    xy_from_data = false;
   
%    if(experiment1)
%       PlotExpOne(list_of_data_to_plot,style);
%    elseif(xy_from_data)
%       
%       %PlotPositionXYfromDat(list_of_folder_xy,experiment_index,name_of_methods_xy,style_xy,interpolation_step);
%        PlotPositionXYZfromDat(list_of_folder_xy,experiment_index,name_of_methods_xy,style_xy,interpolation_step);
%    else
for i=1:size(list_of_folder,2)
   % here i take only the first because i make the hypotesis that the
   % data necessary here for each series of experiment are all similar
   % to the first of each series
   cur_folder_path = BuildMatFilePath(list_of_folder{i},1);
   [number_of_iteration,controller,time_struct,cur_qi,cur_qdi,cur_fixed_step,cur_scene,bot] = GetAdditionalData(cur_folder_path);
   
   if(isempty(list_of_subfolder{1,i})) 
      list_of_subfolder{1,i} = 1:number_of_iteration;
   end
   controller_first_iteration{i}= controller;
   list_time_struct{i}= time_struct;
   %TODO generalize to multiple experiment
   qi = cur_qi;
   qdi = cur_qdi;
   %----
   fixed_step(i) = cur_fixed_step;
   all_scene{i} = cur_scene;
   all_bot{i} = bot;
end

 all_cur_fitness = [];
 all_cur_alpha   = [];

% here i collect fitness, acitvation function and controllers for each experiment specified in list_of_folder
for i=1:size(list_of_folder,2)

   for j=1:size(list_of_subfolder{1,i},2)
      cur_folder_path = BuildMatFilePath(list_of_folder{i},list_of_subfolder{1,i}(1,j));
      [cur_fitness,cur_alpha,cur_time,cur_control] = GetFitnessAlphaTimeControl(cur_folder_path);
      all_cur_fitness = [all_cur_fitness, cur_fitness];
      all_cur_alpha  =  [all_cur_alpha cur_alpha];
      all_exec_cur_time(1,j) = cur_time;
      all_controller{i,j} = cur_control;
   end

   all_fitness{i} = all_cur_fitness;
   all_alpha{i} = all_cur_alpha;
   all_exec_time{i} = all_exec_cur_time;
   all_cur_fitness = [];
   all_cur_alpha   = [];
   all_exec_cur_time = [];

end

%ComputeMeanVarianceTime(all_exec_time);
%CountSuccessRate(all_fitness,tresh_for_count_success_experiment);

PlotFitness(all_fitness,variance_flag,name_of_methods,color_list,transparent_flag);

if(alpha_flag)
   PlotAlpha(all_alpha,controller_first_iteration,list_time_struct);
end


if(position_joint_torque_flag)

   [all_q,all_tau,all_ee,all_elbow]=ComputeTorqueAndCartesianJointPos(all_controller,list_time_struct,qi,qdi,fixed_step,interpolation_step);

   PlotJoints(all_q,list_time_struct);
   PlotTorque(all_tau,list_time_struct,interpolation_step);
   PlotTrajectory(all_ee,all_scene,list_time_struct,qi,all_bot);
   PlotTrajectory(all_elbow,all_scene,list_time_struct,qi,all_bot);
end

end

% end


% this function give back the path to .mat file of the current experiment 
function result=BuildMatFilePath(folder_name,experiment_index)

   allpath=which('FindData.m');
   path=fileparts(allpath);
   result = strcat(path,'/results/',folder_name,'/',num2str(experiment_index),'_of_',folder_name,'/data.mat');

end

% eith this function i collect all the data that are necessary for the
% subsequent method. the hypothesis is that the firsrt experiment share the
% same data with the others 
function [n_of_iter, control ,t_struct,qinit,qdinit,cur_fixed_step,scene,bot] = GetAdditionalData(cur_mat_path)

   load(cur_mat_path);
    
   n_of_iter = number_of_iteration;
   t_struct = time_sym_struct;
   control = controller;
   qinit  = qi;
   qdinit = qdi;
   cur_fixed_step = fixed_step;
   scene = name_scenario;
   bot = bot1;
end

% with this function i collect best fitness value for each generation and 
% the best alpha combination fo the current experiment
% and the time execution of the experiment
function [cur_fitness,cur_alpha,cur_time,cur_controller]=GetFitnessAlphaTimeControl(cur_mat_path)

   load(cur_mat_path);
   
   for i = 1:size(bestAction.hist,2)
      cur_fitness(i,1) = bestAction.hist(1, i).performance;
   end
   
   cur_time = exec_time;
   cur_alpha = bestAction.parameters';
   % update controller with the current best action
   controller.UpdateParameters(cur_alpha);
   % i have to clean this variable because in fdyn he expect that it is
   % empty
   controller.current_time = [];
   % i have to provide the controller with this new field that i add after
   % taking data
   if(isempty( controller.torques_time))
      controller.torques_time = cell(controller.subchains.GetNumChains());
      for i = 1 :controller.subchains.GetNumChains()
         controller.torques_time{i} = [];
      end
   end
   cur_controller = controller;

end



% with this function i plot the fitness for each exeperiment and if i
% define more folder i merge the result in one graph
function PlotFitness(all_fitness,variance_flag,name_of_methods,color_list,transparent_flag)
   hold on;
   handle_legend = [];
   for i = 1:size(all_fitness,2)
      fit_mean = zeros(size(all_fitness{i},1),1);
      fit_var  = zeros(size(all_fitness{i},1),1);
      

      for j = 1 : size(all_fitness{i},1)
         % i remove from the computation the failure value 
         current_generation = all_fitness{i}(j,:);
         % remove all the failure 
         current_generation( current_generation==-10000000) = [];
         
         fit_mean(j) = mean(current_generation,2);
         if(isempty( fit_mean(j)))
            fit_var (j) = Nan;
         else
            fit_var (j) = std(current_generation);
         end
      end
      
      if(variance_flag)
         % remove all the not a number. it can happens if in one generation
         % i have a failure in each experiment and in that case mean give
         % back a NaN
         fit_mean(isnan(fit_mean)) = [];
         fit_var(isnan(fit_var)) = [];
         generation = 1:size(fit_mean,1);
         h = shadedErrorBar(generation',fit_mean,fit_var,{'r-o','Color',color_list{i},'markerfacecolor',color_list{i}},transparent_flag);
         handle_legend = [handle_legend,h.mainLine];
         xlabel('generations','FontSize',16);
         ylabel('fitness','FontSize',16);
         h_legend = legend(handle_legend,name_of_methods);
         set(h_legend,'FontSize',15);
         
      else
         % remove all the not a number. it can happens if in one generation
         % i have a failure in each experiment and in that case mean give
         % back a NaN
         fit_mean(isnan(fit_mean)) = [];
         generation = 1:size(fit_mean,1);
         plot(generation',fit_mean,'r-o','Color',color_list{i},'markerfacecolor',color_list{i})
         xlabel('generations','FontSize',16);
         ylabel('fitness','FontSize',16);
         h_legend = legend(name_of_methods);
         set(h_legend,'FontSize',15);
         
      end
      final_fit_mean{i} = fit_mean;
      final_fit_var{i}  = fit_var;
   end
   YL = get(gca,'ylim');
   set(gca,'ylim',[YL(1) 0]);
   % if i want to fix the axis
   %set(gca,'ylim',[-600 0])
   set(gca,'Layer','top')
   
   assignin('base','final_fit_mean',final_fit_mean);
   assignin('base','final_fit_var',final_fit_var);
end

% plot mean and std deviation for each alpha having a bunch of experiment
function PlotAlpha(all_alpha,controller,time_struct)

   for k = 1:size(all_alpha,2)
      
      time = time_struct{k}.ti:time_struct{k}.step:time_struct{k}.tf;
      cur_alpha_mean_time = zeros(size(time,1),1);
      cur_alpha_var_time  = zeros(size(time,1),1);
      
      for kk = 1:size(all_alpha{k},2)
         % build numeric theta for best action 
         controller{k}.UpdateParameters(all_alpha{k}(:,kk))

         % plot alpha 
         for ii = 1:size(controller{k}.alpha,1)
             for jj = 1:size(controller{k}.alpha,2)

                 i=1;
                 for t = time
                     vec_values(i,jj*ii) = controller{k}.alpha{ii,jj}.GetValue(t); 
                     i=i+1;
                 end


             end
         end
         % in vec values i have all the value for the k-th experiment for
         % every kinematic chain one for each 
         alphas_time{k,kk} = vec_values;  
      end
      vec_values =[];
   end
   
   for k=1:size(alphas_time,1)
      time = time_struct{k}.ti:time_struct{k}.step:time_struct{k}.tf;
      % compute mean and average and plot it
      for ii = 1:size(controller{k}.alpha,1)
          for jj = 1:size(controller{k}.alpha,2)

              cur_alpha_time = [];
              
              for kk = 1:size(alphas_time(k,:),2)

                 if(~isnan(alphas_time{k,kk}))
                     cur_alpha_time(:,kk) = alphas_time{k,kk}(:,ii*jj); 
                 end

              end

              i=1;
              for t = time

                  cur_alpha_mean_time(i) = mean(cur_alpha_time(i,:));
                  cur_alpha_var_time(i)  = std(cur_alpha_time(i,:));

                  i=i+1;
              end
              figure
            
              h = shadedErrorBar(time,cur_alpha_mean_time,cur_alpha_var_time,{'r','LineWidth',3});
              xlabel('t','FontSize',16);
              ylabel(strcat('\alpha_{',num2str(jj),'}'),'FontSize',16);
              ylim([0 1])
              names_legend = { 'mean','std deviation'};
              handle_legend = [h.mainLine,h.patch];
              h_legend = legend(handle_legend,names_legend);
              set(h_legend,'FontSize',15);
              set(gca,'Layer','top')
          end
      end
      cur_alpha_mean_time = [];
      cur_alpha_var_time  = [];
   end

   
   
   
   

end

% i need to compute directly the mean for each variable to avoid 
function [all_q,all_tau,all_ee,all_elbow]=ComputeTorqueAndCartesianJointPos(all_controller,time_sym_struct,qi,qdi,fixed_step,interp_step)
   
   RAD = 180/pi;

   for i=1:size(all_controller,1) 
      for j = 1:size(all_controller,2)
            if(isobject(all_controller{i,j}))
                  [t, q, qd] = DynSim(time_sym_struct{i},all_controller{i,j},qi(i,:),qdi(i,:),fixed_step(i));
                  all_q{i,j} = q{1}'*RAD;
                  % all_tau is a cell{i,j} of cell{i}
                  all_tau{i,j} = InterpTorque(all_controller{i,j},time_sym_struct{i},interp_step);
                  %all_tau{i,j} = all_controller{i,j}.torques{1};
                  [ee,elbow]=ComputePositions(q{1},t,all_controller{i,j});
                  all_ee{i,j} = ee;
                  all_elbow{i,j} = elbow;   
            end
      end
   end


end

function [q,tau,ee,elbow]=ComputeTorqueAndCartesianJointPosSingleControllerUknownSolution(controller,time_sym_struct,qi,qdi,fixed_step,torque_interp_step)

   if(isobject(controller))
         [t, q, qd] = DynSim(time_sym_struct,controller,qi,qdi,fixed_step);
         tau = InterpTorque(controller,time_sym_struct,torque_interp_step);
         [ee,elbow]=ComputePositions(q{1},t,controller);
   end

end


function [q,tau,ee,elbow]=ComputeTorqueAndCartesianJointPosSingleControllerKnownSolution(controller,q,t,time_sym_struct,torque_interp_step)

   
   if(isobject(controller))
         
         % all_tau is a cell{i,j} of cell{i}
         tau = InterpTorque(controller,time_sym_struct,torque_interp_step);
         [ee,elbow]=ComputePositions(q,t,controller);
        
   end
   
end


% i obtain the interpolated torque and in this way i can compute mean and
% variance
function torque=InterpTorque(controller,time_struct,interp_step)
   time = time_struct.ti:interp_step:time_struct.tf;
   if(size(controller.torques_time{1},2) == size(controller.torques{1},2))
      [unique_time,ia,ic] = unique(controller.torques_time{1});
   else
      diff = size(controller.torques_time{1},2) - size(controller.torques{1},2) + 1;
      controller.torques_time{1} = controller.torques_time{1}(diff:end);
      [unique_time,ia,ic] = unique(controller.torques_time{1});
   end
   torque = [];
   for i = 1:size(controller.torques{1},1)
      unique_torque = controller.torques{1}(i,ia);
      interp_torque = interp1(unique_time,unique_torque,time,'nearest');
      torque = [torque , interp_torque'];
   end 
end


function PlotJoints(all_q,list_time_struct)
   
   for i = 1:size(all_q,1)
      time = list_time_struct{i}.ti:list_time_struct{i}.step:list_time_struct{i}.tf;
      result_avg = zeros(size(all_q{i,1},2),size(all_q{i,1},1));
      result_std = zeros(size(all_q{i,1},2),size(all_q{i,1},1));
      for k = 1:size(all_q{i,1},2)
         cur_sample = [];
         for j = 1:size(all_q,2)
            
            cur_sample = [cur_sample ,all_q{i,j}(:,k)];
   
         end
         result_avg(k,:) = mean(cur_sample,2)';
         result_std(k,:) = std(cur_sample,0,2)';
         
      end
      
      %time = 1:size(result_avg,1);
      
      % plot each q
      for h=1:size(result_avg,2)
         figure
         hh = shadedErrorBar(time,result_avg(:,h),result_std(:,h),{'Color','r','LineWidth',3});
         xlabel('t','FontSize',16);
         ylabel(strcat('q_{',num2str(h),'}'),'FontSize',16);
         names_legend = { 'mean','std deviation'};
         handle_legend = [hh.mainLine,hh.patch];
         h_legend = legend(handle_legend,names_legend);
         set(h_legend,'FontSize',15);
         set(gca,'Layer','top')
      end
      
   end
end
% plot the average and the std deviation for each bunch
function PlotTorque(all_tau,list_time_struct,interp_step)

for i=1:size(all_tau,1)
   time = list_time_struct{i}.ti:interp_step:list_time_struct{i}.tf;
   % this is a cycle on the different torque joint
   for k = 1:size(all_tau{i,1},2)
   all_cur_value = zeros(size(time,2),size(all_tau,2));   
      for j = 1:size(all_tau,2)
         all_cur_value(:,j) = all_tau{i,j}(:,k);
      end
      cur_mean = mean(all_cur_value,2);
      cur_var  = std(all_cur_value,0,2);
      
     figure
      %cur_var = zeros(size(time,2),1);
      h = shadedErrorBar(time,cur_mean,cur_var,{'Color','b','LineWidth',3'});
      xlabel('t','FontSize',16);
      ylabel(strcat('\tau_{',num2str(k),'}'),'FontSize',16);
      names_legend = { 'mean','std deviation'};
      handle_legend = [h.mainLine,h.patch];
      h_legend = legend(handle_legend,names_legend);
      set(h_legend,'FontSize',15);
      set(gca,'Layer','top')
      
   end
end


end

function PlotTrajectory(all_position,scenes,list_time_struct,qi,bot)

    for i = 1:size(all_position,1)
         time = list_time_struct{i}.ti:list_time_struct{i}.step:list_time_struct{i}.tf;
         result_avg = zeros(size(all_position{i,1},2),size(all_position{i,1},1));
         result_std = zeros(size(all_position{i,1},2),size(all_position{i,1},1));
         for k = 1:size(all_position{i,1},2)
            cur_sample = [];
            for j = 1:size(all_position,2)

               cur_sample = [cur_sample ,all_position{i,j}(:,k)];

            end
            result_avg(k,:) = mean(cur_sample,2)';
            result_std(k,:) = std(cur_sample,0,2)';

         end
         
         figure
         bot{i}.plot(qi{1,1})
         % plot avg final trajectory
         text = LoadScenario(scenes{i});
         eval(text);
         plot3(result_avg(:,1),result_avg(:,2),result_avg(:,3),'Color','r','LineWidth',2);

         label={'x','y','z'};

         for ii =1:size(result_avg,2)
             figure
             shadedErrorBar(time,result_avg(:,ii),result_std(:,ii),{'Color','k','LineWidth',3});  
              xlabel('t','FontSize',16);
              ylabel(label{ii},'FontSize',16);
         end
    end
end

function PlotExpOne(list_of_data_to_plot,style)
      

      % i read all the data that i want to plot
      for iii = 1:size(list_of_data_to_plot,2)
      
         allpath=which('FindData.m');
         path=fileparts(allpath);
         result = strcat(path,'/results/','1_Atest1/',num2str(list_of_data_to_plot(iii)),'.mat');
         load(result);
         
         if iii == 1
            traj{1} = p_tot';
         end
         
         all_controller{iii} = controller;
         all_q{iii} = q{1};
      end
      
      % compute the cartesian position 
      for i = 1:size(list_of_data_to_plot,2)
         traj{i+1} = CompPosition(all_controller{i},all_q{i});   
      end
      
      name_of_methods = {'reference','RUF','UF'}; 
      linewidth = [2.5,2,2];
      for j = 1:size(traj,2)
         subplot(1,2,1);hold on
         plot(traj{j}(:,1),traj{j}(:,2),style{j},'LineWidth',linewidth(j))
         xlabel('x','FontSize',16);
         ylabel('y','FontSize',16);
         xlim([-0.2 1])
         hM = legend(name_of_methods);
         set(hM,'FontSize',15);
         
         subplot(1,2,2);hold on
         plot(traj{j}(:,1),traj{j}(:,3),style{j},'LineWidth',linewidth(j))
         xlabel('x','FontSize',16);
         ylabel('z','FontSize',16);
         hL = legend(name_of_methods);
         set(hL,'FontSize',15);
         
      end
%       newPosition = [0.1 0.5 0.1 0.1];
%       newUnits = 'normalized';
%       set(hL,'Position', newPosition,'Units', newUnits);

end





function pos = CompPosition(controller,q)

        
        bot = controller.GetActiveBot();
        pos = [];
        for i = 1 : size(q,1)
           % compute pose (position + rool pitch yaw) from the current
           % subchain
           T=bot.T0_7(q(i,:));
           pos =[ pos; T(1:3,4)'];
        end



end

function ComputeMeanVarianceTime(all_exec_time)

   mean_time_vector = [];
   std_time_vector = [];
   for i=1:size(all_exec_time,2)
      cur_vec_time = all_exec_time{1,i};
      mean_time_vector = [mean_time_vector;mean(cur_vec_time)];
      std_time_vector = [std_time_vector;std(cur_vec_time)];
   end
   
   assignin('base','mean_time_vector',mean_time_vector);
   assignin('base','std_time_vector',std_time_vector);

end

function CountSuccessRate(all_fitness,tresh)
   
   total_trial = zeros(size(all_fitness,2),1);
   success     = zeros(size(all_fitness,2),1);
   percentage_success = zeros(size(all_fitness,2),1);
   
   for i = 1:size(all_fitness,2)
      
      total_trial(i,1) = size(all_fitness{1,i},2);
      for j =1:total_trial(i,1)
         if(find(all_fitness{1,i}(:,j) >= tresh(1,i),1))
           success(i,1) = success(i,1) + 1; 
         end
      end
       percentage_success(i,1) = (success(i,1)/total_trial(i,1))*100;  
   end
   
   assignin('base','total_trial_success_rate',total_trial);
   assignin('base','success_rate',success);
   assignin('base','percentage_success_rate',percentage_success);
end



function PlotTorqueFromDat(controller)
   plot(controller.torques{1, 1}');
   xlabel('t','FontSize',16);
   ylabel(strcat('torque'),'FontSize',16);
   h_legend = legend({'J1','J2','J3','J4','J5','J6','J7'});
   set(h_legend,'FontSize',15);
   set(gca,'Layer','top');
end




function PlotPositionXYfromDat(list_of_folder,experiment_index,name_of_methods,style,interp_step)

   for i = 1:size(list_of_folder,2)
      folder_path = BuildMatFilePath(list_of_folder{i},experiment_index(i));
      [cur_ee,cur_elbow] = ComputeXYLink(folder_path,interp_step);
      all_ee{i} = cur_ee';
      all_elbow{i} = cur_elbow';
   end
   
   
   figure
   hold on;
   for j = 1:size(list_of_folder,2)
      plot(all_ee{j}(:,1),all_ee{j}(:,2),style{j},'LineWidth',3)
      xlabel('x','FontSize',16);
      ylabel('y','FontSize',16);
      xlim([-0.2 1])
   end
   hM = legend(name_of_methods);
   set(hM,'FontSize',15);
   
   figure
   hold on;
   for j = 1:size(list_of_folder,2)
      plot(all_elbow{j}(:,1),all_elbow{j}(:,2),style{j},'LineWidth',3)
      xlabel('x','FontSize',16);
      ylabel('y','FontSize',16);
      xlim([-0.2 1])
   end
   hM = legend(name_of_methods);
   set(hM,'FontSize',15);
   
   
end



function PlotPositionXYZfromDat(list_of_folder,experiment_index,name_of_methods,style,interp_step)

   for i = 1:size(list_of_folder,2)
      folder_path = BuildMatFilePath(list_of_folder{i},experiment_index(i));
      [cur_ee,cur_elbow,time_sym] = ComputeXYLink(folder_path,interp_step);
      all_ee{i} = cur_ee';
      all_elbow{i} = cur_elbow';
      time_struct{i} = time_sym;
   end
   
   
   figure
   hold on;
   for j = 1:size(list_of_folder,2)
      time = time_struct{j}.ti:time_struct{j}.step:time_struct{j}.tf;
      plot(time,all_ee{j}(:,1),style{j},'LineWidth',3)
      xlabel('t','FontSize',16);
      ylabel('x','FontSize',16);
   end
   hM = legend(name_of_methods);
   set(hM,'FontSize',15);
   
   
   figure
   hold on;
   for j = 1:size(list_of_folder,2)
      time = time_struct{j}.ti:time_struct{j}.step:time_struct{j}.tf;
      plot(time,all_ee{j}(:,2),style{j},'LineWidth',3)
      xlabel('t','FontSize',16);
      ylabel('y','FontSize',16);
   end
   hM = legend(name_of_methods);
   set(hM,'FontSize',15);
  
   figure
   hold on;
   for j = 1:size(list_of_folder,2)
      time = time_struct{j}.ti:time_struct{j}.step:time_struct{j}.tf;
      plot(time,all_ee{j}(:,3),style{j},'LineWidth',3)
      xlabel('t','FontSize',16);
      ylabel('z','FontSize',16);
      %xlim([-0.2 1])
   end
   hM = legend(name_of_methods);
   set(hM,'FontSize',15);
   
   
end


% with GHC from optimization with cmaes doesnt work
function [ee,elbow,time_sym_struct] = ComputeXYLink(path,torque_interp_step)
   q = [];
   load(path);
   if(~isempty(q))
      [~,~,ee,elbow]=ComputeTorqueAndCartesianJointPosSingleControllerKnownSolution(controller,q{1},t,time_sym_struct,torque_interp_step);
   else
      [~,~,ee,elbow]=ComputeTorqueAndCartesianJointPosSingleControllerUknownSolution(controller,time_sym_struct,qi,qdi,fixed_step,torque_interp_step);
   end
   
end