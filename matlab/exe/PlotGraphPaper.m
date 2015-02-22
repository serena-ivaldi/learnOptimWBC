% with this file is possible to draw graph about method robustness 
% to compare methods and to establish if the activation policies have to many
% basis functions(overfitting).
% TODO add time plot!. i have to change data.mat in the optimization loop
function PlotGraphPaper
   
   close all 
   clear variable 
   clc
   
   % if i give more than one result folder i will merge the result all
   % togheter
   list_of_folder = {'_of_106_sere/LBR4p5.0_scene5_UF_repellers_on_elbow__atrtactive_point_on_ee_fit5_SERE'};
   % name of the method that will be displayed in the legenda of graph
   name_of_methods = {'UF','UF'};
   color_list={'b','r','g'};
   variance_flag = true;
   alpha_flag =true;
   position_joint_torque_flag = false;
   % with this variable i control for each folder how many sample i
   % consider for the plot of the position e-e elbow and for the joint
   %it is necessary to have one increment for each folder 
   increment = [0.1 0.001];
   % interpolation step for the tau if we use a small step wi will badly
   % capture the value of the applied torque
   interpolation_step = 0.01;
   
   
   for i=1:size(list_of_folder,2)
      % here i take only the first because i make the hypotesis that the
      % data necessary here for each series of experiment are all similar
      % to the first of each series
      cur_folder_path = BuildMatFilePath(list_of_folder{i},1);
      [number_of_iteration,controller,time_struct,cur_qi,cur_qdi,cur_fixed_step,cur_scene,bot] = GetAdditionalData(cur_folder_path);
      list_number_of_iteration(i) = number_of_iteration;
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
    
   for i=1:size(list_of_folder,2)
 
      for j=1:list_number_of_iteration(i)
         cur_folder_path = BuildMatFilePath(list_of_folder{i},j);
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

   PlotFitness(all_fitness,variance_flag,name_of_methods,color_list);
   
   if(alpha_flag)
      PlotAlpha(all_alpha,controller_first_iteration,list_time_struct);
   end
   
   
   if(position_joint_torque_flag)
      tic
      [all_q,all_tau,all_ee,all_elbow]=ComputeTorqueAndCartesianJointPos(all_controller,list_time_struct,qi,qdi,fixed_step,increment,interpolation_step);
      toc
      PlotJoints(all_q,list_time_struct);
      PlotTorque(all_tau,list_time_struct,interpolation_step);
      PlotTrajectory(all_ee,all_scene,list_time_struct,qi,all_bot);
      PlotTrajectory(all_elbow,all_scene,list_time_struct,qi,all_bot)
   end
   


end


% this function give back the path to .mat file of the current experiment 
function result=BuildMatFilePath(folder_name,experiment_index)

   allpath=which('FindData.m');
   path=fileparts(allpath);
   result = strcat(path,'/results/',num2str(experiment_index),folder_name,'/data.mat');

end

% eith this function i collect all the data that are necessary for the
% subsequent method. the hypothesis is that the firsrt experiment share the
% same data with the others 
function [n_of_iter, control ,t_struct,qinit,qdinit,cur_fixed_step,scene,bot] = GetAdditionalData(cur_mat_path)

   load(cur_mat_path);
    
   n_of_iter = number_of_iteration;
   t_struct = time_struct;
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
   
   
   cur_alpha = bestAction.parameters';
   
   cur_time = exec_time;
   % i have to clean this variable because in fdyn he expect that it is
   % empty
   controller.current_time = [];
   % i have to provide the controller with this new field that i add after
   % taking data
   controller.torques_time = cell(controller.subchains.GetNumChains());
   for i = 1 :controller.subchains.GetNumChains()
      controller.torques_time{i} = [];
   end
   cur_controller = controller;

end



% with this function i plot the fitness for each exeperiment and if i
% define more folder i merge the result in one graph
function PlotFitness(all_fitness,variance_flag,name_of_methods,color_list)
hold on;

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
         shadedErrorBar(generation',fit_mean,fit_var,{'r-o','Color',color_list{i},'markerfacecolor',color_list{i}});
         xlabel('generations','FontSize',16);
         ylabel('fitness','FontSize',16);
         legend(name_of_methods)
      else
         % remove all the not a number. it can happens if in one generation
         % i have a failure in each experiment and in that case mean give
         % back a NaN
         fit_mean(isnan(fit_mean)) = [];
         generation = 1:size(fit_mean,1);
         plot(generation',fit_mean,'r-o','Color',color_list{i},'markerfacecolor',color_list{i})
         xlabel('generations','FontSize',16);
         ylabel('fitness','FontSize',16);
         legend(name_of_methods)
      end
      
   end
   
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
   end
   
   for k=1:size(alphas_time,1)
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
              shadedErrorBar(time,cur_alpha_mean_time,cur_alpha_var_time,{'r-o','Color','r','markerfacecolor','r'});
              xlabel('t','FontSize',16);
              ylabel(strcat('\alpha_{',num2str(ii),num2str(jj),'}'),'FontSize',16);
              ylim([0 1])

          end
      end
   end

   
   
   
   

end

% i need to compute direcctly the mean for each variable to avoid 
function [all_q,all_tau,all_ee,all_elbow]=ComputeTorqueAndCartesianJointPos(all_controller,time_sym_struct,qi,qdi,fixed_step,increment,interp_step)

for i=1:size(all_controller,1)
   % in this way i reduce the number of sample changing the step for the
   % current time_sym_struct
   time_sym_struct{i}.step = increment(1,i); 
   for j = 1:size(all_controller,2)
         if(isobject(all_controller{i,j}))
               [t, q, qd] = DynSim(time_sym_struct{i},all_controller{i,j},qi(i,:),qdi(i,:),fixed_step(i));
               all_q{i,j} = q{1}';
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


function [ee,elbow]=ComputePositions(q,t,controller)
   ee = [];
   elbow = [];
   
   for i=1:size(t,2)
      q_cur = q(i,:);
      % compute the trajectory error (absolute error)
      kinematic=CStrCatStr({'controller.subchains.sub_chains{1}.T0_'},num2str(controller.subchains.GetNumSubLinks(1,1)),{'(q_cur)'});
      T = eval(kinematic{1});
      ee = [ee , T(1:3,4)];
      kinematic=CStrCatStr({'controller.subchains.sub_chains{1}.T0_'},'3',{'(q_cur)'});
      T = eval(kinematic{1});
      elbow = [elbow, T(1:3,4)];
   end
end

% i obtain the interpolated torque and in this way i can compute mean and
% variance
function torque=InterpTorque(controller,time_struct,interp_step)
   time = time_struct.ti:interp_step:time_struct.tf;
   [unique_time,ia,ic] = unique(controller.torques_time{1});
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
         shadedErrorBar(time,result_avg(:,h),result_std(:,h),{'r-o','Color','r','markerfacecolor','r'});
         xlabel('t','FontSize',16);
         ylabel(strcat('q_{',num2str(h),'}'),'FontSize',16);
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
      shadedErrorBar(time,cur_mean,cur_var,{'b-o','Color','b','markerfacecolor','b'});
      xlabel('t','FontSize',16);
      ylabel(strcat('\tau_{',num2str(k),'}'),'FontSize',16);
      
      
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
      plot3(result_avg(:,1),result_avg(:,2),result_avg(:,3),'Color','r');
      
      label={'x','y','z'};
      
      for ii =1:size(result_avg,2)
          figure
          shadedErrorBar(time,result_avg(:,ii),result_std(:,ii),{'k-o','Color','k','markerfacecolor','k'});  
           xlabel('t','FontSize',16);
           ylabel(label{ii},'FontSize',16);
      end
      
      
 end




end


