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
   list_of_folder = {'_of_12_sere/LBR4p5.0_scene5_UF_repellers_on_elbow__atrtactive_point_on_ee_fit5_SERE'};
   % name of the method that will be displayed in the legenda of graph
   name_of_methods = {'UF','UF'};
   color_list={'b','r','g'};
   alpha_flag =true;
   variance_flag = true;
   
   % for now i have to use a driver because when i compute value in the
   % last version i did not put the number of experiments in data.mat
   for i=1:size(list_of_folder,2)
      % here i take only the first because i make the hypotesis that the
      % data necessary here for each series of experiment are all similar
      % to the first of each series
      cur_folder_path = BuildMatFilePath(list_of_folder{i},1);
      [number_of_iteration,controller,time_struct] = GetAdditionalData(cur_folder_path);
      list_number_of_iteration(i) = number_of_iteration;
      list_controller{i}= controller;
      list_time_struct{i}= time_struct;
   end
    
    all_cur_fitness = [];
    all_cur_alpha   = [];
    
   for i=1:size(list_of_folder,2)

      for j=1:list_number_of_iteration(i)
         cur_folder_path = BuildMatFilePath(list_of_folder{i},j);
         [cur_fitness,cur_alpha,cur_time]=GetFitnessResultAndAlpha(cur_folder_path);
         all_cur_fitness = [all_cur_fitness, cur_fitness];
         all_cur_alpha  =  [all_cur_alpha cur_alpha];
         all_exec_cur_time(1,j) = cur_time;
      end

      all_fitness{i} = all_cur_fitness;
      all_alpha{i} = all_cur_alpha;
      all_exec_time{i} = all_exec_cur_time;
      all_cur_fitness = [];
      all_cur_alpha   = [];
   end

   PlotFitness(all_fitness,variance_flag,name_of_methods,color_list);
   
   if(alpha_flag)
      PlotAlpha(all_alpha,list_controller,list_time_struct);
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
function [n_of_iter, control ,t_struct] = GetAdditionalData(cur_mat_path)

   load(cur_mat_path);
   
   n_of_iter = number_of_iteration;
   t_struct = time_struct;
   control = controller;
end

% with this function i collect best fitness value for each generation and 
% the best alpha combination fo the current experiment
% and the time execution of the experiment
function [cur_fitness,cur_alpha,cur_time]=GetFitnessResultAndAlpha(cur_mat_path)

   load(cur_mat_path);
   
   for i = 1:size(bestAction.hist,2)
      cur_fitness(i,1) = bestAction.hist(1, i).performance;
   end
   
   
   cur_alpha = bestAction.parameters';
   
   %cur_time = exec_time;
   %DEBUG
   cur_time = 0;
   %---
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
         fit_var (j) = std(current_generation);
      end
      
      if(variance_flag)
         % remove all the not a number. it can happens if in one generation
         % i have a failure in each experiment and in that case mean give
         % back a NaN
         fit_mean(isnan(fit_mean)) = [];
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
         disp('yea')  
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







