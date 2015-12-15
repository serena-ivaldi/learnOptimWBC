%% test main for the cmaes optimizer
%% this file is conceived to test a single function with multiple algorithm
clear variables
close all
clc

%% DATA 1
function_2_test = 'tr2';
method_to_use = {'vanilla'};  % adaptive , vanilla , empty
repetition_of_the_experiment = 2; % at least 2

 %% CONSTRAINTS PARAMETERS
 if(strcmp(function_2_test,'g06'))
     search_space_dimension = 2;
     function_2_test_4_comparison = {'g06Test'};
     epsilon = [1, 1]; % for adaptive
     constraints_functions = {'g06Constr1','g06Constr2'}; 
     constraints_for_test = 'g06Constr';
     constraints_type = [1 1];      
     constraints_values =[0,0];
     n_constraints = length(constraints_functions);
     cmaes_value_range{1} = [13 , 0];  % lower bound that define the search space
     cmaes_value_range{2} = [100 , 100];  % upper bound that define the search space
 elseif(strcmp(function_2_test,'tr2'))
      search_space_dimension = 2;
     function_2_test_4_comparison = {'tr2Test'};
     epsilon = [1]; % for adaptive
     constraints_functions = {'tr2Constr1'}; 
     constraints_for_test = 'tr2Constr';
     constraints_type = [1];      
     constraints_values =[0];
     n_constraints = length(constraints_functions);
     cmaes_value_range{1} = [0 , 0];  % lower bound that define the search space
     cmaes_value_range{2} = [100 , 100];  % upper bound that define the search space
 elseif(strcmp(function_2_test,'g07'))
     search_space_dimension = 10;
     function_2_test_4_comparison = {'g07Test'};
     epsilon = [1 1 1 1 1 1 1 1]; % for adaptive
     constraints_functions = {'g07Constr1','g07Constr2','g07Constr3','g07Constr4','g07Constr5','g07Constr6','g07Constr7','g07Constr8'}; 
     constraints_for_test = 'g07Constr';
     constraints_type = [1 1 1 1 1 1 1 1];      
     constraints_values =[0 0 0 0 0 0 0 0];
     n_constraints = length(constraints_functions);
     cmaes_value_range{1} = [-10 , -10, -10 ,-10, -10, -10,-10,-10,-10 ,-10 ];  % lower bound that define the search space
     cmaes_value_range{2} = [ 10, 10, 10, 10, 10, 10, 10, 10, 10 ,10];  % upper bound that define the search space
 elseif(strcmp(function_2_test,'g09'))
     search_space_dimension = 7;
     function_2_test_4_comparison = {'g09Test'};
     epsilon = [1 1 1 1]; % for adaptive
     constraints_functions = {'g09Constr1','g09Constr2','g09Constr3','g09Constr4'}; 
     constraints_for_test = 'g09Constr';
     constraints_type = [1 1 1 1];      
     constraints_values =[0 0 0 0];
     n_constraints = length(constraints_functions);
     cmaes_value_range{1} = [-10 , -10, -10 ,-10, -10, -10,-10 ];  % lower bound that define the search space
     cmaes_value_range{2} = [ 10, 10, 10, 10, 10, 10, 10];  % upper bound that define the search space
 elseif(strcmp(function_2_test,'g10')) % hard to find a feasible starting point
     search_space_dimension = 8;
     function_2_test_4_comparison = {'g10Test'};
     epsilon = [1 1 1 1 1 1]; % for adaptive
     constraints_functions = {'g10Constr1','g10Constr2','g10Constr3','g10Constr4','g10Constr5','g10Constr6'}; 
     constraints_for_test = 'g10Constr';
     constraints_type = [1 1 1 1 1 1];      
     constraints_values =[0 0 0 0 0 0];
     n_constraints = length(constraints_functions);
     cmaes_value_range{1} = [100 , 1000, 1000 ,10, 10, 10,10,10 ];  % lower bound that define the search space
     cmaes_value_range{2} = [ 1000, 10000, 10000, 1000, 1000, 1000, 1000, 1000];  % upper bound that define the search space
 elseif(strcmp(function_2_test,'f240'))
     search_space_dimension = 5;
     function_2_test_4_comparison = {'f240Test'};
     epsilon = [1]; % for adaptive
     constraints_functions = {'f240Constr1'}; 
     constraints_for_test = 'f240Constr';
     constraints_type = [1];      
     constraints_values =[0];
     n_constraints = length(constraints_functions);
     cmaes_value_range{1} = [ 0, 0, 0, 0, 0];  % lower bound that define the search space
     cmaes_value_range{2} = [10000 , 10000, 10000 ,10000, 10000 ];  % upper bound that define the search space
 elseif(strcmp(function_2_test,'f241'))
     search_space_dimension = 5;
     function_2_test_4_comparison = {'f241Test'};
     epsilon = [1]; % for adaptive
     constraints_functions = {'f240Constr1'}; 
     constraints_for_test = 'f240Constr';
     constraints_type = [1];      
     constraints_values =[0];
     n_constraints = length(constraints_functions);
     cmaes_value_range{1} = [ 0, 0, 0, 0, 0];  % lower bound that define the search space
     cmaes_value_range{2} = [100000 , 100000, 100000 ,100000, 100000 ];  % upper bound that define the search space
 elseif(strcmp(function_2_test,'HB'))
     search_space_dimension = 5;
     function_2_test_4_comparison = {'HBTest'};
     epsilon = [1 1 1 1 1 1]; % for adaptive
     constraints_functions = {'HBConstr1','HBConstr2','HBConstr3','HBConstr4','HBConstr5','HBConstr6'}; 
     constraints_for_test = 'HBConstr';
     constraints_type = [1 1 1 1 1 1];      
     constraints_values =[0 0 0 0 0 0];
     n_constraints = length(constraints_functions);
     cmaes_value_range{1} = [ 78 , 33, 27 ,27, 27 ];  % lower bound that define the search space
     cmaes_value_range{2} = [ 102, 45, 45, 45, 45];  % upper bound that define the search space
 end
 
 %% INSTANCE PARAMETER
 run_function = @EmptyPreprocessing;
 fitness = str2func(function_2_test);
 clean_function = @EmptyPostprocessing;
 
 %% CMAES PARAMETER
 explorationRate = 0.1; %0.1; %0.5; %0.1;%[0, 1]
 niter = 8000;  %number of generations
 % starting value of parameters
 generation_of_starting_point = 'random'; % 'test', 'given', 'random'
 switch generation_of_starting_point
        case 'test'
           start_action = user_defined_start_action;
        case 'given'
            start_action = init_parameters_from_out*ones(1,search_space_dimension);
        case 'random'
           if(isvec(cmaes_value_range))
               start_action = (cmaes_value_range(1,2)-cmaes_value_range(1,1)).*rand(1,search_space_dimension) + cmaes_value_range(1,1)*ones(1,search_space_dimension);
           elseif(iscell(cmaes_value_range))
               start_action = (cmaes_value_range{2}-cmaes_value_range{1}).*rand(1,search_space_dimension) + cmaes_value_range{1};
           end
 end

%% OPTIMIZATION WITH DERIVATIVE METHOD FOR BENCHMARKING
options = optimoptions(@fmincon,'Algorithm','sqp');
x0 = start_action;
[benchmark_x,benchmark_fval] = fmincon(str2func(function_2_test_4_comparison{1}),x0,[],[],[],[],cmaes_value_range{1},cmaes_value_range{2},str2func(constraints_for_test),options); 
 
 %% CONSTRAINTS
 
 for i=1:length(method_to_use) 
     if(strcmp(method_to_use{i},'vanilla'))
        constr{i}=Optimization.FixPenalty(search_space_dimension,constraints_functions,constraints_type,constraints_values);
     elseif(strcmp(method_to_use{i},'adaptive'))
        constr{i} = Optimization.AdaptivePenalty(epsilon,niter,search_space_dimension,constraints_functions,constraints_type,constraints_values);
     elseif(strcmp(method_to_use{i},'empty'))
        constr{i} = [];
     end
 end
 %% DATA 2
 lambda = round(4 + 3 * log(search_space_dimension)); % number of candidates used in cmaes
 last_generation = 100; % analyze the last n = last_generation to look for the best (hopefully in steady state)
 if(last_generation > niter)
    last_generation = round(niter/10);
 end
 
 %all_perfomance = zeros(repetition_of_the_experiment,niter*lambda);
 all_best = zeros(repetition_of_the_experiment,1);
 all_last_best = zeros(repetition_of_the_experiment,1);
 all_best_violations = zeros(repetition_of_the_experiment,n_constraints);
 all_last_best_violations = zeros(repetition_of_the_experiment,n_constraints);
 
 %% OPTIMIZATION
 for jj=1:length(method_to_use)
     for kk = 1:repetition_of_the_experiment
         switch generation_of_starting_point
         case 'test'
           start_action = user_defined_start_action;
         case 'given'
            start_action = init_parameters_from_out*ones(1,search_space_dimension);
         case 'random'
           if(isvec(cmaes_value_range))
               start_action = (cmaes_value_range(1,2)-cmaes_value_range(1,1)).*rand(1,search_space_dimension) + cmaes_value_range(1,1)*ones(1,search_space_dimension);
           elseif(iscell(cmaes_value_range))
               start_action = (cmaes_value_range{2}-cmaes_value_range{1}).*rand(1,search_space_dimension) + cmaes_value_range{1};
           end 
         end
         inst =  Optimization.Instance(constr{jj},run_function,fitness,clean_function,[]);
         tic
         [mean_performances, bestAction, BestActionPerEachGen, policies, costs, succeeded, data2save] = inst.CMAES(search_space_dimension,start_action,niter,explorationRate,cmaes_value_range);
         exec_time = toc
         all_covariance{kk} = data2save.C;
         all_mean_perfomance(kk,:) = mean_performances';
         all_mean_perfomance_without_correction(kk,:) = data2save.performance';
         if(strcmp(method_to_use{jj},'adaptive'))
            all_weights{kk} = inst.penalty_handling.weights;
         end
         all_best_action(kk,:) = bestAction.parameters;
         all_best(kk,1) = bestAction.performance;
         last_cost = costs(end-last_generation:end);
         if(~isempty(policies))
            last_policies = policies(end-last_generation:end,:);
            [tmp , id]=min(last_cost);
            all_last_best(kk,1) = -costs(id);
            all_best_last_action(kk,:) = last_policies(id,:);
         end
         for ii=1:n_constraints
             all_best_violations(kk,ii) = feval(constraints_functions{ii},bestAction.parameters);
             if(~isempty(policies))
               all_last_best_violations(kk,ii) = feval(constraints_functions{ii},last_policies(id,:));
             end
         end
         close all;
     end
     
     if(length(data2save.performance)>1)
        all_perfomance_without_constraint_correction{jj} = all_mean_perfomance_without_correction;
     end
     all_perfomance_with_constraint_correction{jj} = all_mean_perfomance;
    
     
     % compute mean and variance of perfomance without correction
     if(length(data2save.performance)>1)
        prf_pure.average  = mean(all_mean_perfomance_without_correction,1);
        prf_pure.variance = var(all_mean_perfomance_without_correction);
        all_prf_pure{jj} = prf_pure;
     end
     % compute the mean and variance of overall perfomance with constraints
     % correction
     prf.average  = mean(all_mean_perfomance,1);
     prf.variance = var(all_mean_perfomance);
     all_prf{jj} = prf;
     % mean and variance of the best perfomance over all the experiments
     G_best(jj,1) = mean(all_best,1);
     G_best(jj,2) = var(all_best);
     % mean and variance of the best perfomance of the last k generations
     % over all the experiments
     G_last_best(jj,1) = mean(all_last_best,1);
     G_last_best(jj,2) = var(all_last_best);
     
     for ii=1:n_constraints
         G_best_violations.mean(1,ii) = mean(all_best_violations(:,ii),1); 
         G_best_violations.variance(1,ii) = var(all_best_violations(:,ii)); 
         G_last_best_violations.mean(1,ii) = mean(all_last_best_violations(:,ii),1); 
         G_last_best_violations.variance(1,ii) = var(all_last_best_violations(:,ii)); 
     end
     
     all_G_best_violations{jj} = G_best_violations;
     all_G__last_best_violations{jj} = G_last_best_violations;
          
 end
 
 
%% PLOT
number_of_plot_of_single_experiments = 10;
if(number_of_plot_of_single_experiments > repetition_of_the_experiment)
   number_of_plot_of_single_experiments = repetition_of_the_experiment;
end
color_list={'b','r','m','g','c','k'};
transparent_flag = 1;
handle_legend = [];

if(exist('all_prf_pure','var'))   
   % plot average fitness without correction
   generation = 1:length(all_prf_pure{1}.average);
   figure
   hold on;
   for z = 1:length(method_to_use)
      plot(generation',all_prf_pure{z}.average','Color',color_list{z});
      xlabel('generations','FontSize',16);
      ylabel('fitness','FontSize',16);
      text_title =  sprintf('average fitness without correction');
      title(text_title,'FontSize',20);
   end

   % plot average and variance of fitness without correction
   figure
   hold on;
   for z = 1:length(method_to_use)
      h = shadedErrorBar(generation',all_prf_pure{z}.average,all_prf_pure{z}.variance,{'r-o','Color',color_list{i},'markerfacecolor',color_list{i}},transparent_flag);
      %handle_legend = [handle_legend,h.mainLine];
      xlabel('generations','FontSize',16);
      ylabel('fitness','FontSize',16);
      %h_legend = legend(handle_legend,method_to_use{z});
      %set(h_legend,'FontSize',15);
       text_title =  sprintf('average and variance fitness without correction');
      title(text_title,'FontSize',20);
   end
end

% plot average fitness with correction
generation = 1:length(all_prf{1}.average);
figure
hold on;
for z = 1:length(method_to_use)
   plot(generation',all_prf{z}.average','Color',color_list{z});
   xlabel('generations','FontSize',16);
   ylabel('fitness','FontSize',16);
   text_title =  sprintf('average fitness with correction');
   title(text_title,'FontSize',20);
end

% plot average and variance of fitness with correction
figure
hold on;
for z = 1:length(method_to_use)
   h = shadedErrorBar(generation',all_prf{z}.average,all_prf{z}.variance,{'r-o','Color',color_list{i},'markerfacecolor',color_list{i}},transparent_flag);
   %handle_legend = [handle_legend,h.mainLine];
   xlabel('generations','FontSize',16);
   ylabel('fitness','FontSize',16);
   %h_legend = legend(handle_legend,method_to_use{z});
   %set(h_legend,'FontSize',15);
   text_title =  sprintf('average and variance fitness with correction');
   title(text_title,'FontSize',20);
end

% plot # number_of_plot_of_single_execution of fitness
for z = 1:length(method_to_use)
   for zz = 1:number_of_plot_of_single_experiments
      figure
      plot(generation',all_perfomance_with_constraint_correction{z}(zz,:)');
      xlabel('generations','FontSize',16);
      ylabel('fitness','FontSize',16);
      text_title =  sprintf('experiment %d', zz);
      title(text_title,'FontSize',20);
   end
end

if(strcmp(method_to_use{jj},'adaptive'))
   % plot # number_of_plot_of_single_execution weight
   for zz = 1:number_of_plot_of_single_experiments
      figure
      text_title =  sprintf('experiment %d', zz);
      title(text_title);
      plot(generation',all_weights{zz});
      xlabel('generations','FontSize',16);
      ylabel('weight values','FontSize',16);
      legend('weight 1','weight 2','weight 3');
      text_title =  sprintf('experiment %d', zz);
      title(text_title,'FontSize',20);
   end
end


 
 
 
 
