
function CMAEStest
%% test main for the cmaes optimizer
%% this file is conceived for testing one method with different test functions
clear variables
close all
clc

%% DATA 1
function_2_test ={'g06','g07','g09','f240','f241','HB'};%,'g06','g07','g09','f240','f241','HB'};
method_to_use = 'adaptive';  % adaptive , vanilla , empty
learn_approach = 'CMAES'; %CMAES (1+1)CMAES               with (1+1)CMAES i have to use vanilla constraints management  (temporary)
repetition_of_the_experiment = 50; % at least 2
threshold = 50; % value to identify the beginning of steady state
explorationRate = 0.1; %0.1; %0.5; %0.1;%[0, 1]
niter = 8000;  %number of generations
% starting value of parameters
generation_of_starting_point = 'test'; % 'test', 'given', 'random'


for jj=1:length(function_2_test)
    %% CONSTRAINTS PARAMETERS
    if(strcmp(function_2_test{jj},'g06'))
        search_space_dimension = 2;
        function_2_test_4_comparison = {'g06Test'};
        epsilon = [0.001, 0.001]; % for adaptive
        constraints_functions = {'g06Constr1','g06Constr2'}; 
        constraints_for_test = 'g06Constr';
        constraints_type = [1 1];      
        constraints_values =[0,0];
        n_constraints = length(constraints_functions);
        cmaes_value_range{1} = [13 , 0];  % lower bound that define the search space
        cmaes_value_range{2} = [100 , 100];  % upper bound that define the search space
        user_defined_start_action = [14.6111 2.1491]; 
    elseif(strcmp(function_2_test{jj},'tr2'))
         search_space_dimension = 2;
        function_2_test_4_comparison = {'tr2Test'};
        epsilon = [0.001]; % for adaptive
        constraints_functions = {'tr2Constr1'}; 
        constraints_for_test = 'tr2Constr';
        constraints_type = [1];      
        constraints_values =[0];
        n_constraints = length(constraints_functions);
        cmaes_value_range{1} = [0 , 0];  % lower bound that define the search space
        cmaes_value_range{2} = [100 , 100];  % upper bound that define the search space
        user_defined_start_action = [50,50];
    elseif(strcmp(function_2_test{jj},'g07'))
        search_space_dimension = 10;
        function_2_test_4_comparison = {'g07Test'};
        epsilon = [0.001 0.001 0.001 0.001 0.001 0.001 0.001 0.001]; % for adaptive
        constraints_functions = {'g07Constr1','g07Constr2','g07Constr3','g07Constr4','g07Constr5','g07Constr6','g07Constr7','g07Constr8'}; 
        constraints_for_test = 'g07Constr';
        constraints_type = [1 1 1 1 1 1 1 1];      
        constraints_values =[0 0 0 0 0 0 0 0];
        n_constraints = length(constraints_functions);
        cmaes_value_range{1} = [-10 , -10, -10 ,-10, -10, -10,-10,-10,-10 ,-10 ];  % lower bound that define the search space
        cmaes_value_range{2} = [ 10, 10, 10, 10, 10, 10, 10, 10, 10 ,10];  % upper bound that define the search space
        user_defined_start_action = [2.22222222222222,3.70370370370370,6.66666666666666,8.88888888888889,0,6.66666666666666,0,0,6.66666666666666,6.66666666666666];
    elseif(strcmp(function_2_test{jj},'g09'))
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
        user_defined_start_action = [0,0,0,0,0,0,0];
    elseif(strcmp(function_2_test{jj},'g10')) % hard to find a feasible starting point
        search_space_dimension = 8;
        function_2_test_4_comparison = {'g10Test'};
        epsilon = [0.001 0.001 0.001 0.001 0.001 0.001]; % for adaptive
        constraints_functions = {'g10Constr1','g10Constr2','g10Constr3','g10Constr4','g10Constr5','g10Constr6'}; 
        constraints_for_test = 'g10Constr';
        constraints_type = [1 1 1 1 1 1];      
        constraints_values =[0 0 0 0 0 0];
        n_constraints = length(constraints_functions);
        cmaes_value_range{1} = [100 , 1000, 1000 ,10, 10, 10,10,10 ];  % lower bound that define the search space
        cmaes_value_range{2} = [ 1000, 10000, 10000, 1000, 1000, 1000, 1000, 1000];  % upper bound that define the search space
        %user_defined_start_action =
    elseif(strcmp(function_2_test{jj},'f240'))
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
        user_defined_start_action = [555.555555555556,1666.66666666667,555.555555555556,555.555555555556,555.555555555556];
    elseif(strcmp(function_2_test{jj},'f241'))
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
        user_defined_start_action =[1851.85185185185,617.283950617287,617.283950617287,617.283950617287,617.283950617287];
    elseif(strcmp(function_2_test{jj},'HB'))
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
        user_defined_start_action = [90,35,36,36,36];
    end

    %% INSTANCE PARAMETER
    run_function = @EmptyPreprocessing;
    fitness = str2func(function_2_test{jj});
    clean_function = @EmptyPostprocessing;

    %% CMAES PARAMETER
    
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
    if(strcmp(method_to_use,'vanilla'))
        constr =Optimization.FixPenalty(search_space_dimension,constraints_functions,constraints_type,constraints_values);
    elseif(strcmp(method_to_use,'adaptive'))
        constr = Optimization.AdaptivePenalty(epsilon,niter,search_space_dimension,constraints_functions,constraints_type,constraints_values);
    elseif(strcmp(method_to_use,'empty'))
        constr = [];
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
         inst =  Optimization.Instance(constr,learn_approach,run_function,fitness,clean_function,[]);
         tic
         [mean_performances, bestAction, BestActionPerEachGen, policies, costs, succeeded, data2save] = inst.CMAES(search_space_dimension,start_action,niter,explorationRate,cmaes_value_range);
         %% collect all the data from each experiments
         % execution time
         exec_time(kk) = toc;
         % perfomance 
         all_mean_perfomance(kk,:) = mean_performances';
         begin_of_steady_state(kk) = IndetifySteadyState(all_mean_perfomance(kk,:),threshold);
         % perfomance without correction
         all_mean_perfomance_without_correction(kk,:) = data2save.performance';
         if(strcmp(method_to_use,'adaptive'))
            all_weights{kk} = inst.penalty_handling.weights;
         end
         % best results
         all_best_action(kk,:) = bestAction.parameters;
         distance_from_best_action(kk) = norm(bestAction.parameters - benchmark_x);
         all_best(kk,1) = bestAction.performance;
         % all best last result
         if(~isempty(policies))
            last_cost = costs(end-last_generation:end);
            last_policies = policies(end-last_generation:end,:);
            [tmp , id]=min(last_cost);
            all_last_best(kk,1) = -last_cost(id);
            all_best_last_action(kk,:) = last_policies(id,:);
            distance_from_last_best_action(kk) = norm(last_policies(id,:) - benchmark_x);
         end
         % violations
         for ii=1:n_constraints
             all_best_violations(kk,ii) = feval(constraints_functions{ii},bestAction.parameters);
             if(~isempty(policies))
               all_last_best_violations(kk,ii) = feval(constraints_functions{ii},last_policies(id,:));
             end
         end
         
         close all;
    end
    
    
    %% data collected at the end of each test function
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
    
    % compute mean and variance of the distance from optimal solutions
    G_distance_from_best_action(jj,:) = distance_from_best_action;
    G_mean_distance_from_best_action(jj) = mean(distance_from_best_action);
    G_variance_distance_from_best_action(jj) = var(distance_from_best_action);
    if(~isempty(policies))
      G_distance_from_last_best_action(jj,:) = distance_from_last_best_action;
      G_mean_distance_from_last_best_action(jj) = mean(distance_from_last_best_action);
      G_var_distance_from_last_best_action(jj) = var(distance_from_last_best_action);
    end
    % compute the sum of violations per each test function (i just sum the effective violations)
    mask = all_best_violations > 0;
    all_best_violations = all_best_violations.*mask;
    
    mask = all_last_best_violations > 0;
    all_last_best_violations = all_last_best_violations.*mask;
    
    % violations 
%     for ii=1:n_constraints
%          G_best_violations.mean(1,ii) = mean(all_best_violations(:,ii),1); 
%          G_best_violations.variance(1,ii) = var(all_best_violations(:,ii)); 
%          G_last_best_violations.mean(1,ii) = mean(all_last_best_violations(:,ii),1); 
%          G_last_best_violations.variance(1,ii) = var(all_last_best_violations(:,ii)); 
%     end
     
    all_G_best_violations(:,jj) = sum(all_best_violations,2);
    all_G_last_best_violations(:,jj) = sum(all_last_best_violations,2);
    
    % all steady state
    G_steady_state(jj,:)=begin_of_steady_state;
    
    %if(strcmp(method_to_use,'adaptive'))
    %  weight_pure.average  = mean(all_weights,1);
    %  weight_pure.variance = var(all_weights);
    %  all_weight_pure{jj} = weight_pure;   
    %end      
    
    % clean variable
    all_best_action = [];
    all_best_last_action =[];
 end
%%  SAVE PATH
 allpath=which('FindData.m');
 local_path=fileparts(allpath);
 local_path = strcat(local_path,'/benckmark');
 name_folder = strcat(learn_approach,'-',method_to_use);
 mkdir(local_path,name_folder);
 local_path = strcat(local_path,'/',name_folder);
 
 
%% PLOT
number_of_plot_of_single_experiments = 0;
if(number_of_plot_of_single_experiments > repetition_of_the_experiment)
   number_of_plot_of_single_experiments = repetition_of_the_experiment;
end
color_list={'b','r','m','g','c','k','y'};
transparent_flag = 1;
%handle_legend = [];

if(exist('all_prf_pure','var'))   
   % plot average fitness without correction
   generation = 1:length(all_prf_pure{1}.average);
   h1 = figure;
   hold on;
   for z = 1:length(function_2_test)
      plot(generation',all_prf_pure{z}.average','Color',color_list{z});
      xlabel('generations','FontSize',16);
      ylabel('fitness','FontSize',16);
      text_title =  sprintf('average fitness without correction');
      title(text_title,'FontSize',20);
   end
   name_fig = strcat(local_path,'/','prf_pure_mean');
   saveas(h1,name_fig);
   % plot average and variance of fitness without correction
   h2=figure;
   hold on;
   for z = 1:length(function_2_test)
      h = shadedErrorBar(generation',all_prf_pure{z}.average,all_prf_pure{z}.variance,{'r-o','Color',color_list{z},'markerfacecolor',color_list{z}},transparent_flag);
      %handle_legend = [handle_legend,h.mainLine];
      xlabel('generations','FontSize',16);
      ylabel('fitness','FontSize',16);
      %h_legend = legend(handle_legend,method_to_use{z});
      %set(h_legend,'FontSize',15);
       text_title =  sprintf('average and variance fitness without correction');
      title(text_title,'FontSize',20);
   end
   name_fig = strcat(local_path,'/','prf_pure_mean_var');
   saveas(h2,name_fig);
end
% plot average fitness with correction
generation = 1:length(all_prf{1}.average);
h3=figure;
hold on;
for z = 1:length(function_2_test)
   plot(generation',all_prf{z}.average','Color',color_list{z});
   xlabel('generations','FontSize',16);
   ylabel('fitness','FontSize',16);
   text_title =  sprintf('average fitness with correction');
   title(text_title,'FontSize',20);
end
name_fig = strcat(local_path,'/','prf_mean');
saveas(h3,name_fig);
% plot average and variance of fitness with correction
h4 = figure;
hold on;
for z = 1:length(function_2_test)
   h = shadedErrorBar(generation',all_prf{z}.average,all_prf{z}.variance,{'r-o','Color',color_list{z},'markerfacecolor',color_list{z}},transparent_flag);
   %handle_legend = [handle_legend,h.mainLine];
   xlabel('generations','FontSize',16);
   ylabel('fitness','FontSize',16);
   %h_legend = legend(handle_legend,method_to_use{z});
   %set(h_legend,'FontSize',15);
   text_title =  sprintf('average and variance fitness with correction');
   title(text_title,'FontSize',20);
end
name_fig = strcat(local_path,'/','prf_mean_var');
saveas(h4,name_fig);
% plotbox plot with the distance from the true policy
h5=figure;
boxplot(G_distance_from_best_action',function_2_test)
text_title =  'policy error';
title(text_title,'FontSize',20);
name_fig = strcat(local_path,'/','policy_error');
saveas(h5,name_fig);
if(~isempty(policies))
   h6=figure;
   boxplot(G_distance_from_last_best_action',function_2_test)
   text_title =  'last policy error';
   title(text_title,'FontSize',20);
   name_fig = strcat(local_path,'/','last_policy_error');
   saveas(h6,name_fig);
end
%plotbox plot of the sum of constraints violations
h5_1=figure;
boxplot(all_G_best_violations,function_2_test)
text_title =  'constraints violations';
title(text_title,'FontSize',20);
name_fig = strcat(local_path,'/','constraints_violations');
saveas(h5_1,name_fig);
if(~isempty(policies))
   h6_1=figure;
   boxplot(all_G_last_best_violations,function_2_test)
   text_title =  'last constraints violations';
   title(text_title,'FontSize',20);
   name_fig = strcat(local_path,'/','last_constraints_violations');
   saveas(h6_1,name_fig);
end

% plotbox plot with the steady state time
h7=figure;
boxplot(G_steady_state',function_2_test)
text_title =  'generation to reach the steady value';
name_fig = strcat(local_path,'/','generation_to_steady');
saveas(h7,name_fig);
title(text_title,'FontSize',20);
% plot # number_of_plot_of_single_execution of fitness
% for z = 1:length(function_2_test)
%    for zz = 1:number_of_plot_of_single_experiments
%       figure
%       plot(generation',all_perfomance_with_constraint_correction{z}(zz,:)');
%       xlabel('generations','FontSize',16);
%       ylabel('fitness','FontSize',16);
%       text_title =  sprintf('experiment %d', zz);
%       title(text_title,'FontSize',20);
%    end
% end
% if(strcmp(method_to_use,'adaptive'))
%    % plot # number_of_plot_of_single_execution weight
%    for zz = 1:number_of_plot_of_single_experiments
%       figure
%       text_title =  sprintf('experiment %d', zz);
%       title(text_title);
%       plot(generation',all_weights{zz});
%       xlabel('generations','FontSize',16);
%       ylabel('weight values','FontSize',16);
%       legend('weight 1','weight 2','weight 3');
%       text_title =  sprintf('experiment %d', zz);
%       title(text_title,'FontSize',20);
%    end
%    
% end
% save workspace
name_file = strcat(local_path,'/','dat.mat');
save(name_file);

end


function zzz = IndetifySteadyState(vector,tresh)
   steady_value = vector(end);
   for zzz = 1:length(vector)
      if(abs(steady_value-vector(zzz))<tresh)
         break
      end
   end
   
end
