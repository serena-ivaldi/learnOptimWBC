%% test main for the cmaes optimizer 
clear variables
close all
clc

%% DATA 1
function_2_test = {'g06'};
method_to_use = {'adaptive'};  % adaptive , vanilla , empty
search_space_dimension = 2;
repetition_of_the_experiment = 50; % at least 2

 %% CONSTRAINTS PARAMETERS
 if(strcmp(function_2_test{1},'g06'))
     epsilon = [1, 1, 1];
     constraints_functions = {'g06Constr1','g06Constr2','g06Constr3'}; 
     constraints_type = [1 1 1];      
     constraints_values =[0,0,0];
     n_constraints = length(constraints_functions);
 end

 %% INSTANCE PARAMETER
 run_function = @EmptyPreprocessing;
 if(strcmp(function_2_test{1},'g06'))
    fitness = @g06;
 end
 clean_function = @EmptyPostprocessing;
 
 %% CMAES PARAMETER
 explorationRate = 0.5; %0.1; %0.5; %0.1;%[0, 1]
 niter = 100;  %number of generations
 if(strcmp(function_2_test{1},'g06'))
    cmaes_value_range = [0 , 100];  % boudn that define the search space
 end
 % starting value of parameters
 generation_of_starting_point = 'random'; % 'test', 'given', 'random'
 switch generation_of_starting_point
        case 'test'
           start_action = user_defined_start_action;
        case 'given'
            start_action = init_parameters_from_out*ones(1,search_space_dimension);
        case 'random'
           start_action = (cmaes_value_range(1,2)-cmaes_value_range(1,1)).*rand(1,search_space_dimension) + cmaes_value_range(1,1)*ones(1,search_space_dimension);
        
 end
 
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
         inst =  Optimization.Instance(constr{jj},run_function,fitness,clean_function,[]);
         tic
         [mean_performances, bestAction, BestActionPerEachGen, policies, costs, succeeded, data2save] = inst.CMAES(search_space_dimension,start_action,niter,explorationRate,cmaes_value_range);
         exec_time = toc
         all_mean_perfomance(kk,:) = mean_performances';
         all_mean_perfomance_without_correction(kk,:) = data2save.performance';
         if(strcmp(method_to_use{jj},'adaptive'))
            all_weights{kk} = inst.penalty_handling.weights;
         end
         all_best_action(kk,:) = bestAction.parameters;
         all_best(kk,1) = bestAction.performance;
         last_cost = costs(end-last_generation:end);
         last_policies = policies(end-last_generation:end,:);
         [tmp , id]=min(last_cost);
         all_last_best(kk,1) = -costs(id);
         
         for ii=1:n_constraints
             all_best_violations(kk,ii) = feval(constraints_functions{ii},bestAction.parameters);
             all_last_best_violations(kk,ii) = feval(constraints_functions{ii},last_policies(id,:));
         end
         close all;
     end
     
     all_perfomance_without_constraint_correction{jj} = all_mean_perfomance_without_correction;
     all_perfomance_with_constraint_correction{jj} = all_mean_perfomance;
    
     
     % compute mean and variance of perfomance without correction
     prf_pure.average  = mean(all_mean_perfomance_without_correction,1);
     prf_pure.variance = var(all_mean_perfomance_without_correction);
     all_prf_pure{jj} = prf_pure;
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


% plot average fitness without correction
generation = 1:length(all_prf{1}.average);
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

% plot average fitness with correction
generation = 1:length(all_prf{1}.average);
figure
hold on;
for z = 1:length(method_to_use)
   plot(generation',all_prf{z}.average','Color',color_list{z});
   xlabel('generations','FontSize',16);
   ylabel('average fitness with correction','FontSize',16);
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


 
 
 
 
