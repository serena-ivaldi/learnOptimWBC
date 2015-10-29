%% test main for the cmaes optimizer 
clear variables
close all
clc



function_2_test = 'g06';

method_to_use = 'adaptive';  % adaptive , vanilla


search_space_dimension = 2;
 %% CONSTRAINTS PARAMETERS
 if(strcmp(function_2_test,'g06'))
     epsilon = [1, 1, 1];
     constraints_functions = {'g06Constr1','g06Constr2','g06Constr3'}; 
     constraints_type = [1 1 1];      
     constraints_values =[0,0,0];
 end
 
 activate_constraints_handling = true;

 %% INSTANCE PARAMETER
 run_function = @EmptyPreprocessing;
 if(strcmp(function_2_test,'g06'))
    fitness = @g06;
 end
 clean_function = @EmptyPostprocessing;
 
 %% CMAES PARAMETER
 explorationRate = 0.1; %0.1; %0.5; %0.1;%[0, 1]
 niter = 1000;  %number of generations
 if(strcmp(function_2_test,'g06'))
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
 
 %% Constraints
 if(strcmp(method_to_use,'vanilla'))
    constr=Optimization.FixPenalty(search_space_dimension,constraints_functions,constraints_type,constraints_values);
 elseif(strcmp(method_to_use,'adaptive'))
    constr = Optimization.AdaptivePenalty(epsilon,niter,search_space_dimension,constraints_functions,constraints_type,constraints_values);
 end
     
 %% Instance
 inst =  Optimization.Instance(constr,activate_constraints_handling,run_function,fitness,clean_function,[]);

 
 tic
 [mean_performances, bestAction, BestActionPerEachGen, policies, costs, succeeded] = inst.CMAES(search_space_dimension,start_action,niter,explorationRate,cmaes_value_range);
 exec_time = toc

