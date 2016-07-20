%% METRICS
% m1 = % distance from best action
% m2 = % sum of constraints violations 
% m2_1 = constraintsviolations
% m3 = % steady state solutions
% m4 = % execution time
% %-----------------------------------------------------------------------------------
% m5 = % fitness function 
% m6 = % best action
% m7 = % best perfomance

% IMPORTANT!!! this executable the optimization method that we are going to use is defined in the
% main while in MainOptRobust.m the optimization method is defined in the
% configuration files

function ParallelBechmarkRoutine(local_path,learn_approach,method_to_use,function_2_test,n_of_experiment,robotics_experiment,generation_of_starting_point,niter_tot,threshold)  
    
    disp(strcat('********* current experiment = ',num2str(n_of_experiment), '****** '))
    %% initialize all the data
    %% CONSTRAINTS PARAMETERS
    if(robotics_experiment)
        %% initialize all the data
        optim = true;
        [epsilon,search_space_dimension,explorationRate,cmaes_value_range,...
        n_constraints,constraints_functions,constraints_type,constraints_values,run_function,fitness,clean_function,input]=InitForBenchmark(function_2_test,optim);
        user_defined_start_action = [0 0 0 0 0 14 14 14 14 14 0 0 0 0 0 ]; 
        benchmark_x = [];
    else
        if(strcmp(function_2_test,'g06'))
            search_space_dimension = 2;
            function_2_test_4_comparison = {'g06Test'};
            epsilon = [0.01, 0.01]; % for adaptive
            constraints_functions = {'g06Constr1','g06Constr2'};
            constraints_for_test = 'g06Constr';
            constraints_type = [1 1];
            constraints_values =[0,0];
            n_constraints = length(constraints_functions);
            cmaes_value_range{1} = [13 , 0];  % lower bound that define the search space
            cmaes_value_range{2} = [100 , 100];  % upper bound that define the search space
            user_defined_start_action = [14.6111 2.1491];
            benchmark_x = [14.095000000000038,0.842960789162843];
            benchmark_fval = 6.961813875638089e+03;
        elseif(strcmp(function_2_test,'tr2'))
            search_space_dimension = 2;
            function_2_test_4_comparison = {'tr2Test'};
            epsilon = [0.01]; % for adaptive
            constraints_functions = {'tr2Constr1'};
            constraints_for_test = 'tr2Constr';
            constraints_type = [1];
            constraints_values =[0];
            n_constraints = length(constraints_functions);
            cmaes_value_range{1} = [0 , 0];  % lower bound that define the search space
            cmaes_value_range{2} = [100 , 100];  % upper bound that define the search space
            user_defined_start_action = [50,50];
            benchmark_x = 0;
            benchmark_fval = 0;
        elseif(strcmp(function_2_test,'g07'))
            search_space_dimension = 10;
            function_2_test_4_comparison = {'g07Test'};
            epsilon = [0.01 0.01 0.01 0.01 0.01 0.01 0.01 0.01]; % for adaptive
            constraints_functions = {'g07Constr1','g07Constr2','g07Constr3','g07Constr4','g07Constr5','g07Constr6','g07Constr7','g07Constr8'};
            constraints_for_test = 'g07Constr';
            constraints_type = [1 1 1 1 1 1 1 1];
            constraints_values =[0 0 0 0 0 0 0 0];
            n_constraints = length(constraints_functions);
            cmaes_value_range{1} = [-10 , -10, -10 ,-10, -10, -10,-10,-10,-10 ,-10 ];  % lower bound that define the search space
            cmaes_value_range{2} = [ 10, 10, 10, 10, 10, 10, 10, 10, 10 ,10];  % upper bound that define the search space
            user_defined_start_action = [2.22222222222222,3.70370370370370,6.66666666666666,8.88888888888889,0,6.66666666666666,0,0,6.66666666666666,6.66666666666666];
            benchmark_x = [2.17199636680015,2.36368298196841,8.77392573462452,5.09598446196233,0.990654711300080,1.43057385002253,1.32164420106261,9.82872580290504,8.28009162124264,8.37592656787441];
            benchmark_fval = 24.306209068179456;
        elseif(strcmp(function_2_test,'g09'))
            search_space_dimension = 7;
            function_2_test_4_comparison = {'g09Test'};
            epsilon = [0.01 0.01 0.01 0.01]; % for adaptive
            constraints_functions = {'g09Constr1','g09Constr2','g09Constr3','g09Constr4'};
            constraints_for_test = 'g09Constr';
            constraints_type = [1 1 1 1];
            constraints_values =[0 0 0 0];
            n_constraints = length(constraints_functions);
            cmaes_value_range{1} = [-10 , -10, -10 ,-10, -10, -10,-10 ];  % lower bound that define the search space
            cmaes_value_range{2} = [ 10, 10, 10, 10, 10, 10, 10];  % upper bound that define the search space
            user_defined_start_action = [0,0,0,0,0,0,0];
            benchmark_x = [2.33049981339068,1.95137229535189,-0.477539284687175,4.36572621388739,-0.624486692382246,1.03813151729074,1.59422710623982];
            benchmark_fval = 6.806300573743688e+02;
        elseif(strcmp(function_2_test,'g10')) % hard to find a feasible starting point
            search_space_dimension = 8;
            function_2_test_4_comparison = {'g10Test'};
            epsilon = [0.01 0.01 0.01 0.01 0.01 0.01]; % for adaptive
            constraints_functions = {'g10Constr1','g10Constr2','g10Constr3','g10Constr4','g10Constr5','g10Constr6'};
            constraints_for_test = 'g10Constr';
            constraints_type = [1 1 1 1 1 1];
            constraints_values =[0 0 0 0 0 0];
            n_constraints = length(constraints_functions);
            cmaes_value_range{1} = [100 , 1000, 1000 ,10, 10, 10,10,10 ];  % lower bound that define the search space
            cmaes_value_range{2} = [ 1000, 10000, 10000, 1000, 1000, 1000, 1000, 1000];  % upper bound that define the search space
            benchmark_x = 0;
            benchmark_fval = 0;
            %user_defined_start_action =
        elseif(strcmp(function_2_test,'f240'))
            search_space_dimension = 5;
            function_2_test_4_comparison = {'f240Test'};
            epsilon = [0.01]; % for adaptive
            constraints_functions = {'f240Constr1'};
            constraints_for_test = 'f240Constr';
            constraints_type = [1];
            constraints_values =[0];
            n_constraints = length(constraints_functions);
            cmaes_value_range{1} = [ 0, 0, 0, 0, 0];  % lower bound that define the search space
            cmaes_value_range{2} = [10000 , 10000, 10000 ,10000, 10000 ];  % upper bound that define the search space
            user_defined_start_action = [555.555555555556,1666.66666666667,555.555555555556,555.555555555556,555.555555555556];
            benchmark_x = [5000.00000000000,0,3.38813178901720e-21,3.15544362088405e-30,0];
            benchmark_fval = -5.000000000000001e+03;
        elseif(strcmp(function_2_test,'f241'))
            search_space_dimension = 5;
            function_2_test_4_comparison = {'f241Test'};
            epsilon = [0.01]; % for adaptive
            constraints_functions = {'f240Constr1'};
            constraints_for_test = 'f240Constr';
            constraints_type = [1];
            constraints_values =[0];
            n_constraints = length(constraints_functions);
            cmaes_value_range{1} = [ 0, 0, 0, 0, 0];  % lower bound that define the search space
            cmaes_value_range{2} = [100000 , 100000, 100000 ,100000, 100000 ];  % upper bound that define the search space
            user_defined_start_action =[1851.85185185185,617.283950617287,617.283950617287,617.283950617287,617.283950617287];
            benchmark_x = [2.54109884097404e-21,8.47032947254300e-22,3.00712570113065e-22,0,3571.42857142857];
            benchmark_fval = -1.785714285714286e+04;
        elseif(strcmp(function_2_test,'HB'))
            search_space_dimension = 5;
            function_2_test_4_comparison = {'HBTest'};
            epsilon = [0.01 0.01 0.01 0.01 0.01 0.01]; % for adaptive
            constraints_functions = {'HBConstr1','HBConstr2','HBConstr3','HBConstr4','HBConstr5','HBConstr6'};
            constraints_for_test = 'HBConstr';
            constraints_type = [1 1 1 1 1 1];
            constraints_values =[0 0 0 0 0 0];
            n_constraints = length(constraints_functions);
            cmaes_value_range{1} = [ 78 , 33, 27 ,27, 27 ];  % lower bound that define the search space
            cmaes_value_range{2} = [ 102, 45, 45, 45, 45];  % upper bound that define the search space
            user_defined_start_action = [90,35,36,36,36];
            benchmark_x = [78,33,29.9952560256816,45,36.7758129057882];
            benchmark_fval = -3.066553867178332e+04;
        end
        explorationRate = 0.1; %0.1; %0.5; %0.1;%[0, 1]
        run_function = @EmptyPreprocessing;
        fitness = str2func(function_2_test);
        clean_function = @EmptyPostprocessing;
        input = [];
    end
    %% CONSTRAINTS
    % number of candidates per generation for cmaes
    lambda = round(4 + 3 * log(search_space_dimension)); % number of candidates used in cmaes
    % to compare the three methods i have to normalize the number of iterations with the
    % number of candidates per generation
    if(strcmp(learn_approach,'CMAES'))
        niter = round(niter_tot/lambda);
        %niter = niter_tot;
    elseif(strcmp(learn_approach,'(1+1)CMAES'))
        niter = niter_tot;
    elseif(strcmp(learn_approach,'CEM'))
        niter = round(niter_tot/lambda);
    elseif(strcmp(learn_approach,'fmincon'))
        niter = niter_tot;
    end

    if(strcmp(method_to_use,'vanilla'))
        constr =Optimization.FixPenalty(search_space_dimension,constraints_functions,constraints_type,constraints_values);
    elseif(strcmp(method_to_use,'adaptive'))
        constr = Optimization.AdaptivePenalty(epsilon,niter,search_space_dimension,constraints_functions,constraints_type,constraints_values);
    elseif(strcmp(method_to_use,'fmincon'))
        constr =Optimization.ObjProblemPenalty(search_space_dimension,constraints_functions,constraints_type,constraints_values);
    end

    switch generation_of_starting_point
        case 'test'
            start_action = user_defined_start_action;
        case 'given'
            start_action = init_parameters_from_out*ones(1,search_space_dimension);
        case 'random'
            if(iscell(cmaes_value_range))
                start_action = (cmaes_value_range{2}-cmaes_value_range{1}).*rand(1,search_space_dimension) + cmaes_value_range{1};
            elseif(isvector(cmaes_value_range))
                start_action = (cmaes_value_range(1,2)-cmaes_value_range(1,1)).*rand(1,search_space_dimension) + cmaes_value_range(1,1)*ones(1,search_space_dimension);
            end
        otherwise
            disp('wrong method written!')
    end

    if(strcmp(learn_approach,'fmincon'))
        inst = ObjProblem(search_space_dimension,cmaes_value_range,constr,learn_approach,run_function,fitness,clean_function,input);
        tic
        [mean_performances,bestAction] = inst.minimize(start_action,niter,threshold);
        % execution time
        m4 = toc;
        m8 = inst;
    else
        inst =  Optimization.Instance(constr,learn_approach,run_function,fitness,clean_function,input);
        tic
        [mean_performances, bestAction,~,~,~,~,G_data2save] = inst.CMAES(search_space_dimension,start_action,niter,explorationRate,cmaes_value_range); 
        % execution time
        m4 = toc;
        % many data on the experiment
        inst.data2save = G_data2save;
        m8 = inst;
    end
    %% collect all the data from each experiments
    
    
    
    % perfomance
    if(strcmp(learn_approach,'fmincon'))
        if(isrow(mean_performances))
            m5{1} = mean_performances';
        else
            m5{1} = mean_performances;
        end
    else
        if(isrow(mean_performances))
            m5 = mean_performances';
        else
            m5 = mean_performances;
        end
    end
    
    %m3(kk,1) = IndetifySteadyState(m5{kk,:},threshold);
    % best results
    m6 = bestAction.parameters;
    m1 =[];
    if(~robotics_experiment)
        % distance from benchmark actions
        m1 = norm(bestAction.parameters - benchmark_x);
    end
    % best perfomance
    m7 = bestAction.performance;
    % violations
    [c, ceq] = inst.computeConstr(bestAction.parameters);
    m2 = sum(abs((c > 0).*c)) + sum(abs((ceq ~= 0).*ceq));
    m2_1 = inst.penalty_handling.penalties(1,:);
    close all 
    if(robotics_experiment)
        robotic_flag = true;
    else
        robotic_flag = false;
    end
    
    if(strcmp(learn_approach,'fmincon'))
        name_to_save = strcat(local_path,'/','mn.mat');
    else
        name_to_save = strcat(local_path,'/',num2str(n_of_experiment),'.mat');
    end
    save(name_to_save,'m1','m2','m2_1','m4','m5','m6','m7','m8','robotic_flag');
end
