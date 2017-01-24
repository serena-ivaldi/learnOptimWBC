
function CMAEStest
%% test main for the cmaes optimizer
%% this file is conceived for testing one method with different test functions
clear variables
close all
clc

%% DATA 1
robotics_experiment = [0]; % series of value that say if the current experiment is a robotics experiments or not 1 or 0
niter_tot = 100;  %number of functions evaluations
function_2_test ={'to_test_withBOGP_stuff'};%'robotic_experiments','g06','g07','g09','f240','f241','HB'};
learn_approach = 'BO(1+1)CMAES'; %CMAES (1+1)CMAES  CEM BO(1+1)CMAES fmincon     with (1+1)CMAES and BO(1+1)CMAES i have to use nopenalty
method_to_use = 'nopenalty';  % adaptive , vanilla ,empty,fmincon, nopenalty

repetition_of_the_experiment = 2; % at least 2
threshold = 2.5; % value to identify the beginning of steady state
% the threshold is express in %, means +/- 2,5% from the steady value

% starting value of parameters
generation_of_starting_point = 'test'; % 'test', 'given', 'random'
number_of_function_2_test = length(function_2_test);

current_experiment=0;

%% INSTANCE PARAMETER

%% global metric
metric1 = zeros(repetition_of_the_experiment,number_of_function_2_test);  % distance from best action
metric2 = zeros(repetition_of_the_experiment,number_of_function_2_test);  % constraints violations
metric3 =  zeros(repetition_of_the_experiment,number_of_function_2_test); % steady state solutions
metric4 =  zeros(repetition_of_the_experiment,number_of_function_2_test); % execution time
%----------------------------------------------------------------------------------------------------
metric5 = zeros(repetition_of_the_experiment,number_of_function_2_test);  % total_performance
metric6 = [];                                                                % best action  (cell variable)
metric7 = [];                                                                % best perfomance

for jj=1:number_of_function_2_test
    %% CONSTRAINTS PARAMETERS
    if(robotics_experiment(jj))
        %% initialize all the data
        optim = true;
        %[epsilon,search_space_dimension,explorationRate,cmaes_value_range,...
        %    n_constraints,constraints_functions,constraints_type,constraints_values,run_function,fitness,clean_function,input]=InitForBenchmark(function_2_test{jj},optim);
       % user_defined_start_action = []; % to use for  (1+1)cmaes
        
         [epsilon,search_space_dimension,explorationRate,cmaes_value_range,...
           n_constraints,constraints_functions,constraints_type,constraints_values,run_function,fitness,clean_function,input]=InitForBenchmark(function_2_test{jj},optim);
       user_defined_start_action = [0 0 0 0 0 14 14 14 14 14 0 0 0 0 0 ]; 
       % to use for  (1+1)cmaes

        
        
        
    else
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
            benchmark_x = [14.095000000000038,0.842960789162843];
            benchmark_fval = 6.961813875638089e+03;
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
            benchmark_x = 0;
            benchmark_fval = 0;
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
            benchmark_x = [2.17199636680015,2.36368298196841,8.77392573462452,5.09598446196233,0.990654711300080,1.43057385002253,1.32164420106261,9.82872580290504,8.28009162124264,8.37592656787441];
            benchmark_fval = 24.306209068179456;
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
            benchmark_x = [2.33049981339068,1.95137229535189,-0.477539284687175,4.36572621388739,-0.624486692382246,1.03813151729074,1.59422710623982];
            benchmark_fval = 6.806300573743688e+02;
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
            benchmark_x = 0;
            benchmark_fval = 0;
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
            benchmark_x = [5000.00000000000,0,3.38813178901720e-21,3.15544362088405e-30,0];
            benchmark_fval = -5.000000000000001e+03;
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
            benchmark_x = [2.54109884097404e-21,8.47032947254300e-22,3.00712570113065e-22,0,3571.42857142857];
            benchmark_fval = -1.785714285714286e+04;
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
            benchmark_x = [78,33,29.9952560256816,45,36.7758129057882];
            benchmark_fval = -3.066553867178332e+04;
        elseif(strcmp(function_2_test{jj},'rosenbrock'))
            search_space_dimension = 2;
            function_2_test_4_comparison = {''};
            epsilon = [1 1 1 1 1 1]; % for adaptive
            constraints_functions = {};
            constraints_for_test = [];
            constraints_type = [];
            constraints_values =[];
            n_constraints = 0;
            cmaes_value_range{1} = [ -1.5 , -0.5];  % lower bound that define the search space
            cmaes_value_range{2} = [ 2.0, 3.0];  % upper bound that define the search space
            user_defined_start_action = [];
            benchmark_x = [1,1];
            benchmark_fval = 0;
        elseif(strcmp(function_2_test{jj},'to_test_withBOGP_stuff'))
            search_space_dimension = 2;
            function_2_test_4_comparison = {''};
            epsilon = [1 1 1 1 1 1]; % for adaptive
            constraints_functions = {'stuffGPConstr1','stuffGPConstr1_1','stuffGPConstr2','stuffGPConstr2_1'};
            constraints_for_test = [];
            constraints_type = [1 1 1 1];
            constraints_values =[0 0 0 0];
            n_constraints = 4;
            cmaes_value_range{1} = [0 0];  % lower bound that define the search space
            cmaes_value_range{2} = [ 10.0, 10.0];  % upper bound that define the search space
            user_defined_start_action = [];
            benchmark_x = [1,1];
            benchmark_fval = 0;
        end
        explorationRate = 0.1; %0.1; %0.5; %0.1;%[0, 1]
        run_function = @EmptyPreprocessing;
        fitness = str2func(function_2_test{jj});
        clean_function = @EmptyPostprocessing;
        input = [];
    end
    
    %% OPTIMIZATION WITH DERIVATIVE METHOD FOR BENCHMARKING
    %     options = optimoptions(@fmincon,'Algorithm','sqp');
    %     x0 = start_action;
    %     [benchmark_x,benchmark_fval] = fmincon(str2func(function_2_test_4_comparison{1}),x0,[],[],[],[],cmaes_value_range{1},cmaes_value_range{2},str2func(constraints_for_test),options);
    %     disp('problem solved');
    %% CONSTRAINTS
    % number of candidates per generation for cmaes
    lambda = round(4 + 3 * log(search_space_dimension)); % number of candidates used in cmaes
    % to compare the three methods i have to normalize the number of iterations with the
    % number of candidates per generation
    if(strcmp(learn_approach,'CMAES'))
        niter = round(niter_tot/lambda);
    elseif(strcmp(learn_approach,'(1+1)CMAES'))
        niter = niter_tot;
    elseif(strcmp(learn_approach,'CEM'))
        niter = round(niter_tot/lambda);
    elseif(strcmp(learn_approach,'fmincon'))
        niter = niter_tot;
    elseif(strcmp(learn_approach,'BO(1+1)CMAES'))
        niter = niter_tot;
    end
    
    if(strcmp(method_to_use,'nopenalty')) % for 1+1CMAES and BO
       constr =Optimization.NoPenalty(search_space_dimension,constraints_functions,constraints_type,constraints_values);
    elseif(strcmp(method_to_use,'vanilla'))
        constr =Optimization.FixPenalty(search_space_dimension,constraints_functions,constraints_type,constraints_values);
    elseif(strcmp(method_to_use,'adaptive'))
        constr = Optimization.AdaptivePenalty(epsilon,niter,search_space_dimension,constraints_functions,constraints_type,constraints_values);
    elseif(strcmp(method_to_use,'fmincon'))
        constr =Optimization.ObjProblemPenalty(search_space_dimension,constraints_functions,constraints_type,constraints_values);
    end
    
    %% DATA 2
    
    m1 = zeros(repetition_of_the_experiment,1);  % distance from best action
    m2 = zeros(repetition_of_the_experiment,n_constraints);  % constraints violations
    m3 = zeros(repetition_of_the_experiment,1); % steady state solutions
    m4 = zeros(repetition_of_the_experiment,1); % execution time
    %-----------------------------------------------------------------------------------
    m5 = cell(repetition_of_the_experiment,1);  % total_performance
    m6 = zeros(repetition_of_the_experiment,search_space_dimension);      % best action
    m7 = zeros(repetition_of_the_experiment,1);                           % best perfomance
    
    %% OPTIMIZATION
    
    for kk = 1:repetition_of_the_experiment
        
        current_experiment=kk;
        disp(strcat('********* current experiment = ',num2str(current_experiment), '****** '))
        
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
            m4(kk) = toc;
        else
            inst =  Optimization.Instance(constr,learn_approach,run_function,fitness,clean_function,input);
            tic
            [mean_performances, bestAction] = inst.CMAES(search_space_dimension,start_action,niter,explorationRate,cmaes_value_range);
            % execution time
            m4(kk) = toc;
        end
        
        %% collect all the data from each experiments
        % perfomance
        m5{kk,:} = mean_performances';
        m3(kk) = IndetifySteadyState(m5{kk,:},threshold);
        % best results
        m6(kk,:) = bestAction.parameters;
        if(~robotics_experiment(jj))
            % distance from benchmark actions
            m1(kk) = norm(bestAction.parameters - benchmark_x);
        end
        % best perfomance
        m7(kk,1) = bestAction.performance;
        
        % violations
        %for ii=1:n_constraints
        %    m2(kk,ii) = feval(constraints_functions{ii},bestAction.parameters);
        %end
        % violations
        [c, ceq] = inst.computeConstr(bestAction.parameters);
        m2 = sum(abs((c > 0).*c)) + sum(abs((ceq ~= 0).*ceq));
        close all
        
    end
    
    
    %% data collected at the end of each test function
    try
        m5 = cell2mat(m5);
    catch err
        m5 = convert2mat(m5);
    end
    all_perfomance_with_constraint_correction{jj} = m5;
    % compute the mean and variance of overall perfomance with constraints
    % correction
    prf.average  = mean(m5,1);
    prf.variance = var(m5);
    all_prf{jj} = prf;
    % mean and variance of the best perfomance over all the experiments
    metric7(:,jj) = -m7;
    if(~robotics_experiment(jj))
        % the distance from optimal solutions
        metric1(:,jj) = m1;
    end
    % compute the sum of violations per each test function (i just sum the effective violations)
    mask = m2 > 0;
    m2 = m2.*mask;
    metric2(:,jj) = sum(m2,2);
    
    % best action for each repetition of the exepriment
    metric6{jj} = m6;
    
    % all steady state
    if(strcmp(learn_approach,'CMAES'))
        metric3(:,jj)=m3*lambda;
    else
        metric3(:,jj)=m3;
    end
    
    % execution time
    metric4(:,jj) = m4;
    
    % clean variable
    m1 = [];
    m2 = [];
    m3 = [];
    m4 = [];
    m5 = [];
    m6 = [];
    m7 = [];
    
end
%%  SAVE PATH
allpath=which('FindData.m');
local_path=fileparts(allpath);
local_path = strcat(local_path,'/benckmark');
name_folder = strcat(learn_approach,'-',method_to_use);
mkdir(local_path,name_folder);
local_path = strcat(local_path,'/',name_folder);


%% PLOT
color_list={'b','r','m','g','c','k','y'};
transparent_flag = 1;
%handle_legend = [];


% plot average fitness with correction
h3=figure;
hold on;
for z = 1:length(function_2_test)
    generation = 1:length(all_prf{z}.average);
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
    generation = 1:length(all_prf{z}.average);
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
if(~robotics_experiment(jj))
    % plotbox plot with the distance from the true policy
    h5=figure;
    boxplot(metric1,function_2_test)
    text_title ='policy error';
    title(text_title,'FontSize',20);
    name_fig = strcat(local_path,'/','policy_error');
    saveas(h5,name_fig);
end
%plotbox plot of the sum of constraints violations
h5_1=figure;
boxplot(metric2,function_2_test)
text_title ='constraints violations';
title(text_title,'FontSize',20);
name_fig = strcat(local_path,'/','constraints_violations');
saveas(h5_1,name_fig);
%plotbox plot of the best performance
h7=figure;
boxplot(metric7,function_2_test)
text_title ='best performance';
title(text_title,'FontSize',20);
name_fig = strcat(local_path,'/','best_performance');
saveas(h7,name_fig);
% plotbox plot with the steady state time
h9=figure;
boxplot(metric3,function_2_test)
text_title ='generation to reach the steady value';
name_fig = strcat(local_path,'/','generation_to_steady');
saveas(h9,name_fig);
title(text_title,'FontSize',20);
% plotbox plot with the steady state time
h10=figure;
boxplot(metric4,function_2_test)
text_title ='time to compute a solution';
name_fig = strcat(local_path,'/','generation_to_steady');
saveas(h10,name_fig);
title(text_title,'FontSize',20);

name_file = strcat(local_path,'/','dat.mat');
save(name_file);

end


function zzz = IndetifySteadyState(vector,tresh)
steady_value = vector(end);
for zzz = 1:length(vector)
    if(abs(steady_value-vector(zzz))<(tresh/100*steady_value))
        break
    end
end
end


% convert a cell with rows of different length in a matrix  by completing
% the short row with
function out = convert2mat(in)
maxLength=max(cellfun(@(x)numel(x),in));
out=cell2mat(cellfun(@(x)cat(2,x,zeros(1,maxLength-length(x))),in,'UniformOutput',false));
for row = 1:size(out,1)
    for col = 2:size(out,2)        
        if out(row,col) == 0
            out(row,col) = out(row,col-1);
        end
    end
end
end
