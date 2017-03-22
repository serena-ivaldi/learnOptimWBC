%Script computing the perfomances of fmincon on 3 known problems
%written by ugo chervet

clear variables
close all
clc

nbrep = 10; %50
MaxFunEvals = 50;
threshold = 20;
problem_name = {'g07'};
metric1 = [];
metric2 = [];
metric3 = [];
metric4 = [];

for i = 1:length(problem_name)  
    if(strcmp(problem_name{i},'g06'))
        search_space_dimension = 2;
        epsilon = [0.001, 0.001]; % for adaptive
        constraints_functions = {'g06Constr1','g06Constr2'}; 
        constraints_type = [1 1];      
        constraints_values =[0,0];
        n_constraints = length(constraints_functions);
        cmaes_value_range{1} = [13 , 0];  % lower bound that define the search space
        cmaes_value_range{2} = [100 , 100];  % upper bound that define the search space
        user_defined_start_action = [14.6111 2.1491]; 
    elseif(strcmp(problem_name{i},'tr2'))
        search_space_dimension = 2;
        epsilon = [0.001]; % for adaptive
        constraints_functions = {'tr2Constr1'}; 
        constraints_type = [1];      
        constraints_values =[0];
        n_constraints = length(constraints_functions);
        cmaes_value_range{1} = [0 , 0];  % lower bound that define the search space
        cmaes_value_range{2} = [100 , 100];  % upper bound that define the search space
        user_defined_start_action = [50,50];
    elseif(strcmp(problem_name{i},'g07'))
        search_space_dimension = 10;     
        epsilon = [0.001 0.001 0.001 0.001 0.001 0.001 0.001 0.001]; % for adaptive
        constraints_functions = {'g07Constr1','g07Constr2','g07Constr3','g07Constr4','g07Constr5','g07Constr6','g07Constr7','g07Constr8'}; 
        constraints_type = [1 1 1 1 1 1 1 1];      
        constraints_values =[0 0 0 0 0 0 0 0];
        n_constraints = length(constraints_functions);
        cmaes_value_range{1} = [-10 , -10, -10 ,-10, -10, -10,-10,-10,-10 ,-10 ];  % lower bound that define the search space
        cmaes_value_range{2} = [ 10, 10, 10, 10, 10, 10, 10, 10, 10 ,10];  % upper bound that define the search space
        user_defined_start_action = [2.22222222222222,3.70370370370370,6.66666666666666,8.88888888888889,0,6.66666666666666,0,0,6.66666666666666,6.66666666666666];
    elseif(strcmp(problem_name{i},'g09'))
        search_space_dimension = 7;
        epsilon = [1 1 1 1]; % for adaptive
        constraints_functions = {'g09Constr1','g09Constr2','g09Constr3','g09Constr4'}; 
        constraints_type = [1 1 1 1];      
        constraints_values =[0 0 0 0];
        n_constraints = length(constraints_functions);
        cmaes_value_range{1} = [-10 , -10, -10 ,-10, -10, -10,-10 ];  % lower bound that define the search space
        cmaes_value_range{2} = [ 10, 10, 10, 10, 10, 10, 10];  % upper bound that define the search space
        user_defined_start_action = [0,0,0,0,0,0,0];
    elseif(strcmp(problem_name{i},'g10')) % hard to find a feasible starting point
        search_space_dimension = 8;
        epsilon = [0.001 0.001 0.001 0.001 0.001 0.001]; % for adaptive
        constraints_functions = {'g10Constr1','g10Constr2','g10Constr3','g10Constr4','g10Constr5','g10Constr6'}; 
        constraints_type = [1 1 1 1 1 1];      
        constraints_values =[0 0 0 0 0 0];
        n_constraints = length(constraints_functions);
        cmaes_value_range{1} = [100 , 1000, 1000 ,10, 10, 10,10,10 ];  % lower bound that define the search space
        cmaes_value_range{2} = [ 1000, 10000, 10000, 1000, 1000, 1000, 1000, 1000];  % upper bound that define the search space
        %user_defined_start_action =
    elseif(strcmp(problem_name{i},'f240'))
        search_space_dimension = 5;
        epsilon = [1]; % for adaptive
        constraints_functions = {'f240Constr1'}; 
        constraints_type = [1];      
        constraints_values =[0];
        n_constraints = length(constraints_functions);
        cmaes_value_range{1} = [ 0, 0, 0, 0, 0];  % lower bound that define the search space
        cmaes_value_range{2} = [10000 , 10000, 10000 ,10000, 10000 ];  % upper bound that define the search space
        user_defined_start_action = [555.555555555556,1666.66666666667,555.555555555556,555.555555555556,555.555555555556];
    elseif(strcmp(problem_name{i},'f241'))
        search_space_dimension = 5;
        epsilon = [1]; % for adaptive
        constraints_functions = {'f240Constr1'}; 
        constraints_type = [1];      
        constraints_values =[0];
        n_constraints = length(constraints_functions);
        cmaes_value_range{1} = [ 0, 0, 0, 0, 0];  % lower bound that define the search space
        cmaes_value_range{2} = [100000 , 100000, 100000 ,100000, 100000 ];  % upper bound that define the search space
        user_defined_start_action =[1851.85185185185,617.283950617287,617.283950617287,617.283950617287,617.283950617287];
    elseif(strcmp(problem_name{i},'HB'))
        search_space_dimension = 5;
        epsilon = [1 1 1 1 1 1]; % for adaptive
        constraints_functions = {'HBConstr1','HBConstr2','HBConstr3','HBConstr4','HBConstr5','HBConstr6'}; 
        constraints_type = [1 1 1 1 1 1];      
        constraints_values =[0 0 0 0 0 0];
        n_constraints = length(constraints_functions);
        cmaes_value_range{1} = [ 78 , 33, 27 ,27, 27 ];  % lower bound that define the search space
        cmaes_value_range{2} = [ 102, 45, 45, 45, 45];  % upper bound that define the search space
        user_defined_start_action = [90,35,36,36,36];
    end
    
    constr =Optimization.ObjProblemPenalty(search_space_dimension,constraints_functions,constraints_type,constraints_values);
    algorithm = 'fmincon';
    run_function = @EmptyPreprocessing;
    fitness = str2func(problem_name{i});
    clean_function = @EmptyPostprocessing;
    input = {};
    inst = ObjProblem(search_space_dimension,cmaes_value_range,constr,algorithm,run_function,fitness,clean_function,input);
    
    for iter=1:nbrep
        inst.randStartPoint();
        [~,~,m1(iter),m2(iter),m3(iter),m4(iter)] = inst.minimize(user_defined_start_action,MaxFunEvals,threshold);
    end
    metric1 =  [metric1,  m1]; %metric 1 = fitness error
    metric2 =  [metric2,  m2]; %metric 2 = constraints violation
    metric3 =  [metric3,  m3]; %metric 3 = # of steps to steady value
    metric4 =  [metric4,  m4]; %metric 4 = compuating duration
end

%matrix to automaticly adpat the box plot to the number of problems
A=[];
for i=1:length(problem_name)
    A = [A; i*ones(nbrep,1)];
end

figure(1);
colors = [1 0 0; 0 0 1; 0 0.5 0];
subplot(1,4,1)
boxplot(metric1', A, 'Colors',colors);
title('Fitness error','FontSize',10);
subplot(1,4,2)
boxplot(metric2', A, 'Colors',colors);
title('Constraints violation','FontSize',10);
subplot(1,4,3)
boxplot(metric3', A, 'Colors',colors);
title('# of steps to steady value','FontSize',10);
subplot(1,4,4)
boxplot(metric4', A, 'Colors',colors);
title('Times','FontSize',10);
