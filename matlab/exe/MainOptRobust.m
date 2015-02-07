clear variables
close all
clc

% this 2 line are necessatry if i want to catch warning message from ode
warning on verbose
warning('error', 'MATLAB:ode15s:IntegrationTolNotMet');
warning('error', 'MATLAB:illConditionedMatrix')


% Parametets 
n_of_experiment = 1;        % number that we use to distinguish between the same static parameters settings but with different runtime parameters
init_parameters = 6;        %initial value for the optimization  
number_of_iteration = 1;    % number of  optimzation (for robustness assesement)
random = false;             % if true i randomize init_parameters for each iteration




for iter=1:number_of_iteration
    [tau, init_parameters, mean_performances, bestAction, policies, costs, succeeded]=OptimizationUF(n_of_experiment,iter,init_parameters,random);
end




