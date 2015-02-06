clear variables
close all
clc

% this 2 line are necessatry if i want to catch warning message from ode
warning on verbose
warning('error', 'MATLAB:ode15s:IntegrationTolNotMet');
warning('error', 'MATLAB:illConditionedMatrix')


% Parametets 
init_parameters = 6;
number_of_iteration = 1;
random = false;




for iter=1:number_of_iteration
    [tau, init_parameters, mean_performances, bestAction, policies, costs, succeeded]=OptimizationUF(init_parameters,iter,random);
end




