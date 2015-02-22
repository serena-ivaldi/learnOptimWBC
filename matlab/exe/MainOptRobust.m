clear variables
close all
clc

tic

% this 2 line are necessatry if i want to catch warning message from ode
warning on verbose
warning('error', 'MATLAB:ode15s:IntegrationTolNotMet');
warning('error', 'MATLAB:illConditionedMatrix')


% Parameters 
%n_of_experiment = 5;        % number that we use to distinguish between the same static parameters settings but with different runtime parameters
%init_parameters = 6;        %initial value for the optimization  
%number_of_iteration = 3;    % number of  optimzation (for robustness assesement)
%random = false;             % if true i randomize init_parameters for each iteration

% SERE
% Parameters 
n_of_experiment = 106;        % number that we use to distinguish between the same static parameters settings but with different runtime parameters
init_parameters = 6;        %initial value for the optimization  (the scale is 0-12, so 6=0.5)
number_of_iteration = 10;    % number of  optimzation (for robustness assesement)
random = false;             % if true i randomize init_parameters for each iteration


p = ProgressBar(number_of_iteration); 




parfor iter=1:number_of_iteration
    [tau, mean_performances, bestAction, policies, costs, succeeded]=OptimizationRoutine(number_of_iteration,n_of_experiment,iter,init_parameters,random);
     p.progress;
end

p.stop;


toc


