clear variables
close all
clc

tic

% this 2 line are necessatry if i want to catch warning message from ode
warning on verbose
warning('error', 'MATLAB:ode15s:IntegrationTolNotMet');
warning('error', 'MATLAB:illConditionedMatrix')

% Parameters 
n_of_experiment = 3;      % number that we use to distinguish between the same static parameters settings but with different runtime parameters
init_parameters = 6;        %initial value for the optimization  (the scale is 0-12, so 6=0.5)
number_of_iteration = 1;   % number of  optimization (for robustness assesement)
generation_of_starting_point = 'given'; % 'test', 'given', 'random'


p = ProgressBar(number_of_iteration); 


parfor iter=1:number_of_iteration
    [tau, mean_performances, bestAction, policies, costs, succeeded]=OptimizationRoutine(number_of_iteration,n_of_experiment,iter,init_parameters,generation_of_starting_point);
     p.progress;
end

p.stop;


toc


