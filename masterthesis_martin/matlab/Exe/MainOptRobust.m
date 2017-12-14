clear variables
close all
clc

% IMPORTANT!!! in this executable the optimization method that we are going to use is defined in the
% main in the configuration files while in CMAESParallelBenchmark.m the
% optimization method is in the CMAESParallelBenchmark.m itself

%parpool

tic

% this 2 line are necessatry if i want to catch warning message from ode
warning on verbose
warning('error', 'MATLAB:ode15s:IntegrationTolNotMet');
warning('error', 'MATLAB:illConditionedMatrix');

% Parameters
n_of_experiment = 7; % number that we use to distinguish between the same static parameters settings but with different runtime parameters
init_parameters = 6; % initial value for the optimization  (the scale is 0-14, so 6=0.5)
number_of_experiment_ripetition = 20; % number of optimization (for robustness assesement)
current_experiment = 0;
configuration_file_name = 'config_icub_lift_obj';
all_results = cell(number_of_experiment_ripetition,1);

% create folder to contain all the experiment with a provisional name
name_folder   = 'current_experiments'; % do not change the provisional name!
allpath       = which('FindData.m');
path          = fileparts(allpath);
complete_path = strcat(path, '/results/', name_folder);
mkdir(complete_path);

for iter = 1:number_of_experiment_ripetition
    current_experiment = iter;
    [tau, mean_performances, bestAction, BestActionPerEachGen, policies, costs, succeeded] = OptimizationRoutine(number_of_experiment_ripetition, n_of_experiment, ...
     																										     iter, init_parameters, configuration_file_name);
    all_results{1,iter} = BestActionPerEachGen;
end

% rename folder at the end of the optimization procedure
new_complete_path = strcat(path, '/results/', new_name_folder);
movefile(complete_path, new_complete_path);
% save the variable all_results to make further analysis later
new_complete_path_to_file = strcat(new_complete_path, '/all_results_optimization.mat');
save(new_complete_path_to_file, 'all_results');
%p.stop;

toc

%parpool close
