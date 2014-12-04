clear all
close all
clc

% this 2 line are necessatry if i want to catch warning message from ode
warning on verbose
warning('error', 'MATLAB:ode15s:IntegrationTolNotMet');
warning('error', 'MATLAB:illConditionedMatrix')




%ALPHA PARAMETERS

%rbf
number_of_basis = 10;
redundancy = 3;
range = [0 , 12];
precomp_sample = false;
numeric_theta = 2*ones(number_of_basis,1);
%constant alpha
value1 = 1*ones(chains.GetNumTasks(1));
values{1} = value1;


% INSTANCE PARAMETERS
qi{1} = qz;
qdi{1} = zeros(1,chains.GetNumLinks(1));
fitness= @fitness2;
options= [];
simulator_type = {'rbt'};


% CMAES PARAMETER
% starting value of parameters
explorationRate =0.1;%[0, 1]
niter = 100;

%% Reference
% if type_of_task = sampled i have to specify the Time to reach the
% end of the trajectories that is equal to the simulation time
reference = References(target_link,type,control_type,traj,geom_parameters,time_law,time_struct,dim_of_task,type_of_traj);
reference.BuildTrajs();

%% Alpha

%alphas = ConstantAlpha.BuildCellArray(chains.GetNumChains(),chains.GetNumTasks(1),values,time_struct);
alphas = RBF.BuildCellArray(chains.GetNumChains(),chains.GetNumTasks(1),time_struct,number_of_basis,redundancy,range,precomp_sample,numeric_theta);        

%% Controller 

% for using package function we have to call the name of the package before
% the constructor
controller = Controllers.UF(chains,reference,alphas,metric,Kp,Kd,combine_rule,max_time,display_opt);

%% Instance
start_action = 6*ones(1,controller.GetTotalParamNum());
inst = Instance(controller,simulator_type,qi,qdi,time_sym_struct,fixed_step,fitness,options);
[mean_performances bestAction policies costs succeeded] = inst.CMAES(start_action,niter,explorationRate);







