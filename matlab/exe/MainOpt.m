clear all
close all
clc

% this 2 line are necessatry if i want to catch warning message from ode
warning on verbose
warning('error', 'MATLAB:ode15s:IntegrationTolNotMet');
warning('error', 'MATLAB:illConditionedMatrix')

%GENERAL PARAMETERS
% for other strucutures
time_struct.ti = 0;
time_struct.tf = 10;
time_struct.step = 0.1;

% for simulation 
time_sym_struct = time_struct;
time_sym_struct.step = 0.001; 
fixed_step = false;

% TASK PARAMETERS
path=LoadParameters('2_LBR4p_scene2_position_tasks');
load(path);

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

%CONTROLLER PARAMETERS
max_time = 50;
combine_rule = {'sum'}; 

% INSTANCE PARAMETERS
qi{1} = qz;
qdi{1} = zeros(1,chains.GetNumLinks(1));
options= [];
simulator_type = {'rbt'};

% CMAES PARAMETER
% starting value of parameters
explorationRate =0.1;%[0, 1]
niter = 10;

%% Reference
% if type_of_task = sampled i have to specify the Time to reach the
% end of the trajectories that is equal to the simulation time
reference = References(target_link,type,control_type,traj,geom_parameters,time_law,time_struct,dim_of_task,type_of_traj);
reference.BuildTrajs();

%% Alpha
alphas = Alpha.RBF.BuildCellArray(chains.GetNumChains(),chains.GetNumTasks(1),time_struct,number_of_basis,redundancy,range,precomp_sample,numeric_theta);   
%alphas = Alpha.ConstantAlpha.BuildCellArray(chains.GetNumChains(),chains.GetNumTasks(1),values,time_struct);

%% Controller 

% for using package function we have to call the name of the package before
% the constructor
controller = Controllers.UF(chains,reference,alphas,metric,Kp,Kd,combine_rule,max_time);

%% Instance
start_action = 6*ones(1,controller.GetTotalParamNum());
inst = Instance(controller,simulator_type,qi,qdi,time_sym_struct,fixed_step,fitness,options);
[mean_performances bestAction policies costs succeeded] = inst.CMAES(start_action,niter,explorationRate);







