clear all
close all
clc

%GENERAL PARAMETERS
% for other strucutures
time_struct.ti = 0;
time_struct.tf = 10;
time_struct.step = 0.1;

% for simulation 
% options= odeset('MaxStep',0.001);
time_sym_struct = time_struct;
time_sym_struct.step = 0.01; 
fixed_step = false;

% TASK PARAMETERS
name_dat='LBR4p3__scene3_ee_tracking_circ_obstacle_on_traj';
path=LoadParameters(name_dat);
load(path);

%ALPHA PARAMETERS
%rbf
number_of_basis = 10;
redundancy = 3;
range = [0 , 12];
precomp_sample = false;
numeric_theta = [8.482084 9.956197 9.872670 2.204167 2.594941 2.931874 4.891161 10.797650 6.943595 9.898512 3.365269 5.274187 5.884780 5.837289 1.804642 2.824112 1.884467 10.742163 10.639953 7.777113];
%constant alpha
value1 = 1*ones(chains.GetNumTasks(1));
values{1} = value1;

%CONTROLLER PARAMETERS
max_time = 50;
combine_rule = {'sum'}; 

% INTEGRATION START CONDITIONS
qi{1} = qz;
qdi{1} = zeros(1,chains.GetNumLinks(1));

%% Reference
% if type_of_task = sampled i have to specify the Time to reach the
% end of the trajectories that is equal to the simulation time
reference = References(target_link,type,control_type,traj,geom_parameters,time_law,time_struct,dim_of_task,type_of_traj);
reference.BuildTrajs();

%% plot scenario
text = LoadScenario('lbr_scenario4');
eval(text);

%% alpha function
alphas = Alpha.RBF.BuildCellArray(chains.GetNumChains(),chains.GetNumTasks(1),time_struct,number_of_basis,redundancy,range,precomp_sample,numeric_theta);       
%alphas = Alpha.ConstantAlpha.BuildCellArray(chains.GetNumChains(),chains.GetNumTasks(1),values,time_struct);

%% Controller
controller = Controllers.UF(chains,reference,alphas,metric,Kp,Kd,combine_rule,max_time);
%% Simulation
tic
[t, q, qd] = DynSim(time_sym_struct,controller,qi,qdi,fixed_step);%,options);
toc
%% Display
bot1.plot(q{1});




