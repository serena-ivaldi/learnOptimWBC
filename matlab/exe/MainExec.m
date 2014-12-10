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
time_sym_struct.step = 0.001; 
fixed_step = false;

% TASK PARAMETERS
name_dat='LBR4p5__scene3_ee_tracking_circ_obstacle_on_traj_2_task_fit4';
path=LoadParameters(name_dat);
load(path);

%ALPHA PARAMETERS
%rbf
number_of_basis = 10;
redundancy = 3;
range = [0 , 12];
precomp_sample = false;
numeric_theta = [11.762906 11.477649 10.034511 11.971192 2.850863 6.483845 11.097379 11.066734 10.986656 7.900689 0.106901 0.137403 0.208355 0.592537 5.298428 0.051837 0.128208 0.136004 0.139453 0.085803 ];
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
text = LoadScenario('lbr_scenario3');
eval(text);

%% alpha function
alphas = Alpha.RBF.BuildCellArray(chains.GetNumChains(),chains.GetNumTasks(1),time_struct,number_of_basis,redundancy,range,precomp_sample,numeric_theta);       
%alphas = Alpha.ConstantAlpha.BuildCellArray(chains.GetNumChains(),chains.GetNumTasks(1),values,time_struct);

%% Controller
controller = Controllers.UF(chains,reference,alphas,[],metric,Kp,Kd,combine_rule,max_time);
%% Simulation
tic
[t, q, qd] = DynSim(time_sym_struct,controller,qi,qdi,fixed_step);%,options);
toc
%% Display
fps = 200;
video = false;

if(~video)
   bot1.plot(q{1},'fps',fps);
else
   %at the end of the video simulation after chosing a good camera pos and
   %zoom
   % to see camera position call "campos" on the shell 
   % to see zoom call "get(gca,'CameraViewAngle')"
   allpath = which('FindData.m');
   path = fileparts(allpath);
   path = strcat(path,'/video');
   camera_position = [-7.5371   -1.1569   21.1612];
   zoom =  2.4702;
   set(gca,'CameraViewAngle',zoom);
   campos(camera_position)
   bot1.plot(q{1},'movie',path);
end




