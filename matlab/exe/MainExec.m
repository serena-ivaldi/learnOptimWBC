clear variables
close all
clc

RuntimeVariable

%% Reference
% if type_of_task = sampled i have to specify the Time to reach the
% end of the trajectories that is equal to the simulation time
reference = References(target_link,type,control_type,traj,geom_parameters,time_law,time_struct,dim_of_task,type_of_traj);
reference.BuildTrajs();

%% plot scenario
text = LoadScenario(name_scenario);
eval(text);

%% repellers
repellers = Repellers(chain_dof,rep_target_link,rep_type,rep_mask,rep_type_of_J_rep,rep_obstacle_ref,single_alpha,type_of_rep_strct); 
%% alpha function

% TODO generalize to multichain and generalize respect of different controller
if(strcmp(combine_rule,'sum'))
    number_of_action = chains.GetNumTasks(1);
elseif(strcmp(combine_rule,'projector'))
    number_of_action = chains.GetNumTasks(1) + repellers.GetNumberOfWeightFuncRep(1);
end
%%--- 

alphas = Alpha.RBF.BuildCellArray(chains.GetNumChains(),number_of_action,time_struct,number_of_basis,redundancy,range,precomp_sample,numeric_theta);       
%alphas = Alpha.ConstantAlpha.BuildCellArray(chains.GetNumChains(),chains.GetNumTasks(1),values,time_struct);

%% Controller
controller = Controllers.UF(chains,reference,alphas,repellers,metric,Kp,Kd,combine_rule,regularizer,max_time);
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
   % to see zoom call "get(gca,'CameraViewAngle')" on the shell
   allpath = which('FindData.m');
   path = fileparts(allpath);
   path = strcat(path,'/video');
   camera_position = [-7.5371   -1.1569   21.1612];
   zoom =  2.4702;
   set(gca,'CameraViewAngle',zoom);
   campos(camera_position)
   bot1.plot(q{1},'movie',path);
end




