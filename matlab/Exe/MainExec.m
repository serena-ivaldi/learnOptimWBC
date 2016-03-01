clear variables
close all
clc

%% initialize all the data
[bot1,name_scenario,time_struct,time_sym_struct,reference,alphas,controller,constr,learn_approach,inst,~,~,~,~,qi,qdi,fixed_step,torque_saturation,maxtime,rawTextFromStorage,name_dat]=Init();
%% Simulation
tic
[t, q, qd] = DynSim(time_sym_struct,controller,qi,qdi,fixed_step,'TorqueSat',torque_saturation,'maxtime',maxtime);
toc

%% Evaluate fitness 
evaluation = false;
if (evaluation)
    performance = feval(inst.fitness,inst,t,q);
    inst.penalty_handling.ComputeConstraintsViolation(-1)
    performance = performance - inst.penalty_handling.fitness_penalties(1); 
end

%% produce graph and copy parameters
save_result_single_exec = false;
if save_result_single_exec
   name_folder = 'hand_made_jaco1.3_c';
   complete_path = PlotSingleExecResult(name_folder,q,qd,t,time_sym_struct,controller,time_struct,name_scenario,bot1);
   % copy runtime parameters in the newly created folder
   fileID = fopen(strcat(complete_path,'/','runtime_parameters.txt'),'w');
   fprintf(fileID,'%s',rawTextFromStorage);
   complete_path_to_file = strcat(complete_path,'/data.mat');
   save(complete_path_to_file) 
end
%% Display
fps = 200;
video = false;
step_save_fig = 20;
save_fig = false;
ee_trajectory = true; 
elbow_traj    = true;
%---

% plot the trajectory of the elbow or e-e or both
if(ee_trajectory || elbow_traj)
   figure; hold on;
   text = LoadScenario(name_scenario);
   eval(text);
   [ee,elbow] = ComputePositions(q{1},t,controller);
   ee = ee';
   elbow = elbow';
   izy = 1;
   handle_vector = [];
   if(ee_trajectory)
      name_of_trace {1,izy} = 'end-effector';  
      handle1 = plot3(ee(:,1)',ee(:,2)',ee(:,3)','Color','r','LineWidth',2);
      handle_vector=[handle_vector,handle1];
      izy = izy + 1;
   end
   if(elbow_traj)
      name_of_trace{1,izy} = 'elbow';  
      handle2 = plot3(elbow(:,1)',elbow(:,2)',elbow(:,3)','Color','g','LineWidth',2);
      handle_vector=[handle_vector,handle2];
      izy = izy + 1;
   end
   hold on;
   hL = legend(handle_vector,name_of_trace);
   set(hL,'FontSize',15);
end


if(~video && ~save_fig)
   zoom =  5.0698;
   set(gca,'CameraViewAngle',zoom);
   camera_position = [14.3762    9.7004   15.0093];
   campos(camera_position)
   bot1.plot(q{1},'fps',fps);
elseif(video)
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
elseif(save_fig)
   camera_position = [-7.5371   -1.1569   21.1612];
   zoom =  2.4702;it
   set(gca,'CameraViewAngle',zoom);
   campos(camera_position)
   SaveFigures(bot1,q{1},step_save_fig)
end
