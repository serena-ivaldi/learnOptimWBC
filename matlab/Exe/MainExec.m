clear variables
close all
clc

%% initialize all the data
optim = false;
configuration_file_name = 'RP_humanoid_bench_lbrsimple';
[bot1,name_scenario,time_struct,time_sym_struct,simulator_type,reference,alphas,controller,constr,learn_approach,inst,~,~,~,~,~,input,rawTextFromStorage,name_dat]=Init(configuration_file_name,optim);
%% Simulation
if(strcmp(simulator_type{1},'rbt'))
    tic
    [t, q, qd] = DynSim(time_sym_struct,controller,input{2},input{3},input{6},'TorqueSat',input{7},'maxtime',input{8});
    toc
elseif (strcmp(simulator_type{1},'icub_matlab'))
    tic
    [t, q, qd]=DynSim_iCub(controller,input{2});
    toc
end
%% Evaluate fitness 
evaluation = true;
if (evaluation)
    output{1} = t;
    output{2} = q;
    output{3} = qd;
    performance = feval(inst.fitness,inst,output);
    inst.penalty_handling.ComputeConstraintsViolation(-1)
    %performance = performance - inst.penalty_handling.fitness_penalties(1); 
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
fps = 1000;
video = false;
step_save_fig = 20;
save_fig = false;
ee_trajectory = true; 
elbow_traj    = true;
cur_bot = controller.subchains.sub_chains{1};
%---

% plot the trajectory of the elbow or e-e or both
if(ee_trajectory || elbow_traj)
   %figure;
   hold on;
    %text = LoadScenario(name_scenario);
    %eval(text);
   
   if ~(isa(cur_bot,'DummyRvc_iCub'))
    [ee,elbow] = ComputePositions(q{1},t,controller);
   else
    [ee,elbow] = ComputePositionsIcub(q,t,controller);
   end
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
   if ~(isa(cur_bot,'DummyRvc_iCub'))
       bot1.plot(q{1},'fps',fps);
   else
       bot1.plot(q,input{2});
   end
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
