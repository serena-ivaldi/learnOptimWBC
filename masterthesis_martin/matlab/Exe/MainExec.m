clear variables
close all
clc

%% initialize all the data
optim = false;

% initialize the global payload state variable for a mixed
% forward dynamics system to indicate the switchover to an
% other forward dynamics model:
global gbl_plstate;
gbl_plstate = struct('obj_grabbed', false, 'obj_released', false, ...
                     'tidx_go', 0, 'tidx_ro', 0); % time indices grab/release obj.

% configuration file using the "improvised" interface with the WBM-Toolbox:
configuration_file_name = 'config_icub_lift_obj';
%configuration_file_name = 'RP_humanoid_test_iCub_5';
%configuration_file_name = 'RP_humanoid_test_iCub_6';
%configuration_file_name = 'RP_humanoid_test_iCub_7';

[bot1,name_scenario,time_struct,time_sym_struct,simulator_type,reference,alphas,controller,constr,learn_approach,inst,~,~,~,~,~,input,rawTextFromStorage]=Init(configuration_file_name,optim);

%% Simulation
if(strcmp(simulator_type{1},'rbt'))
    tic
    [t, q, qd] = DynSim(time_sym_struct,controller,input{2},input{3},input{6},'TorqueSat',input{7},'maxtime',input{8});
    toc
elseif (strcmp(simulator_type{1},'icub_matlab'))
    input{2}.mixed_fd_models = true;

    if input{2}.mixed_fd_models

        % input arguments for the experiment ...
        foot_contact = [true, true];
        hand_contact = [true, true];
        f_cp = [0; 1.2; 1.2];
        lnk_R_cp = eye(3,3);
        lnk_p_cp = zeros(3,1);
        mu_s = 1; %0.5;

        input{2}.fdyn_data = createFDynData(bot1, foot_contact, hand_contact, f_cp, lnk_R_cp, lnk_p_cp, mu_s);

        tic
        [t, q, qd, stmChi] = mixDynSimICub(controller, input{2});
        toc
    else
        tic
        [t, q, qd, stmChi] = DynSim_iCub(controller, input{2});
        toc
    end
end

%% Evaluate fitness
evaluation = true;
if (evaluation)

    % constraint links for the fitness function:
    % link order: [<links of the right arm>, <head link>, <links of the left arm>].
    % arm link order: bottom up, [x_gripper/x_hand, ..., x_shoulder_1], x ... r/l.
    cstr_lnk_names = { 'r_gripper', 'r_hand', 'r_wrist_1', 'r_elbow_1', 'r_shoulder_1', 'head', ...
                       'l_gripper', 'l_hand', 'l_wrist_1', 'l_elbow_1', 'l_shoulder_1' };

    % cstr_lnk_names = { 'r_gripper', 'r_wrist_1', 'r_elbow_1', 'r_shoulder_1', 'head', ...
    %                    'l_gripper', 'l_wrist_1', 'l_elbow_1', 'l_shoulder_1' }; % link list for fitnessHumanoidsICub5

    % task indices of the goal point positions of the end-effector tasks:
    % note: the indices are in the same order as the elementary tasks.
    tidx_gp = vertcat(1,3,6,7);
    % data input for the fitness function ...
    %input{2}.fit_argin = {cstr_lnk_names, tidx_gp}; % not here ...

    fit_argin   = {t, q, cstr_lnk_names, tidx_gp};
    performance = feval(inst.fitness, inst, fit_argin);

    % output{1} = t;
    % output{2} = q;
    % output{3} = qd;
    % performance = feval(inst.fitness,inst,output);

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
fps = 100;
video = false;
step_save_fig = 20;
save_fig = false;
ee_trajectory = false;
elbow_traj    = false;
cur_bot = controller.subchains.sub_chains{1};
%---

% plot the trajectory of the elbow or e-e or both
if(ee_trajectory || elbow_traj)
    %figure;
    hold on;
    %text = LoadScenario(name_scenario);
    %eval(text);
    if ( ~isa(cur_bot, 'DummyRvc_iCub') && ~isa(cur_bot, 'WBM.Interfaces.IMultChainTree') )
        [ee,elbow] = ComputePositions(q{1},t,controller);
        ee = ee';
        elbow = elbow';
        izy = 1;
        handle_vector = [];
        if(ee_trajectory)
            name_of_trace {1,izy} = 'hand trajectory';
            handle1 = plot3(ee(:,1)',ee(:,2)',ee(:,3)','Color','r','LineWidth',2);
            handle_vector=[handle_vector,handle1];
            izy = izy + 1;
        end
        if(elbow_traj)
            name_of_trace{1,izy} = 'elbow trajectory';
            handle2 = plot3(elbow(:,1)',elbow(:,2)',elbow(:,3)','Color','b','LineWidth',2);
            handle_vector=[handle_vector,handle2];
            izy = izy + 1;
        end
    else
        tags = {'r_wrist_1','r_elbow_1','l_wrist_1','l_elbow_1'}; %names of the joints whose trajectory are ploted
        names = {'right hand','right elbow','left hand','left elbow'}; %names to display in the legend
        colors = {'r','c','r','c'}; %colors of the trajectories
        style = {'-','-',':',':'}; %style of the trajectories
        output = ComputePositionsIcub(q,t,controller,tags);
        handle_vector = [];
        for izy = 1:length(tags)
            name_of_trace{1,izy} = names{izy};
            vect = output(:,(1 + (izy-1)*size(t,1)):(izy*size(t,1)))';
            newhandle = plot3(vect(:,1)',vect(:,2)',vect(:,3)','Color',colors{izy},'LineWidth',2,'LineStyle',style{izy});
            handle_vector = [handle_vector, newhandle];
        end
    end
    hold on;
    hL = legend(handle_vector,name_of_trace,'Location','southeast');
    set(hL,'FontSize',15);
end


if(~video && ~save_fig)
    % zoom =  4.5;
    % set(gca,'CameraViewAngle',zoom);
    % camera_position = [7.9387   -2.8753    8.3434];
    % campos(camera_position)
    if ( ~isa(cur_bot, 'DummyRvc_iCub') && ~isa(cur_bot, 'WBM.Interfaces.IMultChainTree') )
        bot1.plot(q{1},'fps',fps);
    else
        % bot1.plot(q,input{2});

        %plot with the activation function
        %names_of_subplot = {'Right arm tasks','Posture task','Left arm tasks'};
        %grouping = {[1, 2], [5], [3, 4]};
        %colors = {{'r', 'c'}, {'default'}, {'r', 'c'}}; %'default' if no chnage in the color
        %legends = {{'elbow','hand'}, {'none'} ,{'elbow','hand'}}; %'none' if you dont want any legend
        %bot1.plotWActivation(q,input{2},alphas,names_of_subplot,grouping,colors,legends);

        %slowmode plot with CoP visualisation
        %fc = controller.visual_param.fc;
        %bot1.plot(q,input{2},'slowmode',fc);

        sim_tstep  = 0.01;
        urdf_name  = bot1.robot_params.model.urdf_robot_name;
        bot1.sim_config = WBM.RobotModel.iCub_arms_torso_free.initSimConfigICub_atf(urdf_name, 'DarkScn');
        visFwdDyn(bot1, stmChi, sim_tstep);
    end
elseif(video)
    %at the end of the video simulation after chosing a good camera pos and
    %zoom
    % to see camera position call "campos" on the shell
    % to see zoom call "get(gca,'CameraViewAngle')" on the shell
    allpath = which('FindData.m');
    path = fileparts(allpath);
    path = strcat(path,'/video');
    zoom =  4.5;
    set(gca,'CameraViewAngle',zoom);
    camera_position = [7.9387   -2.8753    8.3434];
    campos(camera_position)
    if ( ~isa(cur_bot, 'DummyRvc_iCub') && ~isa(cur_bot, 'WBM.Interfaces.IMultChainTree') )
        bot1.plot(q{1},'movie',path);
    else
        %standard plot
        %bot1.plot(q,input{2},'movie',path);

        %plot with the activation function
        names_of_subplot = {'Right arm tasks','Posture task','Left arm tasks'};
        grouping = {[1, 2], [5], [3, 4]};
        colors = {{'r', 'c'}, {'default'}, {'r', 'c'}}; %'default' if no chnage in the color
        legends = {{'elbow','hand'}, {'none'} ,{'elbow','hand'}}; %'none' if you dont want any legend
        bot1.plotWActivation(q,input{2},alphas,names_of_subplot,grouping,colors,legends,'movie',path);
    end
elseif(save_fig)
    camera_position = [-7.5371   -1.1569   21.1612];
    zoom =  2.4702;
    set(gca,'CameraViewAngle',zoom);
    campos(camera_position)
    SaveFigures(bot1,q{1},step_save_fig)
end
