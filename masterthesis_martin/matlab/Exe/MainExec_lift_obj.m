clear variables
close all
clc

% Global payload state variable for a mixed forward dynamics system
% to indicate the switchover to another forward dynamics model:
global gbl_plstate;
gbl_plstate = struct('obj_grabbed', false, 'obj_released', false, ...
                     'tidx_go', 0, 'tidx_ro', 0, 'qj_go', [], 'fval_go', []);

%% initialize all the data
optim      = false;
evaluation = true;
sim_tstep  = 0.01;

video         = false;
vid_filename  = 'iCub_lift_obj.avi';
vid_fps       = 80;
step_save_fig = 20;
save_fig      = false;

save_result_single_exec = false;

% configuration files using the interfaces of the WBM-Library:
nSteps = 2;
configuration_file_name = cell(nSteps,1);
configuration_file_name{1,1} = 'config_icub_lift_obj_part1';
configuration_file_name{2,1} = 'config_icub_lift_obj_part2';

int_results = cell(nSteps,1); % integration results
performance = cell(nSteps,1);

for i = 1:nSteps
    config_file = configuration_file_name{i,1};
    [bot1, name_scenario, time_struct, time_sym_struct, simulator_type, reference, ...
     alphas, controller, constr, learn_approach, inst,~,~,~,~,~,input, rawTextFromStorage] = Init(config_file, optim);

    if ~strcmp(simulator_type{1,1}, 'icub_matlab')
        error('Wrong simulator type!');
    end
    params = input{1,2};

    % Simulation:
    if params.mixed_fd_models
        tic
        [t, q, qd, stmChi] = mixDynSimICub(controller, params);
        toc
    else
        tic
        [t, q, qd, stmChi] = DynSim_iCub(controller, params);
        toc
    end
    int_results{i,1} = {q, qd, stmChi};

    % Evaluate fitness:
    if evaluation
        fit_argin        = horzcat(t, q, params.fit_argin);
        performance{i,1} = feval(inst.fitness, inst, fit_argin);

        inst.penalty_handling.ComputeConstraintsViolation(-1);
        % performance = performance - inst.penalty_handling.fitness_penalties(1);
    end
end

% concatenate the results of all steps ...
tidx_go = gbl_plstate.tidx_go;
if ~tidx_go
    error('The time index when the object is grabbed is not defined!');
end
len = size(t,1);
ip1 = tidx_go;
ip2 = len - tidx_go;

q1 = int_results{1,1}{1,1};
q2 = int_results{2,1}{1,1};
q  = vertcat(q1(1:ip1,:), q2(1:ip2,:));

qd1 = int_results{1,1}{1,2};
qd2 = int_results{2,1}{1,2};
qd  = vertcat(qd1(1:ip1,:), qd2(1:ip2,:));

chi1   = int_results{1,1}{1,3};
chi2   = int_results{2,1}{1,3};
stmChi = vertcat(chi1(1:ip1,:), chi2(1:ip2,:));

%% produce graph and copy parameters
if save_result_single_exec
    name_folder   = 'hand_made_jaco1.3_c';
    complete_path = PlotSingleExecResult(name_folder, q, qd, t, time_sym_struct, ...
                                         controller, time_struct, name_scenario, bot1);
    % copy runtime parameters in the newly created folder
    fileID = fopen(strcat(complete_path, '/', 'runtime_parameters.txt'), 'w');
    fprintf(fileID, '%s', rawTextFromStorage);
    complete_path_to_file = strcat(complete_path, '/data.mat');
    save(complete_path_to_file)
end

%% Display
if save_fig
    % camera_position = [-7.5371   -1.1569   21.1612];
    % zoom =  2.4702;
    % set(gca,'CameraViewAngle',zoom);
    % campos(camera_position)
    SaveFigures(bot1, q{1,1}, step_save_fig);
else
    stmPos = getPositionsData(bot1.hwbm, stmChi);

    % set the utilization time indices (start, end) of the object:
    setPayloadUtilTime(bot1.sim_config, 1, tidx_go+1, len);

    lnk_traj = bot1.sim_config.trajectories;
    bot1.sim_config.trajectories = setTrajectoriesData(bot1.hwbm, lnk_traj, stmPos, [1; 1], [len; len]);
    bot1.sim_config.show_legend  = true;

    if video
        createVideo(bot1.sim_config, vid_filename, 'r_fast', vid_fps);
        % createVideo(bot1.sim_config, vid_filename, 'r_depth', vid_fps);
    end
    visFwdDyn(bot1, stmChi, sim_tstep);
end
