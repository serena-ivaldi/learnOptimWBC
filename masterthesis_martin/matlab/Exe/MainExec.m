clear variables
close all
clc

% global payload state variable for
% mixed forward dynamics systems:
global gbl_plstate;

%% initialize all the data
optim      = false;
evaluation = true;
sim_tstep  = 0.01;

video         = false;
vid_filename  = 'icub-simulation.avi';
vid_fps       = 80;
step_save_fig = 20;
save_fig      = false;

save_result_single_exec = false;

% configuration file using the interfaces of the WBM-Library:
configuration_file_name = 'config_icub_lift_obj';
%configuration_file_name = 'RP_humanoid_test_iCub_5';
%configuration_file_name = 'RP_humanoid_test_iCub_6';
%configuration_file_name = 'RP_humanoid_test_iCub_7';

[bot1, name_scenario, time_struct, time_sym_struct, simulator_type, reference, ...
 alphas, controller, constr, learn_approach, inst,~,~,~,~,~,input, rawTextFromStorage] = Init(configuration_file_name, optim);

%% Simulation
params = input{1,2};
if strcmp(simulator_type{1,1}, 'icub_matlab')
    if params.mixed_fd_models
        tic
        [t, q, qd, stmChi] = mixDynSimICub(controller, params);
        toc
    else
        tic
        [t, q, qd, stmChi] = DynSim_iCub(controller, params);
        toc
    end
else
    error('Wrong simulator type!');
end

%% Evaluate fitness
if evaluation
    if isfield(params, 'fit_argin')
        fit_argin = horzcat(t, q, params.fit_argin);
    else
        fit_argin = {t, q, qd};
    end
    performance = feval(inst.fitness, inst, fit_argin);

    inst.penalty_handling.ComputeConstraintsViolation(-1);
    %performance = performance - inst.penalty_handling.fitness_penalties(1);
end

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
    if ~isempty(gbl_plstate)
        tidx_go = gbl_plstate.tidx_go;

        if (tidx_go > 0)
            stmPos = getPositionsData(bot1.hwbm, stmChi);
            len    = size(stmPos,1);

            % set the utilization time indices (start, end) of the object:
            setPayloadUtilTime(bot1.sim_config, 1, tidx_go+1, len);

            lnk_traj = bot1.sim_config.trajectories;
            bot1.sim_config.trajectories = setTrajectoriesData(bot1.hwbm, lnk_traj, stmPos, [1; 1], [len; len]);
            bot1.sim_config.show_legend  = true;
        else
            error('The time index when the object is grabbed is not defined!');
        end
    end

    if video
        createVideo(bot1.sim_config, vid_filename, 'r_fast', vid_fps);
        % createVideo(bot1.sim_config, vid_filename, 'r_depth', vid_fps);
    end
    visFwdDyn(bot1, stmChi, sim_tstep);
end
