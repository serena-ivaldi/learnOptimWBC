% Main function to perform in Matlab the integration of the mixed forward dynamics
% system (simulation) of the iCub humanoid robot with foot pose corrections (FPC).
%
% With this function the user can define on which foot the robot is in contact with
% the ground. In dependency of the given foot configuration, the function integrates
% the robot's state variable chi which contains following sub-variables:
%
%       x_b:      the Cartesian position of the base (R^3)
%       qt_b:     the the orientation of the base in quaternions (global parametrization of SO(3))
%       q_j:      the joint positions (R^ndof)
%       dx_b:     the Cartesian velocity of the base (R^3)
%       omega_b:  the angular velocity describing the orientation of the base (SO(3))
%       dq_j:     the joint velocities (R^ndof)
%
function [t, q_j, dq_j, stmChi] = mixDynSimICub(controller, params)
    global gbl_plstate;
    if ( isempty(params.fdyn_data) || isempty(gbl_plstate) )
        error('mixDynSimICub: %s', WBM.wbmErrorMsg.EMPTY_DATA_TYPE);
    end
    fdyn_data = params.fdyn_data;
    tstep     = params.sim_step;
    tstart    = params.tStart;
    tend      = params.tEnd;

    tidx_plg = gbl_plstate.tidx_go; % time index payload grabbed
    tidx_plr = gbl_plstate.tidx_ro; % time index payload released

    % argument input arrays for the mixed forward dynamics system:
    % fdyn-model with FPC:
    argin_fpc = {fdyn_data.foot_conf, fdyn_data.ac_f, 'fpc'};
    % fdyn-model with FPC & payload:
    argin_fpcpl = {fdyn_data.fhTotCWrench, fdyn_data.foot_conf, fdyn_data.hand_conf, ...
                   fdyn_data.f_cp, fdyn_data.ac_f, 'fpc'};

    [tsect_list, sidx_pl] = createTimeSections(tidx_plg, tidx_plr, tstep, tstart, tend);
    nTsecs = size(tsect_list,1); % number of time sections

    ws = GetWholeSystem(controller);
    % update the robot position and set the defined reference link ...
    SetWorldFrameiCub(ws, params.qjInit, params.dqjInit, params.dx_bInit, ...
                      params.omega_bInit, params.root_reference_link);
    [~,vqT_b,~,~]  = GetState(ws);
    params.chiInit = vertcat(vqT_b, params.qjInit, ws.dx_b, ws.omega_W, params.dqjInit);

    % ode-options:
    if ~params.demo_movements
        ode_opt = odeset('RelTol', 1e-3, 'AbsTol', 1e-4);
    else
        ode_opt = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
    end

    % define the policy controller for the torques:
    v = []; % dummy variable for the unused arguments ...
    fhTrqControl = @(t, M, c_qv, stp, nu_s, Jc_f, djcdq_f, foot_conf)policyTrqControl(t, v, v, stp, v, Jc_f, v, v, ...
                                                                                      controller, params.numContacts);
    % perform the mixed forward dynamics integration by using at most once
    % payload forces or external forces between the time spans:
    nSteps = (tend - tstart) / tstep;
    len    = ws.hwbm.stvLen;
    Chi    = zeros(nSteps,len);
    t      = zeros(nSteps,1);
    try
        for i = 1:nTsecs
            is = tsect_list{i,2}(1,1); % start index
            ie = tsect_list{i,2}(1,2); % end index
            if (i == sidx_pl)
                % switch to fdyn-model with FPC & payload ...
                [t(is:ie,1), Chi(is:ie,1:len)] = forwardDynamics(ws, tsect_list{i,1}, params.chiInit, controller, ...
                                                                 fhTrqControl, ode_opt, params.maxtime, argin_fpcpl{:});
            else
                % fdyn-model with only FPC ...
                [t(is:ie,1), Chi(is:ie,1:len)] = forwardDynamics(ws, tsect_list{i,1}, params.chiInit, controller, ...
                                                                 fhTrqControl, ode_opt, params.maxtime, argin_fpc{:});
            end
        end
    catch exc
        stderr = 2;
        fprintf(stderr, 'mixDynSimICub: An exception occurred during the integration of the mixed forward dynamics system!\n\n');
        rethrow(exc);
    end
    q_j  = Chi(:,8:ws.ndof+7);

    if (nargout > 2)
        dq_j = Chi(:,ws.ndof+14:len);

        if (nargout == 4)
            stmChi = Chi;
        end
    end
end

function [tsect_list, sidx_pl] = createTimeSections(tidx_plg, tidx_plr, tstep, tstart, tend)
    % check all cases of grab and release a payload object
    % during the given time span:
    if ( (tidx_plg >= 1) && (tidx_plr > tidx_plg) && (tidx_plr <= tend) )
        % case 1: grab & release the object within
        % the time span [t_s, t_plg, t_plr, t_e]:
        if ( (tstart ~= tidx_plg) && (tend ~= tidx_plr) )
            % general case:
            tsect_list = cell(3,2);
            tsect_list{1,1} = tstart:tstep:tidx_plg;
            tsect_list{2,1} = tidx_plg+1:tstep:tidx_plr;
            tsect_list{3,1} = tidx_plr+1:tstep:tend;

            % start & end indices of each time section ...
            tsect_list{1,2} = horzcat(tstart, tidx_plg);
            tsect_list{2,2} = horzcat(tidx_plg+1, tidx_plr);
            tsect_list{3,2} = horzcat(tidx_plr+1, tend);
            % section index where the payload is grabbed ...
            sidx_pl = 2;
        elseif ( (tstart == tidx_plg) && (tend ~= tidx_plr) ) % special cases:
            % object is already grabbed at the start point:
            tsect_list = cell(2,2);
            tsect_list{1,1} = tidx_plg:tstep:tidx_plr;
            tsect_list{2,1} = tidx_plr+1:tstep:tend;

            tsect_list{1,2} = horzcat(tidx_plg, tidx_plr);
            tsect_list{2,2} = horzcat(tidx_plr+1, tend);
            sidx_pl = 1;
        elseif ( (tstart ~= tidx_plg) && (tend == tidx_plr) )
            % object will be released at the end point:
            tsect_list = cell(2,2);
            tsect_list{1,1} = tstart:tstep:tidx_plg;
            tsect_list{2,1} = tidx_plg+1:tstep:tidx_plr;

            tsect_list{1,2} = horzcat(tstart, tidx_plg);
            tsect_list{2,2} = horzcat(tidx_plg+1, tidx_plr);
            sidx_pl = 2;
        else % (tstart = tidx_plg) and (tend = tidx_plr)
            % object is grabbed at the start point and
            % will be released at the end point:
            tsect_list = cell(1,2);
            tsect_list{1,1} = tidx_plg:tstep:tidx_plr;
            tsect_list{1,2} = horzcat(tidx_plg, tidx_plr);
            sidx_pl = 1;
        end
    elseif ( (tidx_plg >= 1) && (tidx_plg <= tend) && ~tidx_plr )
        % case 2: grab the object and do not release it
        % within the given time span [t_s, t_plg, t_e]:
        if ( (tstart ~= tidx_plg) && (tidx_plg < tend) )
            % general case:
            tsect_list = cell(2,2);
            tsect_list{1,1} = tstart:tstep:tidx_plg;
            tsect_list{2,1} = tidx_plg+1:tstep:tend;

            tsect_list{1,2} = horzcat(tstart, tidx_plg);
            tsect_list{2,2} = horzcat(tidx_plg+1, tend);
            sidx_pl = 2;
        elseif (tstart == tidx_plg)
            % object is already grabbed at the start point:
            tsect_list = cell(1,2);
            tsect_list{1,1} = tidx_plg:tstep:tend;
            tsect_list{1,2} = horzcat(tidx_plg, tend);
            sidx_pl = 1;
        else
            % object will be grabbed at the end point:
            tsect_list = cell(1,2);
            tsect_list{1,1} = tstart:tstep:tidx_plg;
            tsect_list{1,2} = horzcat(tstart, tidx_plg);
            sidx_pl = 0;
        end
    elseif (~tidx_plg && ~tidx_plr)
        % case 3: the object will never grabbed within
        % the given time span [t_s, t_e]:
        tsect_list = cell(1,2);
        tsect_list{1,1} = tstart:tstep:tend;
        tsect_list{1,2} = horzcat(tstart, tend);
        sidx_pl = 0;
    else
        error('createTimeSections: %s', WBM.wbmErrorMsg.IDX_OUT_OF_BOUNDS);
    end
end

function [t, stmChi] = forwardDynamics(ws, tspan, stvChi_0, controller, fhTrqControl, ode_opt, maxtime, varargin)
    if isempty(controller.current_time), controller.current_time = tic; end

    % perform a specific forward dynamics integration for a given time span:
    [t, stmChi] = fwdDyn(ws, tspan, stvChi_0, fhTrqControl, ode_opt, varargin{:});

    if (toc(controller.current_time) > maxtime)
        controller.current_time = [];
        error('forwardDynamics: Stopped. Maximum execution time exceeded.');
    end
end

function tau = policyTrqControl(t,~,~,stp,~,Jc_f,~,~,controller, nCtcs)
    fc_0 = zeros(6*nCtcs,1);
    Jc_t = Jc_f.';
    tau  = Policy(controller, t, stp.q_j, stp.dq_j, fc_0, Jc_t);
end
