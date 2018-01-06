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

    tspan = tstart:tstep:tend; % total time span of the simulation

    tidx_plg = gbl_plstate.tidx_go; % time index when payload is grabbed
    tidx_plr = gbl_plstate.tidx_ro; % time index when payload is released

    % set the argument input arrays and the corresponding torque controller
    % for the mixed forward dynamics system:
    v = []; % dummy variable for the unused arguments ...
    if ~params.active_floating_base
        if (~params.feet_on_ground(1,1) || ~params.feet_on_ground(1,2))
            error('mixDynSimICub: Both feet must be set on the ground (as fixed base), else the torque controller will fail.');
        end
        % fdyn-model with NFB (no floating base):
        argin_fd = {'nfb'};
        % fdyn-model with NFB & payload:
        argin_fdpl = {fdyn_data.fhTotCWrench, fdyn_data.hand_conf, fdyn_data.f_cp, 'nfb'};

        % policy torque controller:
        fhTrqControl = @(t, M, c_qv, stp)policyTrqControl(t, v, v, stp, controller, ...
                                                          params.numContacts, params.torque_saturation);
    else
        % fdyn-model with FPC:
        argin_fd = {fdyn_data.foot_conf, fdyn_data.ac_f, 'fpc'};
        % fdyn-model with FPC & payload:
        argin_fdpl = {fdyn_data.fhTotCWrench, fdyn_data.foot_conf, fdyn_data.hand_conf, ...
                      fdyn_data.f_cp, fdyn_data.ac_f, 'fpc'};

        % extended policy torque controller:
        fhTrqControl = @(t, M, c_qv, stp, nu_s, Jc_f, djcdq_f, foot_conf)policyTrqControlExt(t, v, v, stp, v, Jc_f, v, v, controller, ...
                                                                                             params.numContacts, params.torque_saturation);
    end

    [tsect_list, sidx_pl] = createTimeSections(tidx_plg, tidx_plr, tspan);
    ntsecs = size(tsect_list,1); % number of time sections

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

    % perform the mixed forward dynamics integration by using at most once
    % payload forces or external forces between the time spans:
    nSteps = size(tspan,2); %(tend - tstart) / tstep;
    len    = ws.hwbm.stvLen;
    Chi    = zeros(nSteps,len);
    t      = zeros(nSteps,1);
    try
        for i = 1:ntsecs
            is = tsect_list{i,2}(1,1); % start index
            ie = tsect_list{i,2}(1,2); % end index
            if (i == sidx_pl) % check section index
                % switch to fdyn-model with NFB or FPC + payload ...
                [t(is:ie,1), Chi(is:ie,1:len)] = forwardDynamics(ws, tsect_list{i,1}, params.chiInit, controller, ...
                                                                 fhTrqControl, ode_opt, params.maxtime, argin_fdpl{:});
            else
                % fdyn-model with only NFB or FPC ...
                [t(is:ie,1), Chi(is:ie,1:len)] = forwardDynamics(ws, tsect_list{i,1}, params.chiInit, controller, ...
                                                                 fhTrqControl, ode_opt, params.maxtime, argin_fd{:});
            end
        end
    catch exc
        stderr = 2;
        fprintf(stderr, 'mixDynSimICub: An exception occurred during the integration of the mixed forward dynamics system!\n\n');
        rethrow(exc);
    end
    q_j = Chi(:,1:(ws.ndof+7)); % q_j with VQ-transformation

    if (nargout > 2)
        dq_j = Chi(:,(ws.ndof+8):len); % dq_j with floating base

        if (nargout == 4)
            stmChi = Chi;
        end
    end
end

function [tsect_list, sidx_pl] = createTimeSections(tidx_plg, tidx_plr, tspan)
    tidx_s = 1;             % start time index
    tidx_e = size(tspan,2); % end time index

    % check all cases of grab and release a payload object
    % during the given time span:
    if ( (tidx_plg >= tidx_s) && (tidx_plr > tidx_plg) && (tidx_plr <= tidx_e) )
        % case 1: grab & release the object within
        % the time span [ti_s, ti_plg, ti_plr, ti_e]:
        if ( (tidx_s ~= tidx_plg) && (tidx_e ~= tidx_plr) )
            % general case:
            tsect_list = cell(3,2);
            tsect_list{1,1} = tspan(1,tidx_s:tidx_plg);
            tsect_list{2,1} = tspan(1,(tidx_plg+1):tidx_plr);
            tsect_list{3,1} = tspan(1,(tidx_plr+1):tidx_e);

            % start & end indices of each time section ...
            tsect_list{1,2} = horzcat(tidx_s, tidx_plg);
            tsect_list{2,2} = horzcat(tidx_plg+1, tidx_plr);
            tsect_list{3,2} = horzcat(tidx_plr+1, tidx_e);
            % section index where the payload is grabbed ...
            sidx_pl = 2;
        elseif ( (tidx_s == tidx_plg) && (tidx_e ~= tidx_plr) ) % special cases:
            % object is already grabbed at the start point:
            tsect_list = cell(2,2);
            tsect_list{1,1} = tspan(1,tidx_plg:tidx_plr);
            tsect_list{2,1} = tspan(1,(tidx_plr+1):tidx_e);

            tsect_list{1,2} = horzcat(tidx_plg, tidx_plr);
            tsect_list{2,2} = horzcat(tidx_plr+1, tidx_e);
            sidx_pl = 1;
        elseif ( (tidx_s ~= tidx_plg) && (tidx_e == tidx_plr) )
            % object will be released at the end point:
            tsect_list = cell(2,2);
            tsect_list{1,1} = tspan(1,tidx_s:tidx_plg);
            tsect_list{2,1} = tspan(1,(tidx_plg+1):tidx_plr);

            tsect_list{1,2} = horzcat(tidx_s, tidx_plg);
            tsect_list{2,2} = horzcat(tidx_plg+1, tidx_plr);
            sidx_pl = 2;
        else % (tidx_s = tidx_plg) and (tidx_e = tidx_plr)
            % object is grabbed at the start point and
            % will be released at the end point:
            tsect_list = cell(1,2);
            tsect_list{1,1} = tspan(1,tidx_plg:tidx_plr);
            tsect_list{1,2} = horzcat(tidx_plg, tidx_plr);
            sidx_pl = 1;
        end
    elseif ( (tidx_plg >= tidx_s) && (tidx_plg <= tidx_e) && ~tidx_plr )
        % case 2: grab the object and do not release it
        % within the given time span [ti_s, ti_plg, ti_e]:
        if ( (tidx_s ~= tidx_plg) && (tidx_plg < tidx_e) )
            % general case:
            tsect_list = cell(2,2);
            tsect_list{1,1} = tspan(1,tidx_s:tidx_plg);
            tsect_list{2,1} = tspan(1,(tidx_plg+1):tidx_e);

            tsect_list{1,2} = horzcat(tidx_s, tidx_plg);
            tsect_list{2,2} = horzcat(tidx_plg+1, tidx_e);
            sidx_pl = 2;
        elseif (tidx_s == tidx_plg)
            % object is already grabbed at the start point:
            tsect_list = cell(1,2);
            tsect_list{1,1} = tspan(tidx_plg:tidx_e);
            tsect_list{1,2} = horzcat(tidx_plg, tidx_e);
            sidx_pl = 1;
        else
            % object will be grabbed at the end point:
            tsect_list = cell(1,2);
            tsect_list{1,1} = tspan(tidx_s:tidx_plg);
            tsect_list{1,2} = horzcat(tidx_s, tidx_plg);
            sidx_pl = 0;
        end
    elseif (~tidx_plg && ~tidx_plr)
        % case 3: the object will never grabbed within
        % the given time span [ti_s, ti_e]:
        tsect_list = cell(1,2);
        tsect_list{1,1} = tspan;
        tsect_list{1,2} = horzcat(tidx_s, tidx_e);
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

% policy torque controller for the fdyn-model without floating base:
function tau = policyTrqControl(t,~,~,stp, controller, nCtcs, trq_sat)
    ws  = GetWholeSystem(controller);

    stFltb = getBaseState(ws);
    Jc_lf  = jacob(ws, 'l_sole', stp.q_j, stFltb);
    Jc_rf  = jacob(ws, 'r_sole', stp.q_j, stFltb);
    Jc_f   = vertcat(Jc_lf, Jc_rf);
    Jc_t   = Jc_f.';

    fc_0 = zeros(6*nCtcs,1);
    tau  = Policy(controller, t, stp.q_j, stp.dq_j, fc_0, Jc_t);

    % apply torque saturation (to prevent overload) ...
    tau = WBM.utilities.mbd.satTrq(tau, trq_sat);
end

% extended policy torque controller for the fdyn-model with pose correction:
function tau = policyTrqControlExt(t,~,~,stp,~,Jc_f,~,~,controller, nCtcs, trq_sat)
    fc_0 = zeros(6*nCtcs,1);
    Jc_t = Jc_f.';
    tau  = Policy(controller, t, stp.q_j, stp.dq_j, fc_0, Jc_t);
    tau  = WBM.utilities.mbd.satTrq(tau, trq_sat);
end
