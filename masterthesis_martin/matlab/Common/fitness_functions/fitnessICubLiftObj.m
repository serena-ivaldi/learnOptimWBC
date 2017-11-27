% Fitness function for the two arms of the iCub humanoid robot.
% The aim of this fitness function is to evaluate the behavior for grabbing and
% lifting an volume body object (box) to a specified destination.
%
% During the lifting process of the grabbed object, the end-effectors (hands)
% have to reach several intermediate goal positions with certain active fixed
% distance constraints for keeping the object. The ambition is to reach these
% goal points before a fixed time limit is expired. The target points will be
% reached with minimal torque values under the common robot constraints, the
% joints and torques limits.
%
function [fval, tidx_pl] = fitnessICubLiftObj(obj, vargin)
    ilen = size(vargin,2);
    switch ilen
        case {4, 5}
            t              = vargin{1,1};
            q_j            = vargin{1,2};
            cstr_lnk_names = vargin{1,3};
            tidx_gp        = vargin{1,4};

            if ~iscellstr(cstr_lnk_names)
                error('fitnessICubLiftObj: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            cstr_lnk_names = cstr_lnk_names(:); % make a column array ...
            tidx_gp = tidx_gp(:);
            noi     = size(t,1);

            if (ilen == 5)
                fset = vargin{1,5};
                if ~isstruct(fset)
                    error('fitnessICubLiftObj: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
                end
                if ( (fset.tlim < 1) || (fset.tlim > noi) )
                    error('fitnessICubLiftObj: %s', WBM.wbmErrorMsg.IDX_OUT_OF_BOUNDS);
                end
            else
                % use the default fitness settings ...
                fset.smp_rate        = 10;    % sample rate
                fset.tlim            = 10;    % time limit (in seconds)
                fset.eps             = 1e-3;  % tolerance value epsilon
                fset.intrpl_step     = 0.001; % interpolation step
                fset.max_effort      = 3.5e+5;
                fset.max_traj_err    = 250;
                fset.weight_effort   = 1;
                fset.weight_traj_err = 3;
            end
        otherwise
            error('fitnessICubLiftObj: %s', WBM.wbmErrorMsg.WRONG_ARR_SIZE);
    end
    vlen = size(tidx_gp,1);
    if ( (vlen < 10) || ~mod(vlen,2) )
        error('fitnessICubLiftObj: %s', WBM.wbmErrorMsg.WRONG_VEC_LEN);
    end
    noi   = size(t,1); % number of iterations
    nLnks = size(cstr_lnk_names,1);

    % get the controller (UF or GHC) of the system ...
    control = obj.input_4_run{1,4};
    % setup the parameters for the fitness functions ...
    fprms   = struct('control', control, 'icub_wbc', GetWholeSystem(control), 'ndof', icub_wbc.ndof, ...
                     'nLnks', nLnks, 'nCstrs', 4*ndof+nLnks, 'cstr_idx', 1, ...
                     'idx_ee', 0, 'idx_gr', 0, 'fix_dist', obj.input_4_run{1,5});

    % get the goal positions for the end-effector and gripper tasks
    % which are defined in the specified subchains of the first chain:
    goal_pos = repmat(struct('ee_r', [], 'ee_l', []), 2 ,1);
    goal_pos(1,1).ee_r = control.references.GetTraj(1, tidx_gp(1,1), 1); % end-effector tasks (hands)
    goal_pos(1,1).ee_l = control.references.GetTraj(1, tidx_gp(2,1), 1);

    % Evaluate grabbing object:
    [fval_go, tidx_go, tau_u, fprms] = fitnessGrabObj(obj, noi, t, q_j, cstr_lnk_names, goal_pos(1,1), fprms, fset);

    if (fval_go ~= -1)
        % Evaluate moving object to destination:
        % set the goal positions for the end-effectors (ee) ...
        % intermediate goal points:
        goal_pos(1,1).ee_r = control.references.GetTraj(1, tidx_gp(3,1), 1);
        goal_pos(1,1).ee_l = control.references.GetTraj(1, tidx_gp(4,1), 1);
        % final goal positions (target points):
        goal_pos(2,1).ee_r = control.references.GetTraj(1, tidx_gp(5,1), 1);
        goal_pos(2,1).ee_l = control.references.GetTraj(1, tidx_gp(6,1), 1);

        [fval_mo, tidx_mo, cstr_idx] = fitnessMoveObj(obj, noi, t, q_j, tau_u, cstr_lnk_names, tidx_go, goal_pos, fprms, fset);
        fprms.cstr_idx   = cstr_idx;

        % Evaluate releasing object at destination:
        % intermediate ee-targets:
        goal_pos(1,1).ee_r = control.references.GetTraj(1, tidx_gp(7,1), 1);
        goal_pos(1,1).ee_l = control.references.GetTraj(1, tidx_gp(8,1), 1);
        % final end positions for the ee:
        goal_pos(2,1).ee_r = control.references.GetTraj(1, tidx_gp(9,1), 1);
        goal_pos(2,1).ee_l = control.references.GetTraj(1, tidx_gp(10,1), 1);

        [fval_ro,~] = fitnessReleaseObj(obj, icub_wbc, t, q_j, tau_u, cstr_lnk_names, cstr_idx, idx_ee, tidx_mo, goal_pos, fset);

        % compute the average fitness value off all steps for lifting the object ...
        fval = (fval_go + fval_mo + fval_ro) / 3;
        return
    end
    % else ...
    fval = fval_go;

    if (nargout == 2)
        % set the time indices when the payload is grabbed or released ...
        tidx_pl = struct('grabbed', tidx_go, 'released', tidx_ro);
    end
end
%% END of fitnessICubLiftObj.


%% FITNESS FUNCTIONS, LINK POSITIONS, CONSTRAINT VALUE LIST & PAYLOAD STATE:

function [fval_go, tidx_go, tau_u, fprms] = fitnessGrabObj(obj, noi, t, q_j, cstr_lnk_names, goal_pos, fprms, fset)
    control  = fprms.control;
    icub_wbc = fprms.icub_wbc;
    ndof     = fprms.ndof;
    nLnks    = fprms.nLnks;
    nDCs     = 1;
    nCstrs   = fprms.nCstrs + nDCs;

    % get index positions of the hand and gripper links:
    idx_ee = find(ismember(cstr_lnk_names, {'r_hand', 'l_hand'}), 2);       % end-effectors (ee)
    idx_gr = find(ismember(cstr_lnk_names, {'r_gripper', 'l_gripper'}), 2); % front parts of the hands
    if ( isempty(idx_gr) || isempty(idx_ee) )
        error('fitnessGrabObj: %s', WBM.wbmErrorMsg.LNK_NOT_IN_LIST);
    end
    fprms.idx_ee = idx_ee;
    fprms.idx_gr = idx_gr;

    % uniform the torques tau w.r.t. the number of degrees of freedom (ndof):
    actvty_time = obj.input_4_run{1,3};
    tau_u       = InterpTorque(control, actvty_time, fset.intrpl_step);

    % initialize the distance constraint structure ...
    dist_cstr = struct('active', false, 'epsilon', fset.eps, 'p1', [], 'p2', []);
    fix_dist  = fprms.fix_dist; % fixed distance for the distance constraint.

    % initialize the variables for the constraint values ...
    lnk_pos   = zeros(3,nLnks); % positions of the constraint links
    cstr_vals = cell(1,nCstrs); % constraint value list
    cstr_idx  = fprms.cstr_idx; % constraint index (counter)
    failure  = false;
    traj_err = 0;
    tidx_go  = 0;

    % compute the overall trajectory error until
    % the object is reached and grabbed:
    obj_grabbed = false;
    vlen = size(q_j,2);
    for i = 1:fset.smp_rate:noi
        ti  = t(i,1);
        qj  = q_j(i,1:vlen).';
        tau = tau_u(i,1:ndof);

        % compute and set the current Cartesian positions of all
        % controlled constraint links of the lifting experiment:
        lnk_pos = updateLinkPos(lnk_pos, icub_wbc, qj, cstr_lnk_names, nLnks);

        % update the gripper-positions of the fixed distance constraint
        % and try to activate it:
        % note: the grippers must be parallel to the surface of the
        %       object (box) to be grabbed.
        dist_cstr = updateDistCstr(dist_cstr, fix_dist, lnk_pos(1:3,idx_gr(1,1)), lnk_pos(1:3,idx_gr(2,1)));

        % set the constraint value list for computing the constraint violations ...
        cstr_vals = setCstrValueList(cstr_vals, qj, tau, ndof, lnk_pos, nLnks, dist_cstr, nDCs);

        % add the constraint values to the penalty object and evaluate them for violations:
        obj.penalty_handling.EvaluateConstraints(cstr_vals, cstr_idx);
        cstr_idx = cstr_idx + 1;

        % get the current positions of the end-effectors, calculate the Cartesian
        % position errors (1-norm) and add them to the total trajectory error:
        pos_ee_r = lnk_pos(1:3,idx_ee(1,1));
        pos_ee_l = lnk_pos(1:3,idx_ee(2,1));
        traj_err = traj_err + norm(pos_ee_r - goal_pos.ee_r, 1) + norm(pos_ee_l - goal_pos.ee_l, 1);

        if (ti <= fset.tlim)
            if ~obj_grabbed
                % verify the current distances of the end-effectors to
                % their goal positions for grabbing the object ...
                d_ee_r = WBM.utilities.tfms.edist(pos_ee_r, goal_pos.ee_r);
                d_ee_l = WBM.utilities.tfms.edist(pos_ee_l, goal_pos.ee_l);
                if ( (d_ee_r <= fset.eps) && (d_ee_l <= fset.eps) )
                    obj_grabbed = true;
                    tidx_go     = ti;
                    % actualize the payload state ...
                    setPayloadState(obj_grabbed, ti);
                end
            end
        elseif (dist_cstr.active && obj_grabbed)
            break;
        else
            failure = true;
            break;
        end
    end
    clear setCstrValueList; % clear static variables
    fprms.cstr_idx = cstr_idx;

    if failure
        % grabbing the object failed ...
        fval_go = -1; % punish value
        return
    end
    % compute the fitness value of grabbing the object:
    fval_go = calcFitness(control.torques(:), traj_err, fset);
end

function [fval_mo, tidx_mo, cstr_idx] = fitnessMoveObj(obj, noi, t, q_j, tau_u, cstr_lnk_names, tidx_go, goal_pos, fprms, fset)
    control  = fprms.control;
    icub_wbc = fprms.icub_wbc;
    ndof     = fprms.ndof;
    nLnks    = fprms.nLnks;
    idx_ee   = fprms.idx_ee;
    idx_gr   = fprms.idx_gr;
    nDCs     = 2;
    nCstrs   = fprms.nCstrs + nDCs;

    tidx_go = tidx_go + 1;
    if (tidx_go > noi)
        tidx_go = noi;
    end

    % initialization:
    dist_cstr = repmat(struct('active', false, 'epsilon', fset.eps, 'p1', [], 'p2', []), nDCs, 1);
    fix_dist  = fprms.fix_dist;

    lnk_pos   = zeros(3,nLnks);
    cstr_vals = cell(1,nCstrs);
    cstr_idx  = fprms.cstr_idx + 1;
    traj_err  = 0;
    tidx_mo   = 0;

    % first (interm.) target positions of the end-effectors:
    tpos_ee_r = goal_pos(1,1).ee_r;
    tpos_ee_l = goal_pos(1,1).ee_l;

    % compute the trajectory error from the current position
    % of the grabbed object until to its destination:
    itrg_reached = false;
    vlen = size(q_j,2);
    for i = tidx_go:fset.smp_rate:noi
        ti  = t(i,1);
        qj  = q_j(i,1:vlen).';
        tau = tau_u(i,1:ndof);

        lnk_pos = updateLinkPos(lnk_pos, icub_wbc, qj, cstr_lnk_names, nLnks);

        % update the distance constraints for the hands (ee) and the grippers (gr):
        % note: the distance constraints guarantees that the object (box) to be
        %       moved is hold exact between the hands and the grippers, otherwise
        %       the object is getting lost.
        dist_cstr(1,1) = updateDistCstr(dist_cstr, fix_dist, lnk_pos(1:3,idx_ee(1,1)), lnk_pos(1:3,idx_ee(2,1)));
        dist_cstr(2,1) = updateDistCstr(dist_cstr, fix_dist, lnk_pos(1:3,idx_gr(1,1)), lnk_pos(1:3,idx_gr(2,1)));

        cstr_vals = setCstrValueList(cstr_vals, qj, tau, ndof, lnk_pos, nLnks, dist_cstr, nDCs);

        % evaluate the constraint values ...
        obj.penalty_handling.EvaluateConstraints(cstr_vals, cstr_idx);
        cstr_idx = cstr_idx + 1;

        % get the current end-effector positions ...
        pos_ee_r = lnk_pos(1:3,idx_ee(1,1));
        pos_ee_l = lnk_pos(1:3,idx_ee(2,1));

        if (ti <= fset.tlim)
            % check the current distances to the current target positions
            % of the end-effectors with the grabbed object ...
            d_ee_r = WBM.utilities.tfms.edist(pos_ee_r, tpos_ee_r);
            d_ee_l = WBM.utilities.tfms.edist(pos_ee_l, tpos_ee_l);
            trg_reached = (d_ee_r <= fset.eps) && (d_ee_l <= fset.eps);

            if ~itrg_reached
                if trg_reached
                    % intermediate target positions are reached:
                    itrg_reached = true;
                    if (dist_cstr(1,1).active && dist_cstr(2,1).active)
                        % set the targets to the end goal positions ...
                        tpos_ee_r = goal_pos(2,1).ee_r;
                        tpos_ee_l = goal_pos(2,1).ee_l;
                    else
                        break; % abortion without punishment (*)
                    end
                end
            elseif trg_reached
                tidx_mo = ti;
                break;
            end
        else
            break; % (*)
        end

        % calculate the corresponding trajectory error ...
        traj_err = traj_err + norm(pos_ee_r - tpos_ee_r, 1) + norm(pos_ee_l - tpos_ee_l, 1);
    end
    clear setCstrValueList; % clear static variables

    % compute the fitness value of moving the grabbed object to its target:
    fval_mo = calcFitness(control.torques(:), traj_err, fset);
end

function [fval_ro, tidx_ro, fprms] = fitnessReleaseObj(obj, noi, t, q_j, tau_u, cstr_lnk_names, tidx_mo, goal_pos, fprms, fset)
    setPayloadState(false, tidx_mo); % actualize the payload state (release object)

    control  = fprms.control;
    icub_wbc = fprms.icub_wbc;
    ndof     = fprms.ndof;
    nLnks    = fprms.nLnks;
    nCstrs   = fprms.nCstrs;

    tidx_mo = tidx_mo + 1;
    if (tidx_mo > noi)
        tidx_mo = noi;
    end

    % initialization:
    lnk_pos   = zeros(3,nLnks);
    cstr_vals = cell(1,nCstrs);
    cstr_idx  = fprms.cstr_idx + 1;
    traj_err  = 0;
    tidx_ro   = 0;

    % first (interm.) target positions of the end-effectors:
    tpos_ee_r = goal_pos(1,1).ee_r;
    tpos_ee_l = goal_pos(1,1).ee_l;

    % compute the trajectory error from the target position of the moved
    % and released object to the final destinations of the end-effectors:
    itrg_reached = false;
    vlen = size(q_j,2);
    for i = tidx_mo:fset.smp_rate:noi
        ti  = t(i,1);
        qj  = q_j(i,1:vlen).';
        tau = tau_u(i,1:ndof);

        lnk_pos   = updateLinkPos(lnk_pos, icub_wbc, qj, cstr_lnk_names, nLnks);
        cstr_vals = setCstrValueList(cstr_vals, qj, tau, ndof, lnk_pos, nLnks);
        % evaluate the constraint values ...
        obj.penalty_handling.EvaluateConstraints(cstr_vals, cstr_idx);
        cstr_idx = cstr_idx + 1;

        % current end-effector positions ...
        pos_ee_r = lnk_pos(1:3,idx_ee(1,1));
        pos_ee_l = lnk_pos(1:3,idx_ee(2,1));

        if (ti <= fset.tlim)
            % check the current distances to the current target positions ...
            d_ee_r = WBM.utilities.tfms.edist(pos_ee_r, tpos_ee_r);
            d_ee_l = WBM.utilities.tfms.edist(pos_ee_l, tpos_ee_l);
            trg_reached = (d_ee_r <= fset.eps) && (d_ee_l <= fset.eps);

            if ~itrg_reached
                if trg_reached
                    % intermediate target positions are reached,
                    % set the targets to the final destinations:
                    itrg_reached = true;
                    tpos_ee_r = goal_pos(2,1).ee_r;
                    tpos_ee_l = goal_pos(2,1).ee_l;
                end
            elseif trg_reached
                % final target positions are reached:
                tidx_ro = ti;
                break;
            end
        else
            break; % abortion without punishment ...
        end

        % trajectory error:
        traj_err = traj_err + norm(pos_ee_r - tpos_ee_r, 1) + norm(pos_ee_l - tpos_ee_l, 1);
    end
    clear setCstrValueList; % clear static variables
    fprms.cstr_idx = cstr_idx;

    % compute the fitness value after releasing the object:
    fval_ro = calcFitness(control.torques(:), traj_err, fset);
end

function lnk_pos = updateLinkPos(lnk_pos, icub_wbc, qj, cstr_lnk_names, nLnks)
    for j = 1:nLnks
        lnk_name = cstr_lnk_names{j,1};
        [pos,~]  = icub_wbc.offlineFkine(qj, lnk_name);
        lnk_pos(1:3,j) = pos;
    end
end

function dist_cstr = updateDistCstr(dist_cstr, fix_dist, lnk_pos_r, lnk_pos_l)
    % update the distance positions with the current positions of a specified
    % constraint link pair and activate the distance constraint when the
    % distance is below a fixed threshold:
    dist_cstr.p1 = lnk_pos_r;
    dist_cstr.p2 = lnk_pos_l;
    if ( ~dist_cstr.active && (ti <= fset.tlim) )
        d = WBM.utilities.tfms.edist(dist_cstr.p1, dist_cstr.p2);
        if (d <= fix_dist)
            % enable constraint ...
            dist_cstr.active = true;
        end
    end
end

function cstr_vals = setCstrValueList(cstr_vals, qj, tau, ndof, lnk_pos, nLnks, dist_cstr, nDCs)
    persistent nJVals jnt_vals ip; % static variables
    if isempty(nJVals)
        nJVals   = 4*ndof;
        jnt_vals = zeros(1,nJVals);
        ip       = nJVals + nLnks;
    end
    % joint values to verify the joint positions and torques with
    % the lin. inequalities, (x < k  &   x - k < 0)  and
    %                        (x > k  &  -x + k < 0).
    for i = 1:ndof
        % joint positions:
        q = qj(i+7,1);
        jnt_vals(1,2*i-1) = q; % for ineq. 1
        jnt_vals(1,2*i)   = q; % for ineq. 2
        % joint torques:
        t = tau(1,i);
        jnt_vals(1,2*(ndof+i)-1) = t;
        jnt_vals(1,2*(ndof+i))   = t;
    end
    cstr_vals(1,1:nJVals) = num2cell(jnt_vals);
    % Cartesian positions of the constraint links to compute the
    % distances between obstacles (obstacle avoidance):
    for i = 1:nLnks
        cstr_vals{1,nJVals+i} = lnk_pos(1:3,i).';
    end

    if (nargin == 8)
        % fixed distance constraints between two points
        % (p1, p2) with a specified tolerance:
        for i = 1:nDCs
            cstr_vals{1,ip+i} = dist_cstr(i,1);
        end
    end
end

function fval = calcFitness(ctrl_trqs, traj_err, fset)
    % calculate the effort:
    effort = sum(ctrl_trqs.*ctrl_trqs, 2);

    % control the saturation limits (cut-off) ...
    if (effort > fset.max_effort)
        effort = fset.max_effort;
    end
    if(traj_err > fset.max_traj_err)
       traj_err = fset.max_traj_err;
    end
    % normalize ...
    traj_err = traj_err / fset.max_traj_err;
    effort   = effort / fset.max_effort;
    fprintf(' trajectory error: %.3f\n', traj_err);
    %fprintf(' effort term:      %s\n', mat2str(effort, 2));

    % compute the fitness value ...
    fval = (-traj_err*fset.weight_traj_err - effort*fset.weight_effort) * (1/(fset.weight_traj_err + fset.weight_effort));
end

function setPayloadState(plstate, tidx)
    global gbl_plstate;
    if isempty(gbl_plstate)
        error('setPayloadState: %s', WBM.wbmErrorMsg.EMPTY_DATA_TYPE);
    end
    % set the global state values ...
    if (~gbl_plstate.obj_grabbed && plstate)
        % object grabbed:
        gbl_plstate.obj_grabbed  = plstate;
        gbl_plstate.obj_released = false;
        gbl_plstate.tidx_go      = tidx;
    elseif (gbl_plstate.obj_grabbed && ~gbl_plstate.obj_released && ~plstate)
        % object released:
        gbl_plstate.obj_grabbed  = plstate;
        gbl_plstate.obj_released = true;
        gbl_plstate.tidx_ro      = tidx;
    else
        error('setPayloadState: The payload state is ambiguous!');
    end
end
