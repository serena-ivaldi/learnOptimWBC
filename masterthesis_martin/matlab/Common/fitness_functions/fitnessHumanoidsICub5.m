% Fitness function for the two arms of the iCub humanoid robot.
% The aim of this fitness function is to evaluate the behavior for reaching two
% goal points behind a wall, one point for each hand. The ambition is to reach
% at first these points before a fixed time limit is expired and afterwards
% reaching the changed goal positions behind the wall by activating a fixed
% distance constraint. The points will be reached with minimal torque values
% under the common robot constraints, the joints and torques limits, and with
% collision detection.
%
% Note: This fitness function is a revised version of the fitness function
%       'fitnessHumanoidsIcub4'. The calculation of the evaluation value is
%        identical to the original function but more flexible and efficient.
%
function fval = fitnessHumanoidsICub5(obj, vargin)
    ilen = size(vargin,2);
    switch ilen
        case {4, 5}
            t              = vargin{1,1};
            q_j            = vargin{1,2};
            cstr_lnk_names = vargin{1,3};
            idx_tp         = vargin{1,4}; % index positions of the target points of the end-effector tasks

            WBM.utilities.chkfun.checkCVecDim(idx_tp, 4, 'fitnessHumanoidsICub5');
            if ~iscellstr(cstr_lnk_names)
                error('fitnessHumanoidsICub5: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            cstr_lnk_names = cstr_lnk_names(:); % make a column array ...
            noi   = size(t,1);

            if (ilen == 5)
                fset = vargin{1,5};
                if ~isstruct(fset)
                    error('fitnessHumanoidsICub5: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
                end
                if ( (fset.tlim < 1) || (fset.tlim > noi) )
                    error('fitnessHumanoidsICub5: %s', WBM.wbmErrorMsg.IDX_OUT_OF_BOUNDS);
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
            error('fitnessHumanoidsICub5: %s', WBM.wbmErrorMsg.WRONG_ARR_SIZE);
    end
    nCLnks = size(cstr_lnk_names,1); % number of constraint links

    % get index positions of the wrist and gripper links:
    idx_wr = find(ismember(cstr_lnk_names, {'r_wrist_1', 'l_wrist_1'}), 2);
    idx_ee = find(ismember(cstr_lnk_names, {'r_gripper', 'l_gripper'}), 2); % end-effectors (ee)
    if ( isempty(idx_wr) || isempty(idx_ee) )
        error('fitnessHumanoidsICub5: %s', WBM.wbmErrorMsg.LNK_NOT_IN_LIST);
    end

    % initialize the distance constraint structure ...
    fdist_cstr = struct('active', false, 'epsilon', fset.eps, 'p1', [], 'p2', []);
    fix_dist   = obj.input_4_run{1,5}; % fixed distance value for the distance constraint.

    % get the specified controller (UF or GHC) for the torque function:
    control  = obj.input_4_run{1,4};
    icub_wbc = GetWholeSystem(control);
    ndof     = icub_wbc.ndof;

    % uniform the torques tau w.r.t. the number of degrees of freedom (ndof):
    actvty_time = obj.input_4_run{1,3};
    tau_u       = InterpTorque(control, actvty_time, fset.intrpl_step);

    % initialize the variables for the constraint values ...
    clnk_pos  = zeros(3,nCLnks);         % positions of the constraint links
    cstr_vals = cell(1,4*ndof+nCLnks+1); % constraint value list
    cstr_idx  = 1;                       % constraint index (counter)

    traj_err = 0;
    failure  = false;

    % get the goal points for the end-effector tasks which are defined
    % in the specified subchains of the first chain (see 'subchain1' in
    % the corresponding configuration script):
    % goal positions of the end-effector tasks ...
    goal_pos.ee_r = control.references.GetTraj(1, idx_tp(1,1), 1);
    goal_pos.ee_l = control.references.GetTraj(1, idx_tp(2,1), 1);
    % fixed endpoints (new goal positions for the 2nd step) ...
    goal_pos.ep_r = control.references.GetTraj(1, idx_tp(3,1), 1);
    goal_pos.ep_l = control.references.GetTraj(1, idx_tp(4,1), 1);

    % 1st step - use the goal positions of the
    % end-effectors as attraction positions:
    attr_pos_r = goal_pos.ee_r;
    attr_pos_l = goal_pos.ee_l;

    % compute the total trajectory error of all iterations ...
    scnd_step = false;
    vlen = size(q_j,2);
    for i = 1:fset.smp_rate:noi
        ti  = t(i,1);
        qj  = q_j(i,1:vlen).';
        tau = tau_u(i,1:ndof);

        % compute and set the current Cartesian positions of
        % all controlled constraint links of the experiment:
        clnk_pos = updateCLinkPos(clnk_pos, icub_wbc, qj, cstr_lnk_names, nCLnks);

        % set the positions for the fixed distance constraint and try to activate it:
        fdist_cstr.p1 = clnk_pos(1:3,idx_wr(1,1)); % pos. of r_wrist_1
        fdist_cstr.p2 = clnk_pos(1:3,idx_wr(2,1)); % pos. of l_wrist_1
        if ( ~fdist_cstr.active && (ti < fset.tlim) )
            d = WBM.utilities.tfms.edist(fdist_cstr.p1, fdist_cstr.p2);
            if (d < fix_dist)
                % enable constraint ...
                fdist_cstr.active = true;
            end
        end

        % set the constraint value list for computing the constraint violations ...
        cstr_vals = setCstrValueList(cstr_vals, qj, tau, ndof, clnk_pos, nCLnks, fdist_cstr);

        % add the constraint values to the penalty object and evaluate them for violations:
        EvaluateConstraints(obj.penalty_handling, cstr_vals, cstr_idx);
        cstr_idx = cstr_idx + 1;

        if ( (ti >= fset.tlim) && ~scnd_step )
            if fdist_cstr.active
                % 2nd step - use the fixed endpoints
                % as attraction positions:
                attr_pos_r = goal_pos.ep_r;
                attr_pos_l = goal_pos.ep_l;
                scnd_step = true;
            else
                failure = true;
                break;
            end
        end
        % get the current positions of the end-effectors ...
        pos_ee_r = clnk_pos(1:3,idx_ee(1,1));
        pos_ee_l = clnk_pos(1:3,idx_ee(2,1));

        % calculate the Cartesian position errors (1-norm)
        % and add them to the total trajectory error:
        traj_err = traj_err + norm((pos_ee_r - attr_pos_r), 1) + norm((pos_ee_l - attr_pos_l), 1);
    end
    clear setCstrValueList; % clear static variables

    if failure
        fval = -1; % punish value
        return
    end

    % calculate the effort:
    ctrl_trqs = control.torques(:);
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

    % calculate the overall fitness value ...
    fval = (-traj_err*fset.weight_traj_err - effort*fset.weight_effort) * (1/(fset.weight_traj_err + fset.weight_effort));
end
%% END of fitnessHumanoidsICub5.


%% LINK POSITIONS & CONSTRAINT VALUES:

function clnk_pos = updateCLinkPos(clnk_pos, icub_wbc, qj, cstr_lnk_names, nCLnks)
    for j = 1:nCLnks
        clnk_name = cstr_lnk_names{j,1};
        [pos,~]   = icub_wbc.offlineFkine(qj, clnk_name);
        clnk_pos(1:3,j) = pos;
    end
end

function cstr_vals = setCstrValueList(cstr_vals, qj, tau, ndof, clnk_pos, nCLnks, fdist_cstr)
    persistent nJVals nCstrs jnt_vals; % static variables
    if isempty(nJVals)
        nJVals   = 4*ndof;
        nCstrs   = nJVals + nCLnks + 1;
        jnt_vals = zeros(1,nJVals);
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
    for i = 1:nCLnks
        cstr_vals{1,nJVals+i} = clnk_pos(1:3,i).';
    end
    % fixed distance constraint between two points
    % (p1, p2) with a specified tolerance:
    cstr_vals{1,nCstrs} = fdist_cstr;
end
