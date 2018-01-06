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
% Note: This is a adapted version of the
%
function [fval, tidx_pl] = fitnessICubLiftObj2(obj, vargin)
    global gbl_plstate;
    if isempty(gbl_plstate)
        error('fitnessICubLiftObj2: %s', WBM.wbmErrorMsg.EMPTY_DATA_TYPE);
    end
    ilen = size(vargin,2);

    switch ilen
        case {4, 5}
            t              = vargin{1,1};
            q_j            = vargin{1,2}; % q_j with VQ-transformation
            cstr_lnk_names = vargin{1,3};
            idx_tp         = vargin{1,4}; % index positions of the target points of the end-effector tasks

            if ~iscellstr(cstr_lnk_names)
                error('fitnessICubLiftObj2: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            cstr_lnk_names = cstr_lnk_names(:); % make a column array ...
            idx_tp = idx_tp(:);
            noi    = size(t,1);

            if (ilen == 5)
                fset = vargin{1,5};
                if ~isstruct(fset)
                    error('fitnessICubLiftObj2: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
                end
                if ( (fset.tlim < 1) || (fset.tlim > noi) )
                    error('fitnessICubLiftObj2: %s', WBM.wbmErrorMsg.IDX_OUT_OF_BOUNDS);
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
            error('fitnessICubLiftObj2: %s', WBM.wbmErrorMsg.WRONG_ARR_SIZE);
    end
    if (mod(size(idx_tp,1),2) ~= 0)
        error('fitnessICubLiftObj2: %s', WBM.wbmErrorMsg.WRONG_VEC_LEN);
    end
    noi    = size(t,1);              % number of iterations
    nCLnks = size(cstr_lnk_names,1); % number of constraint links

    control  = obj.input_4_run{1,4};    % controller (UF or GHC) of the system
    icub_wbc = GetWholeSystem(control); % whole-body controller of the iCub
    ndof     = icub_wbc.ndof;

    % setup the parameters for the fitness functions ...
    fprms = struct('control', control, 'icub_wbc', icub_wbc, 'ndof', ndof, ...
                   'nCLnks', nCLnks, 'nCstrs', 4*ndof+nCLnks, 'cstr_idx', 1, ...
                   'idx_ee', 0, 'idx_gr', 0, 'fix_dist', obj.input_4_run{1,5});

    % get index positions of the hand (ee) and gripper (gr) links:
    idx_ee = find(ismember(cstr_lnk_names, {'l_hand', 'r_hand'}), 2);       % hand palms (ee)
    idx_gr = find(ismember(cstr_lnk_names, {'l_gripper', 'r_gripper'}), 2); % finger tips (gr)
    if ( isempty(idx_gr) || isempty(idx_ee) )
        error('fitnessGrabObj: %s', WBM.wbmErrorMsg.LNK_NOT_IN_LIST);
    end
    fprms.idx_ee = idx_ee;
    fprms.idx_gr = idx_gr;

    % set the goal points for the hand palm (end-effector, ee) and finger tips
    % (gripper, gr) tasks:
    % note: the goal points of each task are defined in the 'geom_parameters'
    %       array and the tasks are defined in the specified subchains of the
    %       first chain (see 'subchain1' array) of the current loaded
    %       configuration file.
    goal_pos = repmat(struct('ee_l', [], 'ee_r', [], 'gr_l', [], 'gr_r', []), 2, 1);

    % main goal positions - hand palm tasks (hands):
    goal_pos(1,1).ee_l = control.references.GetTraj(1, idx_tp(1,1), 1);
    goal_pos(1,1).ee_r = control.references.GetTraj(1, idx_tp(2,1), 1);
    % sub-goal positions - finger tips tasks (grippers):
    goal_pos(1,1).gr_l = control.references.GetTraj(1, idx_tp(3,1), 1);
    goal_pos(1,1).gr_r = control.references.GetTraj(1, idx_tp(4,1), 1);

    fval = -1;
    if ~gbl_plstate.obj_grabbed
        % Evaluate grabbing object:
        [fval_go,~,~,~] = fitnessGrabObj(obj, noi, t, q_j, cstr_lnk_names, ...
                                         goal_pos(1,1), fprms, fset);
        fval = fval_go;
    elseif (gbl_plstate.fval_go ~= -1)
        tidx_go = gbl_plstate.tidx_go;
        fval_go = gbl_plstate.fval_go;

        % Evaluate moving object to its destination:
        [fval_mo,~,~] = fitnessMoveObj(obj, noi, t, q_j, cstr_lnk_names, ...
                                       tidx_go, goal_pos, fprms, fset);

        % check the length of both fitness-vectors
        % and extend them to the same length ...
        vlen1 = size(fval_go,1);
        vlen2 = size(fval_mo,1);
        if (vlen1 < vlen2)
            fval_go = vertcat(fval_go, zeros(vlen2-vlen1,1));
        elseif (vlen1 > vlen2)
            fval_mo = vertcat(fval_mo, zeros(vlen1-vlen2,1));
        end

        fval = (fval_go + fval_mo) * 0.5; % avg. fitness value
    end

    if (nargout == 2)
        % set the time indices when the payload is grabbed ...
        tidx_pl = struct('grabbed', tidx_go, 'released', 0); % object will never released (in this case)
    end
end
%% END of fitnessICubLiftObj2.


%% FITNESS FUNCTIONS, LINK POSITIONS, CONSTRAINT VALUE LIST & PAYLOAD STATE:

function [fval_go, tidx_go, tau_u, fprms] = fitnessGrabObj(obj, noi, t, q_j, cstr_lnk_names, goal_pos, fprms, fset)
    control  = fprms.control;
    icub_wbc = fprms.icub_wbc;
    ndof     = fprms.ndof;
    nCLnks   = fprms.nCLnks;
    nCstrs   = fprms.nCstrs; % total number of constraints
    idx_ee   = fprms.idx_ee;
    idx_gr   = fprms.idx_gr;

    % uniform the torques tau w.r.t. the number of degrees of freedom (ndof):
    actvty_time = obj.input_4_run{1,3};
    tau_u       = InterpTorque(control, actvty_time, fset.intrpl_step);

    % initialize the variables for the constraint values ...
    clnk_pos    = zeros(3,nCLnks); % positions of the constraint links
    cstr_vals   = cell(1,nCstrs);  % constraint value list
    cstr_idx    = fprms.cstr_idx;  % constraint index (counter)
    failure     = false;
    traj_err    = 0;
    tidx_go     = 0;
    qj_go       = [];

    % compute the overall trajectory error until
    % the object is reached and grabbed:
    obj_grabbed  = false;
    vlen = size(q_j,2);
    tlim = fset.tlim;
    for i = 1:fset.smp_rate:noi
        ti  = t(i,1);
        qj  = q_j(i,1:vlen).';
        tau = tau_u(i,1:ndof);

        % compute and set the current Cartesian positions of all
        % controlled constraint links of the lifting experiment:
        clnk_pos = updateCLinkPos(clnk_pos, icub_wbc, qj, cstr_lnk_names, nCLnks);

        % set the constraint value list for computing the constraint violations ...
        cstr_vals = setCstrValueList(cstr_vals, qj, tau, ndof, clnk_pos, nCLnks);

        % add the constraint values to the penalty object and evaluate them for violations:
        EvaluateConstraints(obj.penalty_handling, cstr_vals, cstr_idx);
        cstr_idx = cstr_idx + 1;

        % get the current positions of the hand palms (ee) and finger tips (gr)
        % and calculate their Cartesian position errors (1-norm):
        % hand palms:
        pos_ee_l  = clnk_pos(1:3,idx_ee(1,1));
        pos_ee_r  = clnk_pos(1:3,idx_ee(2,1));
        traj_err1 = norm(pos_ee_l - goal_pos.ee_l, 1) + norm(pos_ee_r - goal_pos.ee_r, 1);
        % finger tips:
        pos_gr_l  = clnk_pos(1:3,idx_gr(1,1));
        pos_gr_r  = clnk_pos(1:3,idx_gr(2,1));
        traj_err2 = norm(pos_gr_l - goal_pos.gr_l, 1) + norm(pos_gr_r - goal_pos.gr_r, 1);

        % total trajectory error:
        traj_err = traj_err + ((traj_err1 + traj_err2) * 0.5); % avg. traj. error

        if (ti <= tlim)
            if ~obj_grabbed
                % verify the current distances of the end-effectors and
                % grippers to their goal positions for grabbing the object ...
                d_ee_l = WBM.utilities.tfms.edist(pos_ee_l, goal_pos.ee_l);
                d_ee_r = WBM.utilities.tfms.edist(pos_ee_r, goal_pos.ee_r);

                d_gr_l = WBM.utilities.tfms.edist(pos_gr_l, goal_pos.gr_l);
                d_gr_r = WBM.utilities.tfms.edist(pos_gr_r, goal_pos.gr_r);

                % fprintf('d_ee_l: %.3f\n', d_ee_l);
                % fprintf('d_ee_r: %.3f\n\n', d_ee_r);

                if ( (d_ee_l <= fset.eps) && (d_ee_r <= fset.eps) && ...
                     (d_gr_l <= fset.eps) && (d_gr_r <= fset.eps) )
                    % the main and sub-goals are reached ...
                    obj_grabbed = true;
                    tidx_go     = i;          % save current time index
                    qj_go       = qj(8:vlen); % save current joint pose
                    fprintf('Object grabbed successfully.\n');
                    break;
                end
            end
        else
            failure = true;
            break;
        end
    end
    clear setCstrValueList; % clear static variables
    fprms.cstr_idx = cstr_idx;

    if failure
        % grabbing the object failed:
        fval_go = -1; % punish value
        return
    end
    % compute the fitness value of grabbing the object:
    fval_go = calcFitness(control.torques(:), traj_err, fset);
    % actualize the payload state ...
    setPayloadState(obj_grabbed, tidx_go, qj_go, fval_go);
end

function [fval_mo, tidx_mo, cstr_idx] = fitnessMoveObj(obj, noi, t, q_j, cstr_lnk_names, tidx_go, goal_pos, fprms, fset)
    control  = fprms.control;
    icub_wbc = fprms.icub_wbc;
    ndof     = fprms.ndof;
    nCLnks   = fprms.nCLnks;
    nCstrs   = fprms.nCstrs;
    idx_ee   = fprms.idx_ee;
    idx_gr   = fprms.idx_gr;

    tidx_go = tidx_go + 1;
    if (tidx_go > noi)
        tidx_go = noi;
    end

    % uniformed torques:
    actvty_time = obj.input_4_run{1,3};
    tau_u = InterpTorque(control, actvty_time, fset.intrpl_step);

    % initialization:
    clnk_pos  = zeros(3,nCLnks);
    cstr_vals = cell(1,nCstrs);
    cstr_idx  = fprms.cstr_idx + 1;
    traj_err  = 0;
    tidx_mo   = 0;

    % target positions of the end-effectors and grippers:
    tpos_ee_l = goal_pos(1,1).ee_l;
    tpos_ee_r = goal_pos(1,1).ee_r;
    tpos_gr_l = goal_pos(1,1).gr_l;
    tpos_gr_r = goal_pos(1,1).gr_r;

    % compute the trajectory error from the current position
    % of the grabbed object until to its destination:
    trg_reached = false;
    vlen = size(q_j,2);
    tlim = fset.tlim;
    for i = tidx_go:fset.smp_rate:noi
        ti  = t(i,1);
        qj  = q_j(i,1:vlen).';
        tau = tau_u(i,1:ndof);

        clnk_pos = updateCLinkPos(clnk_pos, icub_wbc, qj, cstr_lnk_names, nCLnks);

        cstr_vals = setCstrValueList(cstr_vals, qj, tau, ndof, clnk_pos, nCLnks);

        % evaluate the constraint values ...
        EvaluateConstraints(obj.penalty_handling, cstr_vals, cstr_idx);
        cstr_idx = cstr_idx + 1;

        % get the current positions of the end-effectors and grippers ...
        pos_ee_l = clnk_pos(1:3,idx_ee(1,1));
        pos_ee_r = clnk_pos(1:3,idx_ee(2,1));
        pos_gr_l = clnk_pos(1:3,idx_gr(1,1));
        pos_gr_r = clnk_pos(1:3,idx_gr(2,1));

        % total trajectory error:
        traj_err1 = norm(pos_ee_l - tpos_ee_l, 1) + norm(pos_ee_r - tpos_ee_r, 1);
        traj_err2 = norm(pos_gr_l - tpos_gr_l, 1) + norm(pos_gr_r - tpos_gr_r, 1);
        traj_err  = traj_err + ((traj_err1 + traj_err2) * 0.5); % avg. traj. error

        if (ti <= tlim)
            % try to reach with the grabbed object the target positions
            % of the hand palms (ee) and finger tips (gr) ...
            if ~trg_reached
                d_ee_l = WBM.utilities.tfms.edist(pos_ee_l, tpos_ee_l);
                d_ee_r = WBM.utilities.tfms.edist(pos_ee_r, tpos_ee_r);

                d_gr_l = WBM.utilities.tfms.edist(pos_gr_l, tpos_gr_l);
                d_gr_r = WBM.utilities.tfms.edist(pos_gr_r, tpos_gr_r);

                % fprintf('d_ee_l: %.3f\n', d_ee_l);
                % fprintf('d_ee_r: %.3f\n\n', d_ee_r);

                if ( (d_ee_l <= fset.eps) && (d_ee_r <= fset.eps) && ...
                     (d_gr_l <= fset.eps) && (d_gr_r <= fset.eps) )
                    % targets are reached:
                    trg_reached = true;
                    tidx_mo     = i;
                    fprintf('Final targets reached.\n');
                end
            end
        elseif ~trg_reached
            fprintf(2, 'Reaching targets failed.\n');
            break; % abortion without punishment
        end
    end
    clear setCstrValueList; % clear static variables

    % compute the fitness value of moving the grabbed object to its target:
    fval_mo = calcFitness(control.torques(:), traj_err, fset);
    setPayloadState(false, 0); % actualize state
end

function clnk_pos = updateCLinkPos(clnk_pos, icub_wbc, qj, cstr_lnk_names, nCLnks)
    for j = 1:nCLnks
        clnk_name = cstr_lnk_names{j,1};
        [pos,~]   = icub_wbc.offlineFkine(qj, clnk_name);
        clnk_pos(1:3,j) = pos;
    end
end

function cstr_vals = setCstrValueList(cstr_vals, qj, tau, ndof, clnk_pos, nCLnks, varargin)
    persistent nJVals jnt_vals ip; % static variables
    if isempty(nJVals)
        nJVals   = 4*ndof;
        jnt_vals = zeros(1,nJVals);
        ip       = nJVals + nCLnks; % index pos.
    end
    switch nargin
        case {6, 8}
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

            if (nargin == 8)
                fdist_cstr = varargin{1,1};
                idx_fd     = varargin{1,2};
                nFDCs      = size(fdist_cstr,1);

                % fixed distance constraints between two points
                % (p1, p2) with a specified tolerance:
                for i = 1:nFDCs
                    idx = idx_fd(i,1);
                    cstr_vals{1,ip+idx} = fdist_cstr(i,1);
                end
            end
        otherwise
            error('setCstrValueList: %s', WBM.utilities.wbmErrorMsg.WRONG_NARGIN);
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
