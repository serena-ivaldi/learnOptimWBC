classdef iCubWBM < IWBM.WBMInterface
    properties(Dependent)
        % public properties for fast get/set methods:
        base_tform@double matrix
        tool_tform@double matrix
        model_name@char
        robot_manuf@char
        robot_name@char
        robot_parameters@WBM.wbmBaseRobotParams
        sim_config@WBM.absSimConfig
        gravity@double vector
        qlim@struct
        ndof@uint16    scalar
    end

    % properties(SetAccess = private, GetAccess = public)

    % end

    properties(Access = protected)
        mwbm_icub@WBM.WBM
        mrobot_manuf@char
        msim_config@WBM.absSimConfig
        mbase_tform@double matrix
        mtool_tform@double matrix
    end

    % properties(SetAccess = private, GetAccess = public)
    %     config % ??
    % end

    methods
        % Constructor:
        function obj = iCubWBM(robot_model, robot_config, wf2fixLnk)
            obj.mbase_tform  = eye(4,4);
            obj.mtool_tform  = obj.mbase_tform;
            obj.mrobot_manuf = 'Istituto Italiano di Tecnologia (IIT) - Genoa, Italy.';

            switch nargin
                % init the mex-WholeBodyModel for the iCub-Robot:
                case 2
                    obj.mwbm_icub = WBM.WBM(robot_model, robot_config);
                case 3
                    obj.mwbm_icub = WBM.WBM(robot_model, robot_config, wf2fixLnk);
            end
            % else, do nothing ...
        end

        function initRobot(robot_wbm)
            if ~isa(robot_wbm, 'WBM.WBM')
                error('iCubWBM::initRobot: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            obj.mwbm_icub = copy(robot_wbm);
        end

        function initRobotFcn(obj, fhInitRobotWBM, wf2fixLnk)
            if ~isa(fhInitRobotWBM, 'function_handle')
                error('iCubWBM::initRobotFcn: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE)
            end

            if ~exist('wf2fixLnk', 'var')
                wf2fixLnk = true;
            end
            obj.mwbm_icub = fhInitRobotWBM(wf2fixLnk);
        end

        function initBaseRobotParams(obj, base_params)
            if ~isa(base_params, 'WBM.wbmBaseRobotParams')
                error('iCubWBM::initBaseRobotParams: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            obj.mwbm_icub = WBM.WBM(base_params.robot_model, base_params.robot_config, base_params.wf2fixLnk);
        end

        function delete(obj)
            obj.delete();
        end

        function stFltb = getFltgBase(obj)
            stFltb = obj.mwbm_icub.getFloatingBaseState();
        end

        function ddq_j = accel(obj, q_j, dq_j, tau, stFltb)
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end
            M     = obj.mwbm_icub.massMatrix(stFltb.wf_R_b, stFltb.wf_p_b, q_j);
            I_acc = obj.Iqdd(q_j, dq_j, tau, stFltb);

            ddq_j = M \ I_acc.';
        end

        % function T = A(obj, joints, q_j) % ??

        % end

        function tau_c = coriolis(obj, q_j, dq_j, stFltb)
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end
            tau_c = obj.mwbm_icub.coriolisCentrifugalForces(stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.wf_v_b);
        end

        function [t, stmChi] = fdyn(obj, tspan, fhCtrlTrqs, stvChi_0, ode_opt)
            if ~exist('ode_opt',. 'var')
                % use the default options for the ODE-solver ...
                [t, stmChi] = obj.mwbm_icub.intForwardDynamics(fhCtrlTrqs, tspan, stvChi_0);
                return
            end
            % else ...
            [t, stmChi] = obj.mwbm_icub.intForwardDynamics(fhCtrlTrqs, tspan, stvChi_0, ode_opt);
        end

        function w_H_rlnk = fkine(obj, q_j, link_name, stFltb)
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end
            w_vqT_rlnk = obj.mwbm_icub.forwardKinematics(link_name, stFltb.wf_R_b, stFltb.wf_p_b, q_j);
            w_H_rlnk   = WBM.utilities.frame2tform(w_vqT_rlnk);
        end

        function tau_fr = friction(obj, dq_j)
            tau_fr = obj.mwbm_icub.frictionForces(dq_j);
        end

        function tau_g = gravload(obj, q_j, stFltb)
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end
            tau_g = obj.mwbm_icub.gravityForces(stFltb.wf_R_b, stFltb.wf_p_b, q_j);
        end

        function updateGrav(obj, g_wf)
            obj.mwbm_icub.g_wf = g_wf;
            obj.mwbm_icub.updateWorldFrame();
        end

        function M = inertia(obj, q_j, stFltb)
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end
            M = obj.mwbm_icub.massMatrix(stFltb.wf_R_b, stFltb.wf_p_b, q_j);
        end

        function tau_gen = invdyn(obj, q_j, dq_j, ddq_j, lnk_name, stFltb)
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end

            % M      = obj.mwbm_icub.massMatrix(stFltb.wf_R_b, stFltb.wf_p_b, q_j);
            % tau_c  = obj.mwbm_icub.coriolisCentrifugalForces(stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.wf_v_b);
            % tau_g  = obj.mwbm_icub.gravityForces(stFltb.wf_R_b, stFltb.wf_p_b, q_j);
            % tau_fr = obj.mwbm_icub.frictionForces(dq_j);

            % tau_gen = M*ddq_j + tau_c + tau_g + tau_fr;

            dv_b = ones(6,1); % dummy ...
            tau_gen = obj.mwbm_icub.inverseDynamics(stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.v_b, ddq_j, dv_b, lnk_name);
        end

        function resv = islimit(obj, q_j)
            resv = obj.mwbm_icub.isJointLimit(q_j);
        end

        function I_acc = Iqdd(obj, q_j, dq_j, tau, stFltb) % ??
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end

        end

        function dJ = jacob_dot(obj, q_j, dq_j, lnk_name, stFltb)
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end
            dJ = obj.mwbm_icub.dJdq(stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.wf_v_b, lnk_name);
        end

        function J_0 = jacob0(obj, q_j, lnk_name, stFltb)
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end
            J_0 = obj.mwbm_icub.jacobian(stFltb.wf_R_b, stFltb.wf_p_b, q_j, lnk_name);
        end

        function J_n = jacobn(obj, q_j, stFltb) % Jacobian matrix in tool frame (end-effector frame)
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end
            % compute the jacobian of the tooltip:
            % use the default tool (1st element of the tool list)
            J_n = obj.mwbm_icub.jacobianTool(obj, stFltb.wf_R_b, stFltb.wf_p_b, q_j, 1);
        end

        function T0_n = T0_m(obj, q_j, lnk_name, stFltb) % ??
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end
            % forward kinematics for the iCub-Robot up to frame m of n with m = [1, n] ...
            T0_n = obj.mwbm_icub.forwardKinematics(stFltb.wf_R_b, stFltb.wf_p_b, q_j, lnk_name);
        end

        function payload(obj, pt_mass, pos, link_names)
            if ( ~iscolumn(pt_mass) || ~ismatrix(pos) || (size(pt_mass,1) ~= size(pos,1) )
                error('iCubWBM::payload: %s', WBM.utilities.DIM_MISMATCH);
            end
            pl_data = horzcat(pt_mass, pos);
            obj.mwbm_icub.setLinkPayloads(link_names, pl_data);
        end

        function [M, C_qv, h_c] = wholeBodyDyn(obj, q_j, dq_j, stFltb)
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end

            M    = obj.mwbm_icub.massMatrix(stFltb.wf_R_b, stFltb.wf_p_b, q_j);
            C_qv = obj.mwbm_icub.generalizedBiasForces(stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.wf_v_b);
            h_c  = obj.mwbm_icub.centroidalMomentum(stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.wf_v_b); % = omega
        end

        function setupSim(obj, sim_config)
            obj.msim_config = obj.mwbm_icub.setupSimulation(sim_config);
        end

        function visualizeFDyn(obj, x_out, sim_tstep, vis_ctrl)
            obj.mwbm_icub.visualizeForwardDynamics(x_out, obj.msim_config, sim_tstep, vis_ctrl);
        end

        function qjout = sortJointValue(obj, string_search, q_j, valueVector) % ??

        end

        function dispParams(obj, prec)
            if exist('prec', 'var')
               obj.mwbm_icub.dispModel(prec);
               obj.mwbm_icub.dispConfig(prec);
               return
            end
            % else, display the values with the default precision ...
            obj.mwbm_icub.dispModel();
            obj.mwbm_icub.dispConfig();
        end

        function set.base_tform(obj, tform)
            if ~exist('tform', 'var') % is initialization correct here, or only eye(4,4)? (not sure)
                R_rlnk = obj.mwbm_icub.wf_R_rootLnk;
                p_rlnk = obj.mwbm_icub.wf_p_rootLnk;
                obj.mbase_tform = WBM.utilities.posRotm2tform(p_rlnk, R_rlnk);
                return
            end
            % else ...
            if ~WBM.utilities.isHomog(tform)
                error('iCubWBM::set.base_tform: %s', WBM.wbmErrorMsg.NOT_HOMOG_MAT);
            end
            [p_rlnk, R_rlnk] = WBM.utilities.tform2posRotm(tform);
            mwbm_icub.updateWorldFrame(R_rlnk, p_rlnk);
            obj.mbase_tform = tform;
        end

        function tform = get.base_tform(obj)
            tform = obj.mbase_tform;
        end

        function set.tool_tform(obj, tform)
            if ~exist('tform', 'var')
                obj.mtool_tform = eye(4,4);

                [tool_lnks, nTools] = obj.mwbm_icub.getToolLinks();
                if (nTools > 0)
                    % use the default tool (is always the first element of the list)
                    obj.mtool_tform(1:3,4) = tool_lnks(1,1).rlnk_p_tp;
                end
                return
            end
            % else ...
            if ~WBM.utilities.isHomog(tform)
                error('iCubWBM::set.tool_tform: %s', WBM.wbmErrorMsg.NOT_HOMOG_MAT);
            end
            obj.mtool_tform = tform;
        end

        function tform = get.tool_tform(obj)
            tform = obj.mtool_tform;
        end

        function set.model_name(obj, model_name)
            obj.mstr_model_name = model_name;
        end

        function model_name = get.model_name(obj)
            if isempty(obj.mstr_model_name)
                model_name = 'unknown'; return
            end
            % else ...
            model_name = obj.mstr_model_name;
        end

        function set.robot_manuf(obj, manuf)
            obj.mrobot_manuf = manuf;
        end

        function robot_manuf = get.robot_manuf(obj)
            if isempty(obj.mrobot_manuf)
                robot_manuf = 'unknown'; return
            end
            % else ...
            robot_manuf = obj.mrobot_manuf;
        end

        function robot_name = get.robot_name(obj)
            if isempty(obj.mstr_model_name)
                if ~isempty(obj.mwbm_icub)
                    [~,model_name, ext] = fileparts(obj.mwbm_icub.robot_model.urdfRobot);
                    model_name = strcat(model_name, ext);
                else
                    model_name = 'unknown';
                end
            end
            robot_name = strcat('iCub, model: ', model_name);
        end

        function robot_params = get.robot_parameters(obj)
            robot_params = obj.mwbm_icub.base_robot_params;
        end

        function sim_config = get.sim_config(obj)
            sim_config = obj.msim_config;
        end

        function g_wf = get.gravity(obj)
            g_wf = obj.mwbm_icub.g_wf;
        end

        function lmts = get.qlim(obj)
            lmts = obj.mwbm_icub.joint_limits;
        end

        function ndof = get.ndof(obj)
            if isempty(obj.mwbm_icub)
                ndof = 0; return
            end
            ndof = obj.mwbm_icub.ndof;
        end

    end
end
