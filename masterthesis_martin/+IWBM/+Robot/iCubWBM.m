classdef iCubWBM < IWBM.WBMInterface
    properties(Dependent)
        % public properties for fast get/set methods:
        base_tform@double matrix
        tool_tform@double matrix
        robot_model@WBM.wbmBaseRobotModel
        robot_config@WBM.wbmBaseRobotConfig
        model_name@char
        ndof@uint16 scalar
    end

    % properties(SetAccess = private, GetAccess = public)

    % end

    properties(Access = protected)
        mwbm_icub@WBM.WBM
        msim_config@WBM.genericSimConfig
        mbase_tform@double matrix
        mtool_tform@double matrix
    end

    % properties(SetAccess = private, GetAccess = public)
    %     config % ??
    % end

    methods
        % Constructor:
        function obj = iCubWBM(robot_model, robot_config, wf2fixLnk)
            obj.mbase_tform = eye(4,4);
            obj.mtool_tform = obj.mbase_tform;

            switch nargin
                % init the mex-WholeBodyModel for the iCub-Robot:
                case 2
                    obj.mwbm_icub = WBM.WBM(robot_model, robot_config);
                case 3
                    obj.mwbm_icub = WBM.WBM(robot_model, robot_config, wf2fixLnk);
            end
            % else, do nothing ...
        end

        function initRobot(wbm_robot)
            if ~isa(wbm_robot, 'WBM.WBM')
                error('iCubWBM::initRobot: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            obj.mwbm_icub = copy(wbm_robot);
        end

        function initRobotFcn(obj, fhInitRobotWBM, wf2fixLnk)
            if ~ishandle(fhInitRobotWBM)
                error('iCubWBM::initRobotFcn: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE)
            end

            if ~exist('wf2fixLnk', 'var')
                wf2fixLnk = true;
            end
            obj.mwbm_icub = @fhInitRobotWBM(wf2fixLnk); % to check if it works ...
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

            ddq_j = M \ I_acc';
        end

        % function T = A(obj, joints, q_j) % ??

        % end

        function tau_c = coriolis(obj, q_j, dq_j, stFltb)
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end
            tau_c = obj.mwbm_icub.coriolisCentrifugalForces(stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.wf_v_b);
        end

        function w_H_rlnk = fkine(obj, q_j, link_name, stFltb)
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end
            w_vqT_rlnk = obj.mwbm_icub.forwardKinematics(link_name, stFltb.wf_R_b, stFltb.wf_p_b, q_j);
            w_H_rlnk   = WBM.utilities.frame2tform(w_vqT_rlnk);
        end

        function tau_fr = friction(obj, dq_j)
            tau_fr = mwbm_icub.frictionForces(dq_j);
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

        function tau = invdyn(obj, q_j, dq_j, ddq_j, stFltb)
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end

            M      = obj.mwbm_icub.massMatrix(stFltb.wf_R_b, stFltb.wf_p_b, q_j);
            tau_c  = obj.mwbm_icub.coriolisCentrifugalForces(stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.wf_v_b);
            tau_g  = obj.mwbm_icub.gravityForces(stFltb.wf_R_b, stFltb.wf_p_b, q_j);
            tau_fr = obj.friction(dq_j);

            tau = M*ddq_j + tau_c + tau_g - tau_fr;
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

        function jacobn = jacobn(obj, q_j, stFltb) % ??
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end

        end

        function T0_n = T0_m(obj, q_j, lnk_name, stFltb) % ??
            if ~exist('stFltb', 'var')
                stFltb = obj.mwbm_icub.getFloatingBaseState();
            end
            % forward kinematics for the iCub-Robot up to frame m of n with m = [1, n] ...
            T0_n = obj.mwbm_icub.forwardKinematics(stFltb.wf_R_b, stFltb.wf_p_b, q_j, lnk_name);
        end

        % function payload(obj, pt_mass, pos, link_name)

        % end

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

        function disp(obj, prec)
            if exist('prec', 'var')
               obj.mwbm_icub.dispWBMParams(prec);
               obj.mwbm_icub.dispWBMConfig(prec);
               return
            end
            % else, display the values with the default precision ...
            obj.mwbm_icub.dispWBMParams();
            obj.mwbm_icub.dispWBMConfig();
        end

        function set.base_tform(obj, tform)
            if isempty(tform)
                obj.mbase_tform = eye(4,4);
            elseif ~WBM.utilities.isHomog(tform)
                error('iCubWBM::set.base_tform: %s', WBM.wbmErrorMsg.NOT_HOMOG_MAT);
            else
                obj.mbase_tform = tform;
            end
        end

        function tform = get.base_tform(obj)
            tform = obj.mbase_tform;
        end

        function set.tool_tform(obj, tform)
            if isempty(tform)
                obj.mtool_tform = eye(4,4);
            elseif ~WBM.utilities.isHomog(tform)
                error('iCubWBM::set.tool_tform: %s', WBM.wbmErrorMsg.NOT_HOMOG_MAT);
            else
                obj.mtool_tform = tform;
            end
        end

        function tform = get.tool_tform(obj)
            tform = obj.mtool_tform;
        end

        function robot_model = get.robot_model(obj)
            robot_model = obj.mwbm_icub.robot_model;
        end

        function robot_config = get.robot_config(obj)
            robot_config = obj.mwbm_icub.robot_config;
        end

        function model_name = get.model_name(obj)
            if isempty(obj.mwbm_icub)
                model_name = 'unknown'; return
            end
            model_name = obj.mwbm_icub.robot_model.urdfRobot;
        end

        function ndof = get.ndof(obj)
            if isempty(obj.mwbm_icub)
                ndof = 0; return
            end
            ndof = obj.mwbm_icub.robot_model.ndof;
        end

    end
end
