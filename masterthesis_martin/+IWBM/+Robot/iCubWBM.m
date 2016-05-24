classdef iCubWBM < IWBM.WBMInterface
    properties(Dependent)
        % public properties for fast get/set methods:
        base_tform@double matrix
        robot_model@WBM.wbmBaseModelParams
        robot_config@WBM.wbmBaseRobotConfig
        model_name@char
        ndof@uint16 scalar

        % stFltBase@WBM.wbmFltgBaseState
    end

    % properties(SetAccess = private, GetAccess = public)

    % end

    properties(Access = protected)
        mwbm_icub@WBM.WBM
        mtform_b@double matrix
    end

    % properties(SetAccess = private, GetAccess = public)
    %     config % ??
    % end

    methods
        % Constructor:
        function obj = iCubWBM(robot_model, robot_config, wf2FixLnk)
            mtform_b = eye(4,4);

            switch nargin
                % init the mex-WholeBodyModel for the iCub-Robot:
                case 2
                    obj.mwbm_icub = WBM.WBM(robot_model, robot_config);
                case 3
                    obj.mwbm_icub = WBM.WBM(robot_model, robot_config, wf2FixLnk);
            end
            % else, do nothing ...
        end

        function initRobot(wbm_robot)
            if ~isa(wbm_robot, 'WBM.WBM')
                error('iCubWBM::initRobot: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            obj.mwbm_icub = copy(wbm_robot);
        end

        function initRobotFcn(obj, fhInitRobotWBM, wf2FixLnk)
            if ~ishandle(fhInitRobotWBM)
                error('iCubWBM::initRobotFcn: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE)
            end

            if ~exist('wf2FixLnk', 'var')
                wf2FixLnk = true;
            end
            obj.mwbm_icub = @fhInitRobotWBM(wf2FixLnk); % to check if it works ...
        end

        function initRobotBaseParams(obj, base_params)
            if ~isa(base_params, 'WBM.wbmRobotBaseParams')
                error('iCubWBM::initRobotBaseParams: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            obj.mwbm_icub = WBM.WBM(base_params.robot_model, base_params.robot_config, base_params.wf2FixLnk);
        end

        function delete(obj)
            obj.delete();
        end

        function ddq_j = accel(obj, q_j, dq_j, tau)
            M     = obj.inertia(q_j);
            I_acc = obj.Iqdd(q_j, dq_j, tau);

            ddq_j = M \ I_acc';
        end

        function I_acc = Iqdd(obj, q_j, dq_j, tau) % ??

        end

        function tau = invdyn(obj, q_j, dq_j, ddq_j)
            stFltb = obj.getFloatingBaseState();

            M      = obj.mwbm_icub.massMatrix(stFltb.wf_R_b, stFltb.wf_p_b, q_j);
            tau_c  = obj.mwbm_icub.coriolisCentrifugalForces(stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.wf_v_b);
            tau_g  = obj.mwbm_icub.gravityForces(stFltb.wf_R_b, stFltb.wf_p_b, q_j);
            tau_fr = obj.friction(dq_j);

            tau = M*ddq_j + tau_c + tau_g - tau_fr;
        end

        % function T = A(obj, joints, q_j) % ??

        % end

        function tau_c = coriolis(obj, q_j, dq_j)
            stFltb = obj.getFloatingBaseState();
            tau_c  = obj.mwbm_icub.coriolisCentrifugalForces(stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.wf_v_b);
        end

        function tau_fr = friction(obj, dq_j)
            stvLen = obj.mwbm_icub.stvLen;
            friction_coeff = ones(stvLen,1); % dummy-vector

            tau_fr = dq_j.*friction_coeff;
        end

        function tau_g = gravload(obj, q_j, g_wf)
            if exist('g_wf', 'var')
                obj.mwbm_icub.g_wf = g_wf;
                obj.mwbm_icub.updateWorldFrame();
            end

            stFltb = obj.getFloatingBaseState();
            tau_g  = obj.mwbm_icub.gravityForces(stFltb.wf_R_b, stFltb.wf_p_b, q_j);
        end

        function M = inertia(obj, q_j)
            stFltb = obj.getFloatingBaseState();
            M = obj.mwbm_icub.massMatrix(stFltb.wf_R_b, stFltb.wf_p_b, q_j);
        end

        function J_0 = jacob0(obj, q_j, lnk_name)
            stFltb = obj.getFloatingBaseState();

            if exist('lnk_name', 'var')
                J_0 = obj.mwbm_icub.jacobian(lnk_name, stFltb.wf_R_b, stFltb.wf_p_b, q_j);
                return
            end
            % else, use the default link ...
            J_0 = obj.mwbm_icub.jacobian(stFltb.wf_R_b, stFltb.wf_p_b, q_j);
        end

        function dJ = jacob_dot(obj, q_j, dq_j, lnk_name)
            stFltb = obj.getFloatingBaseState();

            if exist('lnk_name', 'var')
                dJ = obj.mwbm_icub.dJdq(lnk_name, stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.wf_v_b);
                return
            end
            % else, use the default link ...
            dJ = obj.mwbm_icub.dJdq(stFltb.wf_R_b, stFltb.wf_p_b, q_j, dq_j, stFltb.wf_v_b);
        end



        function T0_n = T0_m(obj, q_j, lnk_name)
            stFltb = obj.getFloatingBaseState();

            % forward kinematics for the iCub-Robot up to frame m of n with m = [1, n] ...
            if exist('lnk_name', 'var')
                T0_n = obj.mwbm_icub.forwardKinematics(lnk_name, stFltb.wf_R_b, stFltb.wf_p_b, q_j);
                return
            end
            % else, use the default link name ...
            T0_n = obj.mwbm_icub.forwardKinematics(stFltb.wf_R_b, stFltb.wf_p_b, q_j);
        end



        % function payload(obj, m, p) % ??

        % end

        function display(obj, prec)
            if exist('prec', 'var')
               obj.mwbm_icub.dispWBMParams(prec);
               obj.mwbm_icub.dispWBMConfig(prec);
               return
            end
            % else, display the values with the default precision ...
            obj.mwbm_icub.dispWBMParams();
            obj.mwbm_icub.dispWBMConfig();
        end

        % function stFltb = get.stFltBase(obj)
        %     stFltb = obj.getFloatingBaseState();
        % end

        function set.base_tform(obj, tform)
            if isempty(tform)
                obj.mtform_b = eye(4,4);
            elseif ~WBM.utilities.isHomog(tform)
                error('iCubWBM::set.base_tform: The base must be a homogeneous transformation!');
            else
                obj.mtform_b = tform;
            end
        end

        function tform_b = get.base_tform(obj)
            tform_b = obj.mtform_b;
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
            ndof = obj.mwbm_icub.robot_config.ndof;
        end

    end
end
