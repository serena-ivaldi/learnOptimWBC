classdef iCubWBM < IWBM.WBMInterface
    properties(Dependent)
        % public properties for fast get/set methods:
        robot_model@WBM.wbmBaseModelParams
        robot_config@WBM.wbmBaseRobotConfig
        stFltBase@WBM.wbmFltgBaseState
        ndof@uint16 scalar
        base@double matrix % ??
    end

    properties(Access = protected)
        mwbm_icub@WBM.WBM
        mstFltBase@WBM.wbmFltgBaseState;
        mstv@double vector
    end

    properties(SetAccess = private, GetAccess = public)
        config % ??
    end

    methods
        % Constructor:
        function obj = iCubWBM(robot_model, robot_config, wf2FixLnk)
            if ( (nargin == 2) || (nargin == 3) )
                if ~exist('wf2FixLnk', 'var')
                    wf2FixLnk = true;
                end

                % init the mex-WholeBodyModel for the iCub-Robot:
                obj.mwbm_icub = WBM.WBM(robot_model, robot_config, wf2FixLnk);
            end
        end

        function wbm_robot = initRobot(wbm_robot)
            if ~isa(wbm_robot, 'WBM.WBM')
                error('iCubWBM::initRobot: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            obj.mwbm_icub = copy(wbm_robot);
        end

        function wbm_robot = initRobotFcn(fhInitRobotWBM, wf2FixLnk)
            if ~ishandle(fhInitRobotWBM)
                error('iCubWBM::initRobotFcn: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE)
            end

            if ~exist('wf2FixLnk', 'var')
                wf2FixLnk = true;
            end
            obj.mwbm_icub = @fhInitRobotWBM(wf2FixLnk);
        end

        function delete(obj)
            obj.delete();
        end

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

        function actualizeFltBaseState(obj)
            obj.mstFltBase = obj.getFloatingBaseState();
        end

        function T = A(obj, joints, q_j) % ??

        end

        function T0_m = T0_m(obj, q_j, fltb_state, lnk_name)
            % forward kinematics for the iCub-Robot up to frame m of n with m = [1, n] ...
            if exist('lnk_name', 'var')
                T0_n = obj.mwbm_icub.forwardKinematics(lnk_name, obj.mstFltBase.wf_R_b, obj.mstFltBase.wf_p_b, q_j);
                return
            end
            % else, use the default link name ...
            T0_n = obj.mwbm_icub.forwardKinematics(obj.mstFltBase.wf_R_b, obj.mstFltBase.wf_p_b, q_j);
        end

        function J_0 = jacob0(obj, q_j, fltb_state, lnk_name)
            if exist('lnk_name', 'var')
                J_0 = obj.mwbm_icub.jacobian(lnk_name, obj.mstFltBase.wf_R_b, obj.mstFltBase.wf_p_b, q_j);
                return
            end
            % else, use the default link ...
            J_0 = obj.mwbm_icub.jacobian(obj.mstFltBase.wf_R_b, obj.mstFltBase.wf_p_b, q_j);
        end

        function J_dot = jacob_dot(obj, q_j, dq_j, fltb_state, lnk_name)
            if exist('lnk_name', 'var')
                J_dot = obj.mwbm_icub.dJdq(lnk_name, obj.mstFltBase.wf_R_b, obj.mstFltBase.wf_p_b, q_j, dq_j, obj.mstFltBase.wf_v_b);
                return
            end
            % else, use the default link ...
            J_dot = obj.mwbm_icub.dJdq(obj.mstFltBase.wf_R_b, obj.mstFltBase.wf_p_b, q_j, dq_j, obj.mstFltBase.wf_v_b);
        end

        function M = inertia(obj, q_j, fltb_state)
            M = obj.mwbm_icub.massMatrix(obj.mstFltBase.wf_R_b, obj.mstFltBase.wf_p_b, q_j);
        end

        function f_c = coriolis(obj, q_j, dq_j, fltb_state)
            f_c = obj.mwbm_icub.coriolisCentrifugalForces(obj.mwf_R_b, obj.mstFltBase.wf_p_b, q_j, dq_j, obj.mstFltBase.wf_v_b);
        end

        function f_g = gravload(obj, q_j, fltb_state, g_wf)
            if exist('g_wf', 'var')
                obj.mwbm_icub.g_wf = g_wf;
                obj.mwbm_icub.updateWorldFrame();
            end
            f_g = obj.mwbm_icub.gravityForces(obj.mstFltBase.wf_R_b, obj.mstFltBase.wf_p_b, q_j);
        end

        function set.base(obj, v) % ??
            if isempty(v)
                obj.base = eye(4,4);
            elseif ~ishomog(v)
                error('iCubWBM::set.base: The base must be a homogeneous transform!');
            else
                obj.base = v;
            end
        end

        function payload(obj, m, p) % ??

        end

        function stFltBase = get.stFltBase(obj)
            stFltBase = obj.mstFltBase;
        end

        function robot_model = get.robot_model(obj)
            robot_model = obj.mwbm_icub.robot_model;
        end

        function robot_config = get.robot_config(obj)
            robot_config = obj.mwbm_icub.robot_config;
        end

        function ndof = get.ndof(obj)
            ndof = obj.mwbm_icub.robot_config.ndof;
        end

    end
end
