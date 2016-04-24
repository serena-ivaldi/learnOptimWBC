classdef iCubWBM < IWBM.WBMInterface
    properties(Dependent)
        % public properties for fast get/set methods:
        wf_R_b@double matrix
        wf_p_b@double vector
        wf_v_b@double vector
        robot_model@WBM.wbmBaseModelParams
        robot_config@WBM.wbmBaseRobotConfig
        base@double   matrix
    end

    properties(Access = protected)
        mwbm_robot@WBM.WBM
        mstv@double    vector
        mwf_R_b@double matrix
        mwf_p_b@double vector
        mwf_v_b@double vector
        mndof@uint16   scalar
        mconfig
    end

    methods
        % Constructor:
        function obj = iCubWBM(@init_robot, wf2FixLnk, L, varargin)
            if ~exist('wf2FixLnk', 'var') )
                wf2FixLnk = true;
            end
            % init the mex-WholeBodyModel for the iCub-Robot:
            mwbm_robot = WBM.Robot.iCub.initRobot_iCub(wf2FixLnk);
        end

        function delete(obj)
            obj.delete();
        end

        function display(obj)

        end

        function actualizeState(obj)
            [wf_vqT_b,~, obj.mwf_v_b,~] = obj.mwbm_robot.getState();
            [obj.mwf_p_b, obj.mwf_R_b]  = WBM.utilities.frame2posRotm(wf_vqT_b);
        end

        function T = A(obj, joints, q_j)

        end

        function T0_6 = T0_6(obj, q_j)

        end

        function J0 = jacob0(obj, q_j, varargin)
            J0 = obj.mwbm_robot.jacobian(obj.mwf_R_b, obj.mwf_p_b, q_j);
        end

        function Jdot = jacob_dot(obj, q_j, dq_j)
            Jdot = obj.mwbm_robot.dJdq(obj.mwf_R_b, obj.mwf_p_b, q_j, dq_j, obj.mwf_v_b);
        end

        function M = inertia(obj, robot, q_j)
            M = obj.mwbm_robot.massMatrix(obj.mwf_R_b, obj.mwf_p_b, q_j);
        end

        function C = coriolis(obj, robot, q_j, dq_j)
            C = obj.mwbm_robot.coriolisCentrifugalForces(obj.mwf_R_b, obj.mwf_p_b, q_j, dq_j, obj.mwf_v_b);
        end

        function tg = gravload(obj, robot, q_j, grav)
            tg = obj.mwbm_robot.gravityForces(obj.mwf_R_b, obj.mwf_p_b, q_j);
        end

        function set.base(obj, v)
            if isempty(v)
                obj.base = eye(4,4);
            elseif ~ishomog(v)
                error('iCubWBM::set.base: The base must be a homogeneous transform!');
            else
                obj.base = v;
            end
        end

        function payload(obj, m, p)

        end

        function wf_R_b = get.wf_R_b(obj)
            wf_R_b = obj.mwf_R_b;
        end

        function wf_p_b = get.wf_p_b(obj)
            wf_p_b = obj.mwf_p_b;
        end

        function wf_v_b = get.wf_v_b(obj)
            wf_v_b = obj.mwf_v_b;
        end

        function robot_model = get.robot_model(obj)
            robot_model = obj.mwbm_robot.robot_model;
        end

        function robot_config = get.robot_config(obj)
            robot_config = obj.mwbm_robot.robot_config;
        end

    end
end
