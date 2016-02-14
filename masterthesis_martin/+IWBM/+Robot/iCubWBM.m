classdef iCubWBM < IWBM.WBMInterface
    properties(Dependent)
        % public properties for fast get/set methods:
        wf_R_bf@double matrix
        wf_p_bf@double vector
        robot_model@WBM.wbmBaseModelParams
        robot_config@WBM.wbmBaseRobotConfig
    end

    properties(Access = protected)
        iwbm_robot@WBM.WBM
        icurr_stv@double vector
        iwf_R_bf@double  matrix
        iwf_p_bf@double  vector
        indof@uint16     scalar
        iconfig
    end

    methods
        % Constructor:
        function obj = iCubWBM(robot_model, robot_config, L, varargin)

        end

        function delete(obj)

        end

        function display(obj)

        end

        function T = A(obj, joints, q_j)

        end

        function T0_6 = T0_6(obj, q_j)

        end

        function J0 = jacob0(obj, q_j, varargin)

        end

        function Jdot = jacob_dot(obj, q_j, dq_j)

        end

        function M = inertia(obj, robot, q_j)

        end

        function C = coriolis(obj, robot, q_j, dq_j)

        end

        function tg = gravload(obj, robot, q_j, grav)

        end

        function set.base(obj, v)

        end

        function payload(obj, m, p)

        end

        function wf_R_bf = get.wf_R_bf(obj)

        end

        function wf_p_bf = get.wf_p_bf(obj)

        end

        function robot_model = get.robot_model(obj)

        end

        function robot_config = get.robot_config(obj)

        end

    end
end
