classdef (Abstract) WBMInterface < handle
    properties(Abstract, Dependent)
        % public properties for fast get/set methods:
        R_b@double matrix
        p_b@double vector
        robot_model@WBM.wbmBaseModelParams
        robot_config@WBM.wbmBaseRobotConfig
    end

    properties(Abstract, Access = protected)
        iwbm_robot@WBM.WBM
        icurr_stv@double vector
        iR_b@double      matrix
        ip_b@double      vector
        indof@
        iconfig
    end

    methods(Abstract)
        % Constructor:
        function obj = WBMInterface(robot_model, robot_config, L, varargin)

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

        % function set.tool(obj, v)

        % end

        function set.base(obj, v)

        end

        % function set.offset(obj, v)

        % end

        % function v = get.offset(obj)

        % end

        % function set.qlim(obj, v)

        % end

        % function v = get.qlim(obj)

        % end

        % function set.gravity(obj, v)

        % end

        % function v = get.config(obj)

        % end

        function payload(obj, m, p)

        end

    end
end
