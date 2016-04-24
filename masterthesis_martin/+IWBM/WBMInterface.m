classdef (Abstract) WBMInterface < handle
    properties(Abstract, Dependent)
        % public properties for fast get/set methods:
        wf_R_b@double matrix
        wf_p_b@double vector
        wf_v_b@double vector
        robot_model@WBM.wbmBaseModelParams
        robot_config@WBM.wbmBaseRobotConfig
    end

    properties(Abstract, Access = protected)
        mwbm_robot@WBM.WBM
        mstv@double    vector
        mwf_R_b@double matrix
        mwf_p_b@double vector
        mwf_v_b@double vector
        mndof@uint16   scalar
        mconfig
    end

    methods(Abstract)
        % Constructor:
        %obj = WBMInterface(robot_model, robot_config, wf2FixLnk, L, varargin)

        delete(obj)

        display(obj)

        T = A(obj, joints, q_j)

        T0_6 = T0_6(obj, q_j)

        J0 = jacob0(obj, q_j, varargin)

        Jdot = jacob_dot(obj, q_j, dq_j)

        M = inertia(obj, robot, q_j)

        C = coriolis(obj, robot, q_j, dq_j)

        tg = gravload(obj, robot, q_j, grav)

        % set.tool(obj, v)

        set.base(obj, v)

        % set.offset(obj, v)

        % v = get.offset(obj)

        % set.qlim(obj, v)

        % v = get.qlim(obj)

        % set.gravity(obj, v)

        % v = get.config(obj)

        payload(obj, m, p)

        wf_R_b = get.wf_R_b(obj)

        wf_p_b = get.wf_p_b(obj)

        wf_v_b = get.wf_v_b(obj)

        robot_model = get.robot_model(obj)

        robot_config = get.robot_config(obj)

    end
end
