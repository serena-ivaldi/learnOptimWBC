classdef (Abstract) WBMInterface < handle
    properties(Abstract, Dependent)
        % public properties for fast get/set methods:
        ndof@uint16 scalar
        robot_model@WBM.wbmBaseModelParams
        robot_config@WBM.wbmBaseRobotConfig
        stFltBase@WBM.wbmFltgBaseState
        base@double matrix % ??
    end

    properties(Abstract, Access = protected)
        mwbm_icub@WBM.WBM
        mstFltBase@WBM.wbmFltgBaseState;
        mstv@double vector
    end

    properties(SetAccess = private, GetAccess = public)
        config % ??
    end

    methods(Abstract)
        % Constructor:
        %obj = WBMInterface(robot_model, robot_config, wf2FixLnk, L, varargin)

        delete(obj)

        display(obj)

        T = A(obj, joints, q_j)

        T0_m = T0_m(obj, q_j, lnk_name)

        J_0 = jacob0(obj, q_j, lnk_name)

        J_dot = jacob_dot(obj, q_j, dq_j, lnk_name)

        M = inertia(obj, q_j)

        f_c = coriolis(obj, q_j, dq_j)

        f_g = gravload(obj, q_j, g_wf)

        % set.tool(obj, v)

        set.base(obj, v) % ??

        % set.offset(obj, v)

        % v = get.offset(obj)

        % set.qlim(obj, v)

        % v = get.qlim(obj)

        % set.gravity(obj, v)

        % v = get.config(obj)

        payload(obj, m, p) % ??

        stFltBase = get.stFltBase(obj)

        robot_model = get.robot_model(obj)

        robot_config = get.robot_config(obj)

        ndof = get.ndof(obj)

    end
end
