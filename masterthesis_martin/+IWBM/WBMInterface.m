classdef (Abstract) WBMInterface < handle
    properties(Abstract, Dependent)
        % public properties for fast get/set methods:
        base_tform@double matrix
        tool_tform@double matrix
        model_name@char
        robot_manuf@char
        robot_name@char
        robot_parameters@WBM.wbmBaseRobotParams
        sim_config@WBM.absSimConfig
        gravity@double vector
        qlim@double    vector
        ndof@uint16    scalar
    end

    % properties(SetAccess = private, GetAccess = public)
    %     config % ??
    % end

    methods(Abstract)
        % Constructor:
        %obj = WBMInterface(robot_model, robot_config, wf2fixLnk)

        initRobot(robot_wbm)

        initRobotFcn(obj, fhInitRobotWBM, wf2fixLnk)

        initBaseRobotParams(obj, base_params)

        delete(obj)

        stFltb = getFltgBase(obj)

        ddq_j = accel(obj, q_j, dq_j, tau, stFltb) % joint acceleration

        %T = A(obj, joints, q_j) % ??

        tau_c = coriolis(obj, q_j, dq_j, stFltb)

        [dstvChi, C_vq] = fdyn(obj, t, stvChi, ctrlTrqs)

        w_H_rlnk = fkine(obj, q_j, link_name, stFltb)

        tau_fr = friction(obj, dq_j)

        tau_g = gravload(obj, q_j, stFltb)

        updateGrav(obj, g_wf)

        M = inertia(obj, q_j, stFltb)

        tau_gen = invdyn(obj, q_j, dq_j, ddq_j, stFltb)

        resv = islimit(obj, q_j)

        I_acc = Iqdd(obj, q_j, dq_j, tau, stFltb) % ??

        dJ = jacob_dot(obj, q_j, dq_j, lnk_name, stFltb)

        J_0 = jacob0(obj, q_j, lnk_name, stFltb) % jacobian in the wolrd frame.

        J_n = jacobn(obj, q_j, stFltb) % jacobian in the tool frame.

        T0_n = T0_m(obj, q_j, lnk_name, stFltb) % ??

        payload(obj, pt_mass, pos, link_names)

        [M, C_qv, h_c] = wholeBodyDyn(obj, q_j, dq_j, stFltb)

        setupSim(obj, sim_config)

        visualizeFDyn(obj, x_out, sim_tstep, vis_ctrl)

        qjout = sortJointValue(obj, string_search, q_j, valueVector) % ??

        dispParams(obj, prec)

        % set.offset(obj, v)

        % v = get.offset(obj)

    end
end
