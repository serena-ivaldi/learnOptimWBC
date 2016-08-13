classdef (Abstract) IWBM < handle
    properties(Abstract, Dependent)
        % public properties for fast get/set methods:
        model_name@char
        robot_name@char
        robot_manuf@char
        robot_params@WBM.wbmBaseRobotParams
        sim_config@WBM.absSimConfig
        base_tform@double matrix
        tool_tform@double matrix
        gravity@double    vector
        jlimits@struct
        ndof@uint16       scalar
    end

    % are they useful for the iCub?
    % offset     kinematic joint coordinate offsets (Nx1)
    % theta       kinematic: joint angles (1xN)
    % d           kinematic: link offsets (1xN)
    % a           kinematic: link lengths (1xN)
    % alpha       kinematic: link twists (1xN)

    methods(Abstract)
        initRobot(obj, robot_wbm)

        initRobotFcn(obj, fhInitRobotWBM, wf2fixLnk)

        initBaseRobotParams(obj, base_params)

        delete(obj)

        [vqT_b, q_j, v_b, dq_j] = getState(obj)

        stFltb = getFloatingBaseState(obj)

        % I_acc = Iqdd(obj, q_j, dq_j, tau, stFltb) % ?? inertial forces? (tau = generalized bias forces?)

        ddq_j = jointAccelerations(obj, q_j, dq_j, tau, stFltb)

        tau_c = coriolisForces(obj, q_j, dq_j, stFltb)

        tau_fr = frictionForces(obj, dq_j)

        C_qv = generalizedBiasForces(obj, q_j, dq_j, stFltb)

        tau_gen = generalizedForces(obj, q_j, dq_j, f_c, Jc_t, stFltb)

        tau_g = gravityForces(obj, q_j, stFltb)

        tau_ctrl = inverseDyn(obj, lnk_name, q_j, dq_j, ddq_j, stFltb)

        [t, stmChi] = forwardDyn(obj, tspan, fhCtrlTrqs, stvChi_0, ode_opt)

        visualizeForwardDyn(obj, x_out, sim_tstep, vis_ctrl)

        w_H_rlnk = forwardKin(obj, lnk_name, q_j, stFltb)

        % wf_H_rlnk = T0_n(obj, lnk_name, q_j, stFltb) % ?? computes the forward kinematics for the end-effector? do we need it?

        wf_H_rlnk = linkFrame(obj, jnt_idx, q_j, stFltb) % link transformation matrix
        % lnk_tform = linkFrame(obj, jnt_idx, q_j) % link transformation matrix

        wf_H_tp = toolFrame(obj, t_idx, q_j, stFltb) % tool-tip transformation matrix

        M = inertia(obj, q_j, stFltb)

        % M_x = cartesianInertia(obj, q_j, stFltb) % ?? useful for the iCub?

        h_c = centroidalMomentum(obj, q_j, dq_j, stFltb)

        [M, C_qv, h_c] = wholeBodyDyn(obj, q_j, dq_j, stFltb)

        J = jacobian(obj, lnk_name, q_j, stFltb)

        dJ = jacobianDot(obj, lnk_name, q_j, dq_j, stFltb)

        J_tp = jacobianTool(obj, t_idx, q_j, stFltb) % Jacobian matrix in tool frame (end-effector frame)

        payload(obj, pt_mass, pos, link_names)

        % tau_pl = payloadForces(obj, q_j, dq_j, stFltb)

        % paycap() % ??

        % gravjac() % ??

        resv = islimit(obj, q_j)

        dispParams(obj, prec)

        % set.offset(obj, v) % useful for the iCub?

        % v = get.offset(obj)

    end
end
