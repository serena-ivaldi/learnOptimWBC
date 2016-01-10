% regularized version
function [tau,J,x,xd,rpy,rpyd] = ComputeRegularizedTorqueSum_iCub(obj, ind_subchain, ind_task, M, F, t, q, qd, u1)
      try
        %[J,Jd,x,xd,rpy,rpyd] = obj.subchains.DirKin(q, qd, ind_subchain, ind_task);
        [~,~,x,xd,rpy,rpyd] = obj.subchains.DirKin(q, qd, ind_subchain, ind_task);

        [vqT_b,~,v_b,~] = obj.wbm_iCub.getState();
        % get the translation and the rotation matrix ...
        [p_b, R_b] = WBM.utilities.frame2posRotm(vqT_b);

        % compute the Jacobian and the corresponding derivative based on the
        % default constraint (contact) link:
        J = obj.wbm_iCub.jacobian(R_b, p_b, q);
        Jd = obj.wbm_iCub.dJdq(R_b, p_b, q, qd, v_b);

        [b, A] = TrajCostraint(obj, ind_subchain, ind_task, t, J, Jd, x, xd, rpy, rpyd, q', qd');

        % the hypothesis is that N is inverted directly from data
        N = evalin('caller',obj.metric{ind_subchain,ind_task});

        AM_inv  = A/M;
        AM_invN_AM_inv_T = AM_inv*N*(AM_inv)';
        reg_factr = obj.regularizer{ind_subchain}(1,ind_task);
        I_damp = eye(size(AM_invN_AM_inv_T,1));
        I = eye(size(q,2));
        %DEBUG look for singularity proximity
        %   t
        %   zz = svd(AM_invN);
        %---

        t1 = (AM_inv)'/(reg_factr*I_damp + AM_invN_AM_inv_T);
        %tau = u1 + N*(AM_inv)'*t1*(b + AM_inv*(F + u1));
        %% TEST PROJECTION
        %(I - N*t1*AM_inv)*u1;
        tau = N*t1*(b + AM_inv*(F)) + (I - N*t1*AM_inv)*u1;
    catch error
        rethrow(error);
    end
end
