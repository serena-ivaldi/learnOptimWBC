function [tau,J,x,xd,rpy,rpyd] = ComputeTorqueSum_iCub(obj, ind_subchain, ind_task, M, F, t, q, qd, u1)
    try
        [J,Jd,x,xd,rpy,rpyd] = obj.subchains.DirKin(q, qd, ind_subchain, ind_task);

        % begin comment:
        % --> this makes not raelly sense, since J and Jd are being calculated
        %     in the DirKin function.
        
        %[~,~,x,xd,rpy,rpyd] = obj.subchains.DirKin(q, qd, ind_subchain, ind_task);

        %[vqT_b,~,v_b,~] = obj.wbm_iCub.getState();
        % get the translation and the rotation matrix ...
        %[p_b, R_b] = WBM.utilities.frame2posRotm(vqT_b);

        % compute the Jacobian and the corresponding derivative based on the
        % default constraint (contact) link:
        %J = obj.wbm_iCub.jacobian(R_b, p_b, q);
        %Jd = obj.wbm_iCub.dJdq(R_b, p_b, q, qd, v_b);

        % end comment.

        [b, A] = TrajCostraint(obj, ind_subchain, ind_task, t, J, Jd, x, xd, rpy, rpyd, q', qd');

        % the hypothesis is that N is inverted directly from data ()
        N = evalin('caller',obj.metric{ind_subchain,ind_task});
        I = eye(size(q,2));
        AM_inv  = A/M;
        AM_invN = AM_inv*N;

        %DEBUG look for singularity proximity
        %   t
        %   zz = svd(AM_invN);
        %---

        t1 = pinv(AM_invN);
        %tau = u1 + N*t1*(b + AM_inv*(F + u1));

        % kathib 87 controller
        %         tau0_1 = A/M;
        %         tau0 = tau0_1*A';
        %         tau1 =A'/tau0;
        %         tau = tau1*(b+tau0_1*F);

        %% test for projector
        tau = N*t1*(b + AM_inv*(F)) + ( I - N*t1*AM_inv) * u1;
    catch error
        rethrow(error);
    end
end
