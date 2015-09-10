% regularized version
function [tau,Jc,x,xd,rpy,rpyd] = ComputeRegularizedTorqueSum_iCub(obj, icub_params, ind_subchain, ind_task, M, gF, t, q_j, dq_j, u1)
   
    ndof = icub_params.ndof;
    try
       [~,~,x,xd,rpy,rpyd] = obj.subchains.DirKin(q_j, dq_j, ind_subchain, ind_task);
       
       % building up contraints jacobian and djdq
       Jc = zeros(6*icub_params.numConstraints, 6 + ndof);
       dJcDq = zeros(6*icub_params.numConstraints, 1);
       for i=1:icub_params.numConstraints
            Jc(6*(i-1) + 1:6*i, :) = wbm_jacobian(icub_params.constraintLinkNames{i});
            dJcDq(6*(i-1) + 1:6*i, :) = wbm_djdq(icub_params.constraintLinkNames{i});
       end
       
       [b,A] = TrajCostraint(obj, ind_subchain, ind_task, t, Jc, dJcDq, x, xd, rpy, rpyd, q_j', dq_j');

       % the hypothesis is that N is inverted directly from data
       N = evalin('caller',obj.metric{ind_subchain,ind_task});
       
       AM_inv  = A/M;
       AM_invN_AM_inv_T = AM_inv*N*(AM_inv)';
       reg_factr = obj.regularizer{ind_subchain}(1,ind_task);
       I_damp = eye(size(AM_invN_AM_inv_T,1));
       I = eye(size(q_j, 2)); 
       %DEBUG look for singularity proximity
    %   t
    %   zz = svd(AM_invN); 
       %---   
      
       t1 = (AM_inv)'/(reg_factr*I_damp + AM_invN_AM_inv_T);
       %tau = u1 + N*(AM_inv)'*t1*(b + AM_inv*(F + u1));
       %% TEST PROJECTION
       %(I - N*t1*AM_inv)*u1;
       tau = N*t1*(b + AM_inv*(gF)) + (I - N*t1*AM_inv)*u1;
       

    catch error
        rethrow(error);
    end

end