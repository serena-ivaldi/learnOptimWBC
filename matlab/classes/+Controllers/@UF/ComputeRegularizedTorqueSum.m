% regularized version
function [tau,J,x,xd,rpy,rpyd] = ComputeRegularizedTorqueSum(obj,ind_subchain,ind_task,M,F,t,q,qd,u1)
   
    try
       [J,Jd,x,xd,rpy,rpyd] = obj.subchains.DirKin(q,qd,ind_subchain,ind_task);
       [b,A] = TrajCostraint(obj,ind_subchain,ind_task,t,J,Jd,x,xd,rpy,rpyd,q',qd');

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
       tau = N*t1*(b + AM_inv*(F)) + (I - N*t1*AM_inv)*u1;
       

    catch error
        rethrow(error);
    end

end