% regularized version
function [tau,J,x,xd,rpy,rpyd] = ComputeRegularizedTorqueSum(obj,ind_subchain,ind_task,M,F,t,q,qd,Fc)
   
    try
       [J,Jd,x,xd,rpy,rpyd] = obj.subchains.DirKin(q,qd,ind_subchain,ind_task);
       % the hypothesis is that N is inverted directly from data
       N = evalin('caller',obj.metric{ind_subchain,ind_task});
       
       %% primary controller
       [b,A] = obj.TrajCostraint(ind_subchain,ind_task,t,J,Jd,x,xd,rpy,rpyd,q',qd',Fc);
       AM_inv  = A/M;
       %det(AM_inv)
       AM_invN_AM_inv_T = AM_inv*N*(AM_inv)';
       reg_factr = obj.regularizer{ind_subchain}(1,ind_task);
       I_damp = eye(size(AM_invN_AM_inv_T,1));
       I = eye(size(q,2));
       t1 = (AM_inv)'/(reg_factr*I_damp + AM_invN_AM_inv_T);  
       %DEBUG look for singularity proximity
    %   t
    %   zz = svd(AM_invN); 
       %---   
       %% secondary controller
       if(strcmp(obj.Secondary_refs.type{ind_subchain,ind_task},'none')~=1)
          [b2,A2] = obj.TrajCostraintSecondary(ind_subchain,ind_task,t,J,Jd,x,xd,rpy,rpyd,q',qd',Fc);
          A2M_inv  = A2/M;
          A2M_invN_A2M_inv_T = A2M_inv*N*(A2M_inv)';
          reg_factr = obj.regularizer{ind_subchain}(1,ind_task);
          I_damp = eye(size(A2M_invN_A2M_inv_T,1));
          I = eye(size(q,2)); 
          t2 = (A2M_inv)'/(reg_factr*I_damp + A2M_invN_A2M_inv_T);
          u2 = N*t2*(b2);
       else
          u2 = zeros(size(q,2),1);
       end
       %% final controller
       %tau = u1 + N*(AM_inv)'*t1*(b + AM_inv*(F + u1));
       % TEST PROJECTION
       %(I - N*t1*AM_inv)*u1;
       tau = N*t1*(b) + (I - N*t1*AM_inv)*u2;
    catch error
        rethrow(error);
    end

end