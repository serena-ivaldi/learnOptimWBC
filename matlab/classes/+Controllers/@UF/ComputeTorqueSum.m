function [tau,J,x,xd,rpy,rpyd] = ComputeTorqueSum(obj,ind_subchain,ind_task,M,F,t,q,qd,u1)
   
    try
       [J,Jd,x,xd,rpy,rpyd] = obj.subchains.DirKin(q,qd,ind_subchain,ind_task);
       [b,A] = TrajCostraint(obj,ind_subchain,ind_task,t,J,Jd,x,xd,rpy,rpyd);

       % the hypothesis is that N is inverted directly from data ()
       N = evalin('caller',obj.metric{ind_subchain,ind_task});
       
       AM_inv  = A/M;
       AM_invN = AM_inv*N;

       %DEBUG look for singularity proximity
    %   t
    %   zz = svd(AM_invN); 
       %---   
      
       t1 = pinv(AM_invN);
       tau = u1 + N*t1*(b + AM_inv*(F + u1));
       
        % kathib 87 controller 
%         tau0_1 = A/M;
%         tau0 = tau0_1*A';
%         tau1 =A'/tau0;
%         tau = tau1*(b+tau0_1*F);
    catch error
        rethrow(error);
    end

end