function [tau,J,x,xd,rpy,rpyd] = ComputeTorqueSum(obj,ind_subchain,ind_task,M,F,t,q,qd)
   
    try
       [J,Jd,x,xd,rpy,rpyd] = obj.subchains.DirKin(q,qd,ind_subchain,ind_task);
       [b,A] = TrajCostraint(obj,ind_subchain,ind_task,t,J,Jd,x,xd,rpy,rpyd);

     
       N = evalin('caller',obj.metric{ind_subchain,ind_task});


       
       AM_inv  = A/M;
       AM_invN = AM_inv*N;

       %DEBUG look for singularity proximity
    %   t
    %   zz = svd(AM_invN); 
       %---

       tau = N*pinv(AM_invN)*(b + AM_inv*F);

    %     % kathib 87 controller 
    %     tau0_1 = A/M;
    %     tau0 = tau0_1*A';
    %     tau1 =A'/tau0;
    %     tau = tau1*(b+tau0_1*F');
    catch error
        rethrow(error);
    end

end