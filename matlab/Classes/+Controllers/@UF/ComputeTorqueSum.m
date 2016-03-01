function [tau,J,x,xd,rpy,rpyd] = ComputeTorqueSum(obj,ind_subchain,ind_task,M,F,t,q,qd,Fc)
   
    try
       [J,Jd,x,xd,rpy,rpyd] = obj.subchains.DirKin(q,qd,ind_subchain,ind_task);
       % the hypothesis is that N is inverted directly from data ()
       N = evalin('caller',obj.metric{ind_subchain,ind_task});
       
       %% primary controller
       [b,A] = TrajCostraint(obj,ind_subchain,ind_task,t,J,Jd,x,xd,rpy,rpyd,q',qd',Fc);
       I = eye(size(q,2)); 
       AM_inv  = A/M;
       AM_invN = AM_inv*N;
       t1 = pinv(AM_invN);
     
       %DEBUG look for singularity proximity
    %   t
    %   zz = svd(AM_invN); 
       %---   
        %% secondary controller
       if(strcmp(obj.Secondary_refs.type{ind_subchain,ind_task},'empty')~=1)
          [b1,A1] = obj.TrajCostraintSecondary(ind_subchain,ind_task,t,J,Jd,x,xd,rpy,rpyd,q',qd',Fc);
          I = eye(size(q,2)); 
          A1M_inv  = A1/M;
          A1M_invN = A1M_inv*N;
          t2 = pinv(A1M_invN);
          u1 = N*t2*(b1);
       else
          u1 = zeros(size(q,2),1);
       end
       
       %% final controller
       %tau = u1 + N*t1*(b + AM_inv*(F + u1));
       
        % kathib 87 controller 
%         tau0_1 = A/M;
%         tau0 = tau0_1*A';
%         tau1 =A'/tau0;
%         tau = tau1*(b+tau0_1*F);

      % test for projector 
      tau = N*t1*(b) + ( I - N*t1*AM_inv)*u1 + F;
      % debug
      %torque_controller = tau
      %F_controller=F
    catch error
        rethrow(error);
    end

end