function [tau,J,x,xd,rpy,rpyd] = ComputeTorqueSum(obj,ind_subchain,ind_task,M,F,t,q,qd,Fc)
   
    try
       [J,Jd,x,xd,rpy,rpyd] = obj.subchains.DirKin(q,qd,ind_subchain,ind_task);
       % the hypothesis is that N is inverted directly from data ()
       N = evalin('caller',obj.metric{ind_subchain,ind_task});
       I = eye(size(M,1)); 
       %% primary controller
       [b,A] = TrajCostraint(obj,ind_subchain,ind_task,t,J,Jd,x,xd,rpy,rpyd,q',qd',Fc);
       M_inv =[];
       if(obj.subchains.floating_base)
           M_inv = M'/(M*M');
           AM_inv = A*M_inv;
       else
           AM_inv  = A/M;
       end
       
       AM_invN = AM_inv*N;
       t1 = pinv(AM_invN);
     
       %DEBUG look for singularity proximity
    %   t
    %   zz = svd(AM_invN); 
       %---   
        %% secondary controller
       if(strcmp(obj.Secondary_refs.type{ind_subchain,ind_task},'none')~=1)
          [b2,A2] = obj.TrajCostraintSecondary(ind_subchain,ind_task,t,J,Jd,x,xd,rpy,rpyd,q',qd',Fc);
          if(obj.subchains.floating_base)
              A2M_inv = A2*M_inv;
          else
              A2M_inv  = A2/M;
          end
          
          A2M_invN = A2M_inv*N;
          t2 = pinv(A2M_invN);
          u2 = N*t2*(b2);
       else
          u2 = zeros(size(M,1),1);
       end
       
       %% final controller
       %tau = u1 + N*t1*(b + AM_inv*(F + u1));
       
        % kathib 87 controller 
%         tau0_1 = A/M;
%         tau0 = tau0_1*A';
%         tau1 =A'/tau0;
%         tau = tau1*(b+tau0_1*F);
      tau = N*t1*(b) + ( I - N*t1*AM_inv)*u2;
      %% debug
      %torque_controller = tau
      %F_controller=F
      % debug 2
      %comparison1 =  N*t1*(b)
      %comparison2 = u1
      %tau = u1 + F;
      % debug
      %( I - N*t1*AM_inv)
      %( I - N*t1*AM_inv)*( I - N*t1*AM_inv)
      %AM_inv*( I - N*t1*AM_inv) = 0;
    catch error
        rethrow(error);
    end

end