function tau = ComputeTorqueSum(obj,index,M,F,t,q,qd)
    

   [J,Jd,x,xd,rpy,rpyd] = obj.subchains.DirKin(index,q,qd,obj.ground_truth);
   [b,A] = TrajCostraint(obj,index,t,J,Jd,x,xd,rpy,rpyd);
   
   N = evalin('caller',obj.metric{index});

   AM_inv  = A/M;
   AM_invN = AM_inv*N;      
   tau = N*pinv(AM_invN)*(b + AM_inv*F');
    
%     % kathib 87 controller 
%     tau0_1 = A/M;
%     tau0 = tau0_1*A';
%     tau1 =A'/tau0;
%     tau = tau1*(b+tau0_1*F');

end