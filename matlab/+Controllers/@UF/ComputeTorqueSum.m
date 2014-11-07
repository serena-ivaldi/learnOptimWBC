function tau = ComputeTorqueSum(obj,index,M,F,t,q,qd)
    

   [J,Jd,x,xd,rpy,rpyd] = obj.subchains.DirKin(index,q,qd,obj.ground_truth);
   [b,A] = TrajCostraint(obj,index,t,J,Jd,x,xd,rpy,rpyd);
   
  % N = evalin('caller',obj.metric{index});

   % eventually if the computation is to slow or i have problem i can use
   % "\"
   %N_pow_half = N^(-1/2);
%   AM_inv  = A*M_inv;
%   AM_invN = AM_inv*N_pow_half;

%DEBUG     
%    Minvers=M_inv   
%    pinversAM_invN=pinv(AM_invN)
%    coriolis_grav = F
%    control = b
%    matA=A
%---
   
    
   %tau = N_pow_half*pinv(AM_invN)*(b + AM_inv*F');
    
    
    tau0_1 = A/M;
    tau0 = tau0_1*A';
    tau1 =A'/tau0;
    tau = tau1*(b+tau0_1*F');

end