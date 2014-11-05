function tau = ComputeTorqueSum(obj,index,M,M_inv,F,t,q,qd)
    

   [J,Jd,x,xd,rpy,rpyd] = obj.subchains.DirKin(index,q,qd,obj.ground_truth);
   [b,A] = TrajCostraint(obj,index,t,J,Jd,x,xd,rpy,rpyd);
   
   N = evalin('caller',obj.metric(index));
   
   N_pow_half = N^(1/2);
   AM  = A*M_inv;
   AMN = AM*N_pow_half;
   
   tau = N_pow_half*pinv(AMN)*(b-AM*F);

end