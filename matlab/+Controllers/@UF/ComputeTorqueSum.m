function tau = ComputeTorqueSmu(obj,index,M_inv,F,t,q,qd,x,xd,rpy,rpyd)
      
   [b,A] = TrajCostraint(obj,index,t,q,qd,x,xd,rpy,rpyd);
   
   N_pow_half = obj.N(:,:,index)^(1/2);
   AM  = A*M_inv;
   AMN = AM*N_pow_half;
   
   tau = N_pow_half*pinv(AMN)*(b-AM*F);

end