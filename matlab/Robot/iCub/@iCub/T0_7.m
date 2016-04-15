function T0_7 = T0_7(rob,q)
%% T0_7 - Forward kinematics for the LBR4p copy arm up to frame 7 of 7. 
% ========================================================================= 
%    
%    T = T0_7(rob,q) 
%    T = rob.T0_7(q) 
%  

    T0_7 = wholeBodyModel('forward-kinematics',rob.R_b,rob.x_b,q,rob.kinematic_chain_selector(rob.cur_chain));
	%wbm_forwardKinematics(rob.R_b,rob.x_b,qj,rob.kinematic_chain_selector[rob.cur_chain])
  
end


   





