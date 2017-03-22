function T0_n = T0_n(rob,q,tag)
%% T0_7 - Forward kinematics for the LBR4p copy arm up to frame 7 of 7. 
% ========================================================================= 
%    
%    T = T0_n(rob,q) 
%    T = rob.T0_n(q) 
%  

    T0_n = wholeBodyModel('forward-kinematics',rob.R_b,rob.x_b,q,tag);
	%wbm_forwardKinematics(rob.R_b,rob.x_b,qj,rob.kinematic_chain_selector[rob.cur_chain])
  
end


   





