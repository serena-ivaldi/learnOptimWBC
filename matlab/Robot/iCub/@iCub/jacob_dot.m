function jacob_dot = jacob_dot(rob,q,dq,tag)
%% JACOB_DOT - Jdot for the LBR4p copy. 
% ========================================================================= 
%    
%    Jdot = jacob_dot(rob,q,qd) 
%    Jdot = rob.jacob_dot(q,qd) 
%    
%  Description:: 
%    Given a set of joint variables and joint velocities up to joint number the function 
%    computes the Jacobian derivatives multiplied by qd with respect to the base frame. 
%    
%  Input:: 
%    q: vector of generalized coordinates. 
%    Angles have to be given in radians!\n 
%    qd: vector of the derivatives of generalized coordinates. 
%    
%  Output:: 
%    Jdot: 6x1 vector equal to Jdot*qd. 
jacob_dot = wholeBodyModel('djdq',reshape(rob.R_b,[],1),rob.x_b,q,dq,[rob.dx_b;rob.omega_b],tag); 
%wbm_djdq(rob.R_b,rob.x_b,qj,dqj,[rob.dx_b;rob.omega_w],rob.kinematic_chain_selector[rob.cur_chain]));
