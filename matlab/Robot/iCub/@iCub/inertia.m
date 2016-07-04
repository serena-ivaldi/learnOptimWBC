function I = inertia(rob,q)
%% INERTIA - Inertia matrix for the LBR4p copy arm. 
% ========================================================================= 
%    
%    I = inertia(rob,q) 
%    I = rob.inertia(q) 
%    
%  Description:: 
%    Given a full set of joint variables the function computes the 
%    inertia Matrix of the robot. 
%    
%  Input:: 
%    rob: robot object of LBR4p copy specific class 
%    q:  n-element vector of generalized 
%         coordinates 
%    Angles have to be given in radians! 
%    
 
%    
I = wholeBodyModel('mass-matrix',reshape(rob.R_b,[],1),rob.x_b,q);
