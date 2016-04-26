function G = gravload(rob,q)
%% GRAVLOAD - Computation of the configuration dependent vector of gravitational load forces/torques for LBR4p copy 
% ========================================================================= 
%    
%    G = gravload(rob,q) 
%    G = rob.gravload(q) 
%    
%  Description:: 
%    Given a full set of joint variables this function computes the 
%    configuration dependent vector of gravitational load forces/torques. 
%    
%  Input:: 
%    rob: robot object of LBR4p copy specific class 
%    q:  7-element vector of generalized 
%         coordinates 
%    Angles have to be given in radians! 
%    
%  Output:: 
%    G:  [7x1] vector of gravitational load forces/torques 
%  
qd = zeros(size(q));
G = wholeBodyModel('generalised-forces',reshape(rob.R_b,[],1),rob.x_b,q,qd,[rob.dx_b;rob.omega_b]);
end
