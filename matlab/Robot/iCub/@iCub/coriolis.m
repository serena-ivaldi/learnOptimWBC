function F = coriolis(rob,q,qd)
%% CORIOLIS 
% ========================================================================= 
%    
%    Crow = coriolis(rob,q,qd) 
%    Crow = rob.coriolis(q,qd) 
%    
%  Description:: 
%    Given a full set of joint variables and their first order temporal derivatives the function computes the 
%    Coriolis matrix of the robot. 
%    
%  Input:: 
%    rob: robot object of LBR4p copy specific class 
%    qd: element vector of generalized 
%    q:  element vector of generalized 
%    
%  Output:: 
%    C:   Coriolis matrix 
zero = zeros(size(q));
G = wholeBodyModel('generalised-forces',rob.R_b,rob.x_b,q,zero,[rob.dx_b;rob.omega_w]);
F_tot = wholeBodyModel('generalised-forces',rob.R_b,rob.x_b,q,qd,[rob.dx_b;rob.omega_w]);
F = F_tot - G;
%wbm_generalisedBiasForces(rob.R_b,rob.x_b,q,qd,[rob.dx_b;rob.omega_w]);


end
