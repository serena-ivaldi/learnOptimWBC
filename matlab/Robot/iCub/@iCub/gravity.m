function g = gravity(rob,q)
%% CORIOLIS 
% ========================================================================= 
%    
%   g = gravity(rob,q) 
%   g = rob.gravity(q) 
%    
%  Description:: 
%    Given a full set of joint variables the function computes the 
%    gravity vector of the robot. 
%    
%  Input:: 
%    rob: robot object of LBR4p copy specific class 
%    q:  element vector of generalized 
%    
%  Output:: 
%    g:   gravity vector 
zero = zeros(size(q));
g    = wbm_generalizedBiasForces(rob.state.R_b,rob.state.x_b,q,zero,zeros(6,1));

end