function F = friction(rob,qd)
%% FRICTION - Joint friction for the LBR4p copy arm. 
% ========================================================================= 
%    
%    F = friction(rob,qd) 
%    F = rob.friction(qd) 
%    
%  Description:: 
%    Given a full set of generalized joint velocities the function 
%    computes the friction forces/torques. 
%    
%  Input:: 
%    rob: robot object of LBR4p copy specific class 
%    qd:  7-element vector of generalized 
%         velocities 
%    Angles have to be given in radians! 
%    
%  Output:: 
%    F:  [7x1] vector of joint friction forces/torques 
%    
end




