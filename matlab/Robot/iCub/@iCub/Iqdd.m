function Iacc = Iqdd(rob,q,qd,tau)
%% IQDD - Vector of computed inertial forces/torques for Puma 560 copy 
% ========================================================================= 
%    
%    Iacc = Iqdd(rob,q,qd,tau) 
%    Iacc = rob.Iqdd(q,qd,tau) 
%    
%  Description:: 
%    Given a full set of joint variables, their temporal derivatives and applied joint forces/torques 
%    this function computes the reaction inertial forces/torques due to joint acceleration. 
%    
%  Input:: 
%    rob: robot object of Puma 560 copy specific class 
%    q:  6-element vector of generalized 
%         coordinates 
%    qd:  6-element vector of generalized 
%         velocities 
%    tau:  6-element vector of joint 
%         input forces/torques 
%    Angles have to be given in radians! 
%    
%  Output:: 
%    Iqdd:  [1x6] vector of inertial reaction forces/torques 
end